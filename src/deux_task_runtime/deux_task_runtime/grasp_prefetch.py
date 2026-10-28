"""Prepare a timed grasp from the approach endpoint while the arm approaches.

This worker never executes motion. A matching later PlanMotion request must
verify measured joints, target innovation, scene and the original deadline.
"""
from copy import deepcopy
import time
import numpy as np
from scipy.spatial.transform import Rotation
from deux_task_core.conveyor import conveyor_velocity,forecast
from deux_task_core.time_candidates import predicted_std,settings
from .common import seconds,pose_matrix


class GraspPrefetch:
    def __init__(self,node):
        self.node=node;self.context=None

    def clear(self):
        if self.context and self.context.get('future') is not None:
            self.context['future'].cancel()
        self.context=None

    def arm(self,plan):
        self.clear()
        if not settings(self.node.config).get('prefetch_grasp',False):return
        self.context=dict(scene_version=plan.scene_version,target_id=plan.target_id,
            armed_at=self.node.now(),
            candidate_id=plan.grasp_candidate_id,approach_plan_id=plan.plan_id,
            approach_start=seconds(plan.trajectory.header.stamp),grasp_start=seconds(plan.grasp_start_at),
            phases=list(plan.grasp_phase_durations),endpoint=np.array(plan.trajectory.points[-1].positions),future=None)

    def poll(self):
        node=self.node;c=self.context
        if c is None:return
        now=node.now()
        if c['scene_version']!=node.config_version or now<c['armed_at'] or now>c['grasp_start']-.1:
            self.clear();return
        if c['future'] is not None or now<c['approach_start']:return
        try:
            obs=deepcopy(node.observation)
            obj=node.target(c['target_id'],obs,max_age=node.grasp_pose_age('TrackGrasp'))
            if not obj.pose_filtered or not obj.velocity_valid:return
            v=obj.velocity;velocity=conveyor_velocity([v.linear.x,v.linear.y,v.linear.z,v.angular.z],node.config).tolist()
            target=pose_matrix(obj.pose.pose);acquired=deepcopy(obs.objects.header.stamp)
            covariance=list(obj.motion_covariance) if obj.motion_covariance_valid else None
        except (ValueError,AttributeError):return
        from .motion_planner_node import compute
        c.update(obj=deepcopy(obj),target=target,acquired=acquired,velocity=velocity,covariance=covariance,
                 submitted_at=now,submitted_wall=time.monotonic())
        c['future']=node.pool.submit(compute,deepcopy(node.config),c['scene_version'],'TrackGrasp',
            c['endpoint'],target,None,(velocity,c['grasp_start']-seconds(acquired)),c['candidate_id'],
            dict(phase_durations=c['phases']),covariance)
        c['future'].add_done_callback(lambda _:c.update(completed_wall=time.monotonic()))
        node.get_logger().info(f'Grasp prefetch started during Approach: deadline={c["grasp_start"]:.3f}, target={c["target_id"]}')

    def take(self,request,start):
        self.poll();c=self.context
        if (c is None or c['future'] is None or c['scene_version']!=request.scene_version
                or c['target_id']!=request.target_id or c['candidate_id']!=request.grasp_candidate_id
                or abs(c['grasp_start']-seconds(request.grasp_start_at))>1e-6
                or len(request.grasp_phase_durations)!=3
                or not np.allclose(c['phases'],request.grasp_phase_durations,atol=1e-8,rtol=0)):
            raise ValueError('matching preplanned grasp unavailable; replan required')
        if np.max(abs(start-c['endpoint']))>self.node.config['start_tolerance']:
            raise ValueError('preplanned grasp endpoint differs from measured joints')
        # Consume once. An old plan can never authorize another task/request.
        self.context=None
        return c

    def guard(self,c,request,start):
        node=self.node;cfg=node.config
        if c['scene_version']!=node.config_version or not c['submitted_at']<=node.now()<=c['grasp_start']-.1:
            raise ValueError('preplanned grasp scene/deadline changed')
        if np.max(abs(start-c['endpoint']))>cfg['start_tolerance']:
            raise ValueError('preplanned grasp endpoint differs from measured joints')
        obs=request.observation
        if (node.observation is not None and node.observation.objects_valid
                and seconds(node.observation.objects.header.stamp)>=seconds(obs.objects.header.stamp)):
            obs=node.observation
        obj=node.target(c['target_id'],obs,max_age=node.grasp_pose_age('TrackGrasp'))
        if not obj.pose_filtered or not obj.velocity_valid:
            raise ValueError('preplanned grasp requires filtered motion evidence')
        stamp=seconds(obs.objects.header.stamp);age=stamp-seconds(c['acquired'])
        if not 0<=age<=cfg['conveyor_tracking']['max_plan_horizon']:
            raise ValueError('preplanned grasp acquisition superseded/expired')
        expected=forecast(c['target'],c['velocity'],age);actual=pose_matrix(obj.pose.pose)
        if np.linalg.norm(actual[:3,3]-expected[:3,3])>cfg['max_target_drift']:
            raise ValueError('preplanned grasp target innovation requires replan')
        if Rotation.from_matrix(actual[:3,:3]@expected[:3,:3].T).magnitude()>cfg.get('max_target_angle_rad',.15):
            raise ValueError('preplanned grasp orientation requires replan')
        v=obj.velocity;velocity=conveyor_velocity([v.linear.x,v.linear.y,v.linear.z,v.angular.z],cfg)
        finish=c['grasp_start']+sum(c['phases']);dt=finish-stamp
        old_finish=forecast(c['target'],c['velocity'],finish-seconds(c['acquired']))
        fresh_finish=forecast(actual,velocity,dt)
        if np.linalg.norm(old_finish[:3,3]-fresh_finish[:3,3])>cfg['max_target_drift']:
            raise ValueError('preplanned grasp closing forecast changed')
        if obj.motion_covariance_valid:
            std=predicted_std(obj.motion_covariance,dt,cfg)
            if std>settings(cfg)['max_prediction_std_m']:
                raise ValueError('preplanned grasp closing uncertainty exceeds bound')
