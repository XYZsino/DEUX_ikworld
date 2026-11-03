"""Offline real-model ScanPose retiming; ROS message types, no nodes or commands."""
import unittest
from copy import deepcopy
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
from deux_task_msgs.msg import MotionPlan
from deux_task_runtime.common import ros_time,seconds
from deux_task_runtime.trajectory import validate_limits
from deux_task_core.planning import Planner
from deux_task_core.model import JOINT_NAMES
from grid_grasp_runtime import ROOT
from classified_grid_runtime import ClassifiedGridLibrary
from lookup_scan_timing import retime_scan


class ScanTimingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        p=ROOT/'src/deux_task_bringup/config/installation.yaml';cls.c=yaml.safe_load(p.read_text())
        cls.c['robot']['urdf_path']=str((p.parent/cls.c['robot']['urdf_path']).resolve())
        cls.lib=ClassifiedGridLibrary(cls.c,directory=ROOT/'artifacts/right_grasp_grid_classified',
            recommended=False,yaw_secondary=True,load_rotation_paths=False)
    def plan(self,yaw):
        pose=np.eye(4);pose[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix();pose[:3,3]=[.6,.5,.97]
        selection=self.lib.select(pose,np.zeros(4),100.,100.)
        start=self.lib.ready.copy();start[:7]=self.c['home_joints'][:7];start[14:21]=self.lib.curves[selection['cell_id']](5.)
        c=deepcopy(self.c);c['ik_backend']='scipy';planner=Planner(c);data=planner.plan('ScanPose',start,np.eye(4))
        result=MotionPlan(stage='ScanPose',plan_id='retiming-check',scene_version='unchanged')
        result.trajectory.joint_names=list(JOINT_NAMES)
        result.trajectory.points=[JointTrajectoryPoint(positions=q.tolist(),time_from_start=ros_time(float(t),Duration)) for t,q in zip(data['times'],data['path'])]
        return result,planner.robot
    def test_faster_scan_preserves_path_and_all_joint_limits(self):
        for yaw in (0.,90.,-90.):
            plan,robot=self.plan(yaw);original=deepcopy(plan)
            fast,report=retime_scan(plan,self.c,robot,.5)
            self.assertEqual(plan,original);self.assertEqual(fast.plan_id,plan.plan_id);self.assertEqual(fast.scene_version,plan.scene_version)
            self.assertEqual([list(p.positions) for p in fast.trajectory.points],[list(p.positions) for p in plan.trajectory.points])
            self.assertAlmostEqual(report['duration_s']/report['original_duration_s'],.7)
            validate_limits(fast.trajectory,JOINT_NAMES,robot)
    def test_rejects_retiming_grasp_or_exceeding_limits(self):
        plan,robot=self.plan(0.)
        for scale in (.1,1.01):
            with self.assertRaises(ValueError):retime_scan(plan,self.c,robot,scale)
        plan.stage='TrackGrasp'
        with self.assertRaises(ValueError):retime_scan(plan,self.c,robot,.5)


if __name__=='__main__':unittest.main()
