"""Exact-plan human review before any arm/hand dispatch."""
import hashlib
import json
import time
from uuid import uuid4
import numpy as np
from std_msgs.msg import String
from .common import LATCHED, pause, seconds, ros_time


class Ticket:
    def __init__(self, plan_id, digest, timeout, now=None):
        self.request_id=uuid4().hex
        self.plan_id=plan_id;self.digest=digest
        self.deadline=(time.monotonic() if now is None else now)+timeout
        self.decision=None

    def decide(self, data, now=None):
        now=time.monotonic() if now is None else now
        if (self.decision is not None or now>=self.deadline
                or data.get('request_id')!=self.request_id
                or data.get('plan_id')!=self.plan_id or data.get('digest')!=self.digest
                or data.get('decision') not in ('approve','reject')):
            return False
        self.decision=data['decision'];return True


def geometry(robot,q):
    fk=robot.fk(q);out={}
    for side in ('left','right'):
        links=[robot.joints[f'arm_{side[0]}_joint{i}']['child'] for i in range(1,8)]
        ee=fk[robot.ee[side]]
        out[side]={'points':[fk[name][:3,3].tolist() for name in links]+[ee[:3,3].tolist()],
                   'ee':ee.tolist()}
    return out


def preview(robot,plan,measured,t,q):
    # Digest covers the exact submitted trajectory, scene and target snapshot.
    from rclpy.serialization import serialize_message
    digest=hashlib.sha256(serialize_message(plan)).hexdigest()
    indices=np.unique(np.linspace(0,len(q)-1,min(60,len(q)),dtype=int))
    frames=[geometry(robot,q[i]) for i in indices]
    return dict(plan_id=plan.plan_id,digest=digest,stage=plan.stage,
                scene_version=plan.scene_version,target_id=plan.target_id,
                frame=robot.base,duration_s=float(t[-1]),sample_count=len(q),
                current=geometry(robot,measured),target=geometry(robot,q[-1]),
                path={s:[f[s]['ee'] for f in frames] for s in ('left','right')},
                frames=frames,times=[float(t[i]) for i in indices],
                hand_start=np.r_[q[0,7:14],q[0,21:28]].tolist(),
                hand_goal=np.r_[q[-1,7:14],q[-1,21:28]].tolist(),
                approvable=not plan.follows_conveyor,
                warning=('Moving conveyor schedule cannot wait for manual approval; stop belt and replan.'
                         if plan.follows_conveyor else 'Skeleton preview only; not collision or grasp verification.'))


class MotionReview:
    def __init__(self,node):
        self.node=node;self.ticket=None;self.data=None
        self.pub=node.create_publisher(String,'/task/motion_preview',LATCHED)
        self.sub=node.create_subscription(String,'/task/motion_decision',self.receive,10)

    def receive(self,msg):
        try:
            data=json.loads(msg.data)
            if self.ticket and isinstance(data,dict):self.ticket.decide(data)
        except (ValueError,TypeError):pass

    def publish(self,status,reason=''):
        if self.data:
            self.pub.publish(String(data=json.dumps(dict(self.data,status=status,reason=reason,
                remaining_s=max(0.,self.ticket.deadline-time.monotonic())),allow_nan=False)))

    async def wait(self,goal,robot,t,q):
        plan=goal.request.plan;node=self.node
        self.data=preview(robot,plan,node.measured(),t,q)
        self.ticket=Ticket(plan.plan_id,self.data['digest'],float(node.config.get('motion_review_timeout_s',300.)))
        self.data['request_id']=self.ticket.request_id
        previous=node.now();last=0.
        try:
            while True:
                if goal.is_cancel_requested:raise InterruptedError('motion review canceled')
                now=node.now()
                if now<previous:raise ValueError('clock reset during motion review')
                previous=now
                if node.config_version!=plan.scene_version:raise ValueError('scene changed during review')
                if time.monotonic()>=self.ticket.deadline:raise TimeoutError('motion review timed out')
                if self.ticket.decision=='reject':raise InterruptedError('operator rejected motion')
                if self.ticket.decision=='approve':
                    if not self.data['approvable']:raise ValueError(self.data['warning'])
                    # Stationary path is unchanged. Renew only its dispatch envelope;
                    # validate() below rechecks measured start, fresh target, original
                    # acquisition age, scene/model and barcode against this same plan.
                    plan.header.stamp=ros_time(now)
                    plan.valid_until=ros_time(now+node.config['max_plan_age'])
                    node.validate(goal.request)
                    if not node.get_parameter('dry_run').value:node.hardware_gate()
                    self.publish('approved','Revalidated; dispatching this path only')
                    return
                if time.monotonic()-last>=.2:
                    self.publish('pending');last=time.monotonic()
                await pause(node)
        except BaseException as error:
            self.publish('closed',str(error));raise
        finally:
            # Decisions cannot be replayed against another plan or request.
            self.ticket=None;self.data=None
