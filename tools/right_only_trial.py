"""Right-only trial: retain measured left geometry without commanding left joints."""
import copy
import numpy as np
from deux_task_core.model import RobotModel, ARM_NAMES

def planning_robot(config):
    robot=RobotModel(config['robot'])
    stationary=config.get('_stationary_left_arm')
    if stationary is not None:
        q=np.asarray(stationary,dtype=float)
        if q.shape!=(7,) or not np.isfinite(q).all():
            raise ValueError('invalid stationary left pose')
        # This is a frozen obstacle pose, not a controllable joint range.
        # Installed URDF and all right-arm limits remain unchanged.
        robot.limits=robot.limits.copy()
        robot.limits[:7]=np.column_stack([q-1e-7,q+1e-7])
    robot.tracking_joint_reserve_rad=np.deg2rad(float(config.get("intercept_window",{}).get("min_nominal_joint_margin_deg",0.)))
    if not np.isfinite(robot.tracking_joint_reserve_rad) or robot.tracking_joint_reserve_rad<0:
        raise ValueError("invalid tracking joint reserve")
    return robot

class RightOnlyClient:
    def __init__(self,client):self.client=client
    def __getattr__(self,name):return getattr(self.client,name)
    def send_goal_async(self,goal,**kwargs):
        goal=copy.deepcopy(goal)
        tr=goal.trajectory
        if len(set(tr.joint_names))!=len(tr.joint_names):raise ValueError('duplicate joints')
        right=list(ARM_NAMES[7:])
        indices=[tr.joint_names.index(n) for n in right]
        left=[tr.joint_names.index(n) for n in ARM_NAMES[:7] if n in tr.joint_names]
        if left:
            baseline=np.array(tr.points[0].positions)[left]
            for p in tr.points:
                if not np.allclose(np.array(p.positions)[left],baseline,atol=1e-8,rtol=0):
                    raise ValueError('right-only plan moves left arm')
                for name in ('velocities','accelerations'):
                    values=getattr(p,name)
                    if values and np.max(abs(np.array(values)[left]))>1e-8:
                        raise ValueError('right-only plan moves left arm')
        for p in tr.points:
            for name in ('positions','velocities','accelerations','effort'):
                values=getattr(p,name)
                if values:setattr(p,name,[values[i] for i in indices])
        tr.joint_names=right
        return self.client.send_goal_async(goal,**kwargs)
