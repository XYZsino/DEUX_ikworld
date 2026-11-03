"""Empty-bag recovery only: same certified geometry, bounded quintic timing."""
import numpy as np
from deux_task_core.model import RobotModel
from lookup_bt_support import prepare_plan

VELOCITY_CAP=.30
ACCELERATION_CAP=.25
JERK_CAP=.8


def smooth_empty_return(config,ready,actual,canceled=lambda:False,*unused):
    data=prepare_plan(config,ready,actual,canceled)
    robot=RobotModel(config['robot']);idx=np.r_[0:7,14:21]
    velocity=np.minimum(robot.velocity[idx],VELOCITY_CAP)
    acceleration=np.minimum(robot.acceleration[idx],ACCELERATION_CAP)
    times=[0.]
    for a,b in zip(data['path'][:-1],data['path'][1:]):
        delta=abs(b[idx]-a[idx])
        # Exact maxima of s(u)=10u^3-15u^4+6u^5 and its derivatives.
        duration=max(.25,float(np.max(1.875*delta/velocity)),
                     float(np.sqrt(np.max((10/np.sqrt(3))*delta/acceleration))),
                     float(np.cbrt(np.max(60*delta/JERK_CAP))))
        times.append(times[-1]+duration)
    data.update(times=np.asarray(times),smooth_empty_return=True,velocity_cap=VELOCITY_CAP,
                acceleration_cap=ACCELERATION_CAP,jerk_cap=JERK_CAP)
    return data
