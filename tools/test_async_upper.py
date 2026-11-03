"""Offline tests. ROS_DOMAIN_ID=219; no action clients are constructed."""
import json,sys,time
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path('tools').resolve()))
from async_upper import compute_replacement,validate,spline,eligible
import sys,json,numpy as np
from pathlib import Path
sys.path.insert(0,str(Path('tools').resolve()))
from moving_upper_trial import compute
from deux_task_core.configuration import load_configuration
from deux_task_core.model import RobotModel,ARM_NAMES,JOINT_NAMES
from deux_task_runtime.trajectory import validate_limits
from deux_task_runtime.common import ros_time
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from builtin_interfaces.msg import Duration
c,_=load_configuration('src/deux_task_bringup/config/installation.yaml');r=RobotModel(c['robot']);q=np.array(json.loads(Path('.setup/selected-ready.json').read_text())['chosen']['new_home']);grasp=np.array(c['grasp_object_from_right_ee']);obj=r.ee_fk('right',q)@np.linalg.inv(grasp);obj[2,3]-=c['approach_height'];snap=json.loads(Path('.setup/roi-ready/snapshot.json').read_text());geom=dict(camera=json.loads(Path('calibration/zed2i_extrinsics.thor.json').read_text())['T_base_camera_optical'],projection=snap['P'],width=snap['width'],height=snap['height'],roi={'width':640,'height':360,'points':[[0,0],[639,0],[639,359],[0,359]]})

p=compute(c,q,obj,np.array([0,-.02,0,0]),3.5,geom)
p.update(start_at=100.,target_id='synthetic',source_stamp=96.5,source_object=obj.tolist())
began=time.monotonic();validate(c,p,r)
replacement=compute_replacement(c,p,obj+np.diag([0.,0.,0.,0.]),np.array([0,-.02,0,0]),96.5,.65,1)
old=spline(p);new=spline(replacement);grid=np.linspace(0,.65,101)
assert np.max(abs(old(grid)-new(grid)))<1e-10
assert np.max(abs(old(grid,1)-new(grid,1)))<1e-10
assert np.max(abs(new(replacement['times'][-1],1)))<1e-10
assert eligible(replacement,100.2,'synthetic',1)
assert not eligible(replacement,100.6,'synthetic',1)
assert not eligible(replacement,100.2,'other',1)
assert not eligible(replacement,100.2,'synthetic',2)
bad=dict(p);bad['velocities']=np.array(p['velocities']).tolist();bad['velocities'][5][14]=100.
try:validate(c,bad,r)
except ValueError:pass
else:raise AssertionError('spline overshoot must fail')
result=dict(synthetic=True,passed=True,compute_and_validation_s=time.monotonic()-began,prefix_position_max=float(np.max(abs(old(grid)-new(grid)))),prefix_velocity_max=float(np.max(abs(old(grid,1)-new(grid,1)))),splice_s=.65,terminal_speed=float(np.max(abs(new(replacement['times'][-1],1)))))
Path('.setup/async-upper-test.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
