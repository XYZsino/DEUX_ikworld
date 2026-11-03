"""Real URDF synthetic replay; no ROS nodes, clients, or robot commands."""
import sys,json,time,copy
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path('tools').resolve()))
from deux_task_core.configuration import load_configuration
from right_only_trial import planning_robot
from intercept_window import compute_intercept
from async_upper import compute_replacement,spline
from deux_task_core.conveyor import forecast
c,_=load_configuration('src/deux_task_bringup/config/installation.yaml');c.update(tracking_grasp_policy=json.loads(Path('calibration/tracking_grasp_policy.json').read_text()),intercept_window=json.loads(Path('calibration/intercept_window.json').read_text()),intercept_approach_first=True,progressive_descent=True,tracking_lookahead_s=0.,upper_replan_horizon_s=.8)
q=np.array(c['home_joints']);c['_stationary_left_arm']=q[:7].tolist();r=planning_robot(c);T=r.ee_fk('right',q);g=np.array(c['grasp_object_from_right_ee']);obj=T@np.linalg.inv(g);obj[2,3]-=c['approach_height']
input=json.loads(Path('.setup/moving-upper-trial/latest-intercept-input.json').read_text());geom=input['geometry'];geom['roi']={'width':640,'height':360,'points':[[0,0],[639,0],[639,359],[0,359]]}
from initial_seed_trial import warm
print('warmup',warm(c),flush=True)
out=[]
for name,y in [('outside',.09),('inside',-.10)]:
 o=obj.copy();o[1,3]=y;v=np.array([0.,-.02,0.,0.]);cc=dict(c,_tracking_target_id='synthetic-'+name,_seed_config_version='offline')
 began=time.perf_counter()
 try:
  p=compute_intercept(cc,q,o,v,0.,geom);first_ms=(time.perf_counter()-began)*1000
  p.update(start_at=100.,target_id=name,source_stamp=100.,source_object=o.tolist())
  splice=.7
  repl=compute_replacement(dict(cc,_descent_admitted=False),p,o,v,100.,splice,1)
  grid=np.linspace(0,splice,80);a=spline(p);b=spline(repl)
  assert np.max(abs(a(grid)-b(grid)))<1e-9
  assert np.max(abs(a(grid,1)-b(grid,1)))<1e-9
  assert np.max(abs(b(repl['times'][-1],1)))<1e-9
  out.append(dict(scenario=name,passed=True,initial_ms=first_ms,initial_seed=p['initial_seed_trial'],workspace_follow=p['workspace_follow'],replacement_timing_ms=repl['planning_timing_ms']))
  Path('.setup/tracking-workspace-update/'+name+'-plan.json').write_text(json.dumps(p))
 except Exception as e:out.append(dict(scenario=name,passed=False,error=f'{type(e).__name__}: {e}',elapsed_s=time.perf_counter()-began))
Path('.setup/tracking-workspace-update/replay.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
assert all(x['passed'] for x in out)
