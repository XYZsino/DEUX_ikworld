"""Offline replay only. No rclpy, publishers, action clients or hardware commands."""
import json,sys,copy,types
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path('tools').resolve()))
from async_upper import spline,validate
from deux_task_core.live_continuous_preview import roi_contains
from deux_task_core.configuration import load_configuration
from deux_task_core.model import RobotModel
from deux_task_core.conveyor import forecast
from deux_task_core.conveyor_screen import clearance
from deux_task_core.position_first_preview import bounded_joint_velocity
import deux_task_core.continuous_grasp_preview as original
# Match the newly installed boundary slowdown, without editing production descent.
source=Path(original.__file__).read_text().replace('wanted=np.clip(wanted,-r.velocity[14:21]*.98,r.velocity[14:21]*.98)','wanted=bounded_joint_velocity(r,q,wanted)')
ns={'__package__':'deux_task_core','bounded_joint_velocity':bounded_joint_velocity};exec(compile(source,'offline_descent_copy','exec'),ns);plan=ns['plan']
c,_=load_configuration('src/deux_task_bringup/config/installation.yaml');r=RobotModel(c['robot']);out=Path('.setup/descent-replay');out.mkdir(exist_ok=True)
trials=[x for x in json.loads(Path('.setup/moving-upper-trial/repeat-results.json').read_text()) if x.get('target_id','').endswith(('-78','-79'))]
assert len(trials)==2
(out/'source-trials.json').write_text(json.dumps(trials))
observations={x['target_id']:[] for x in trials}
for line in Path('.setup/icp-live-test/lane-predictions.jsonl').open():
 try:item=json.loads(line)
 except ValueError:continue
 for e in item.get('estimates',[]):
  if e.get('track_id') in observations:observations[e['track_id']].append((item['stamp'],e['position'],e['velocity']))
reports=[]
for prediction_mode in ['frozen_accepted_plan','refresh_before_descent']:
 for trial in trials:
  tid=trial['target_id'];sp=spline(trial);config=copy.deepcopy(c);config['grasp_object_from_right_ee']=trial['grasp'];proxy=types.SimpleNamespace(robot=r,config=config)
  v=np.array(trial['velocity']);obj0=np.array(trial['object_at_start']);grasp=np.array(trial['grasp']);obs=observations[tid]
  for state,t0 in [('approach_end_command',trial['approach_s']),('before_brake_command',trial['brake_start_s']),('stopped_measured',trial['times'][-1])]:
   q=sp(t0);qd=sp(t0,1)
   if state=='stopped_measured':
    q[np.r_[0:7,14:21]]+=np.radians(trial['final_error_deg']);qd[:]=0
   v=np.array(trial['velocity']);obj=forecast(obj0,v,t0);refresh_age=None
   if prediction_mode=='refresh_before_descent':
    past=[x for x in obs if x[0]<=trial['start_at']+t0]
    if not past or trial['start_at']+t0-past[-1][0]>.5:
     reports.append(dict(target=tid,branch=state,prediction_mode=prediction_mode,skipped='no fresh prior lane observation'));continue
    source=past[-1];refresh_age=trial['start_at']+t0-source[0];v=np.r_[source[2],0.];obj[:3,3]=np.array(source[1])+v[:3]*refresh_age
   for descent in [.5,1.,1.5,2.]:
    result=plan(proxy,q,qd,obj,v,descent_s=descent,close_s=.5)
    times=np.array(result['times']);path=np.array(result['path']);vel=np.array(result['velocities']);expected_total=descent+.5
    hard=[]
    if result['reason'] and result['reason'] not in ['position tracking residual','rotation tracking residual']:hard.append(result['reason'])
    margins=np.array([clearance(r,row) for row in path]);bad=np.flatnonzero(margins<0)
    if len(bad):hard.append('visible surface capsule clearance')
    # Validate complete cubic interpolation including a stop tail, not just knot samples.
    tail=max(.6,float(np.max(abs(vel[-1])/(.7*r.acceleration))));bt=np.linspace(0,tail,int(np.ceil(tail/.02))+1)[1:]
    checked=dict(times=np.r_[times,times[-1]+bt].tolist(),path=np.vstack([path,path[-1]+vel[-1]*(bt-bt*bt/(2*tail))[:,None]]).tolist(),velocities=np.vstack([vel,vel[-1]*(1-bt/tail)[:,None]]).tolist())
    try:checked_min=validate(config,checked,r)
    except ValueError as e:checked_min=None;hard.append('including stop: '+str(e))
    end_fk=r.ee_fk('right',path[-1]);end_obj=forecast(obj,v,float(times[-1]));target=end_obj@grasp
    contact_error=float(np.linalg.norm(end_fk[:3,3]-target[:3,3]));rotation=float(np.degrees(Rotation.from_matrix(target[:3,:3]@end_fk[:3,:3].T).magnitude()))
    at=trial['start_at']+t0+expected_total;actual=None
    if obs:
     earlier=[x for x in obs if x[0]<=at];later=[x for x in obs if x[0]>=at]
     if earlier and later and later[0][0]-earlier[-1][0]<=.5:
      left,right=earlier[-1],later[0];gap=right[0]-left[0];u=(at-left[0])/gap if gap>0 else 0.
      observed=(1-u)*np.array(left[1])+u*np.array(right[1])
      actual=dict(method='interpolated subsequent ICP-derived lane positions, not ground truth',bracket_gap_s=gap,forecast_error_m=float(np.linalg.norm(observed-forecast(obj,v,expected_total)[:3,3])),hand_to_observed_object_m=float(np.linalg.norm(end_fk[:3,3]-observed)))
    roi_valid=roi_contains(trial['geometry'],[forecast(obj,v,t)[:3,3] for t in np.linspace(0,expected_total,30)])
    report=dict(target=tid,branch=state,prediction_mode=prediction_mode,refresh_age_s=refresh_age,roi_valid=roi_valid,execution_eligible=(not hard and roi_valid),descent_start_s=t0,descent_s=descent,close_s=.5,completed_path=bool(abs(times[-1]-expected_total)<1e-8),hard_valid=not hard,hard_failures=hard,minimum_surface_margin_m=float(margins.min()),minimum_validated_margin_including_stop_m=checked_min,first_surface_failure_s=float(times[bad[0]]) if len(bad) else None,first_surface_failure_stage=('close' if times[bad[0]]>=descent else 'descent') if len(bad) else None,terminal_position_error_m=contact_error,terminal_rotation_error_deg=rotation,initial_height_above_predicted_object_m=float(r.ee_fk('right',q)[2,3]-obj[2,3]),future_observation_comparison=actual)
    reports.append(report);print(json.dumps(report),flush=True)
(out/'results.json').write_text(json.dumps(dict(offline_only=True,robot_motion_commands=0,notes=['Commanded state used for moving branches; measured joints reconstructed only at stopped endpoint','Straight constant-velocity belt forecast with recorded selected 0/180 grasp; refreshed mode uses only observations before branch start','0.5s closure is an arm-tracking interval, not simulated fingers or physical contact','Finite visible surfaces and provisional arm capsules; hidden geometry not reconstructed'],results=reports),indent=2))
