"""Post-capture FK and timestamp-aligned plots; never connected to robot control."""
import json,sys,csv
from pathlib import Path
from collections import Counter
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from deux_task_core.model import RobotModel,JOINT_NAMES,ARM_NAMES
root=Path('.setup/motion-alignment-audit');out=Path(sys.argv[1]) if len(sys.argv)>1 else Path((root/'latest.txt').read_text())
rows=[]
for line in (out/'samples.jsonl').read_text().splitlines():
 try:rows.append(json.loads(line))
 except ValueError:pass
cfg=json.loads((out/'config.json').read_text());robot=RobotModel(cfg['robot']);home=np.array(cfg['home_joints']);idx={n:JOINT_NAMES.index(n) for n in JOINT_NAMES}
statuses=[r for r in rows if r['kind']=='status'];active=[r for r in statuses if r['data']['active']]
if not active:raise SystemExit('No active trial captured; retain data and wait for a trial.')
t0=active[0]['received'];t1=active[-1]['received']+1
ids=[r['data']['target_id'] for r in active if r['data']['target_id']]
if not ids:raise SystemExit('No target selected during trial; no bag alignment can be evaluated.')
target=ids[-1]
t0=max(t0,next(r['received'] for r in active if r['data']['target_id']==target)-.5)
motion_file=out/'motion_result.json'
if not motion_file.exists():
 candidate=Path('.setup/moving-upper-trial/result.json')
 if candidate.exists():
  result=json.loads(candidate.read_text())
  if result.get('target_id')==target:motion_file.write_text(candidate.read_text())
if motion_file.exists():
 motion=json.loads(motion_file.read_text())
 if motion.get('target_id')==target:
  t0=max(t0,motion['start_at']-.3);t1=min(t1,motion['start_at']+motion['times'][-1]+.3)
pred=[];raw=[]
for r in rows:
 if r['kind'] not in ['pose','prediction']:continue
 for o in r['data']['objects']:
  if o['track']==target:(pred if r['kind']=='prediction' else raw).append(dict(stamp=r['data']['stamp'],received=r['received'],**o))
pred.sort(key=lambda x:x['received']);raw.sort(key=lambda x:x['stamp'])
series=[];pi=-1
for r in rows:
 if r['kind']!='controller':continue
 d=r['data'];t=d['stamp']
 if not t0<=t<=t1 or len(d['reference'])!=len(d['names']) or len(d['feedback'])!=len(d['names']):continue
 if not all(n in d['names'] for n in ARM_NAMES[7:]):continue
 command=home.copy();actual=home.copy()
 for n,a,b in zip(d['names'],d['reference'],d['feedback']):
  if n in idx:command[idx[n]]=a;actual[idx[n]]=b
 cp=robot.ee_fk('right',command)[:3,3];ap=robot.ee_fk('right',actual)[:3,3]
 while pi+1<len(pred) and pred[pi+1]['received']<=t:pi+=1
 bp=np.full(3,np.nan);age=float('nan');v=np.full(3,np.nan)
 if pi>=0:
  o=pred[pi];age=t-o['stamp']
  if o['valid'] and 0<=age<=1.:
   v=np.array(o['velocity']);bp=np.array(o['position'])+v*age
 series.append(np.r_[t,cp,ap,bp,age,v])
a=np.array(series)
if not len(a):raise SystemExit('No usable controller reference/feedback samples')
# Future raw observations are an audit reference only; never used to create bp.
audits=[]
for o in pred:
 if not o['valid']:continue
 for horizon in [.5,1.]:
  candidates=[r for r in raw if abs(r['stamp']-o['stamp']-horizon)<=.05]
  if not candidates:continue
  r=min(candidates,key=lambda r:abs(r['stamp']-o['stamp']-horizon));p=np.array(o['position'])+np.array(o['velocity'])*(r['stamp']-o['stamp'])
  audits.append(dict(horizon_s=horizon,xy_error_m=float(np.linalg.norm(p[:2]-np.array(r['position'])[:2]))))
cmd=a[:,1:4];act=a[:,4:7];bag=a[:,7:10];err=act-cmd;valid=np.isfinite(bag).all(axis=1)
vel=a[:,11:14];norm=np.linalg.norm(vel[:,:2],axis=1);direction=np.divide(vel[:,:2],norm[:,None],out=np.full((len(a),2),np.nan),where=norm[:,None]>.005)
along=np.sum((act[:,:2]-bag[:,:2])*direction,axis=1)
close=[];last=None
for r in rows:
 if r['kind']=='grip' and r['data'].get('valid') and t0<=r['received']<=t1:
  g=r['data']['grasp']
  if g!=last:
   close.append(dict(time=r['received'],grasp=g));last=g
fig,axs=plt.subplots(4,1,figsize=(13,12),sharex=True)
for j in range(3):
 axs[j].plot(a[:,0]-t0,cmd[:,j]*100,label='Controller reference',color='#3366cc')
 axs[j].plot(a[:,0]-t0,act[:,j]*100,label='Measured FK',color='#e08020')
 axs[j].plot(a[:,0]-t0,bag[:,j]*100,label='Causal bag prediction',color='#299450')
 if raw:axs[j].scatter([r['stamp']-t0 for r in raw if t0<=r['stamp']<=t1],[r['position'][j]*100 for r in raw if t0<=r['stamp']<=t1],s=8,color='#444444',alpha=.5,label='Raw ICP at acquisition')
 axs[j].set_ylabel('XYZ'[j]+' (cm)');axs[j].grid(alpha=.25)
axs[3].plot(a[:,0]-t0,np.linalg.norm(err,axis=1)*100,label='Actual-reference error (3D)')
axs[3].plot(a[:,0]-t0,along*100,label='Hand ahead of bag (+), behind (-)')
axs[3].set_ylabel('Error (cm)');axs[3].set_xlabel('Seconds from plotted window start');axs[3].grid(alpha=.25)
for ax in axs:
 for e in close:
  if e['grasp']>.99:ax.axvline(e['time']-t0,color='red',linestyle='--',label='Close command')
  elif abs(e['grasp']-.4)<.01:ax.axvline(e['time']-t0,color='purple',linestyle=':',label='Pregrasp 0.4 (not close)')
 ax.legend(loc='best',fontsize=8)
axs[0].set_title(target+' | base-frame, same controller timestamp | offline FK')
axs[-1].set_xlim(0,t1-t0);fig.tight_layout();fig.savefig(out/'comparison.png',dpi=140);plt.close(fig)
def stats(x):
 x=np.asarray(x);x=x[np.isfinite(x)]
 return dict(n=len(x),median=float(np.median(x)) if len(x) else None,p95=float(np.percentile(x,95)) if len(x) else None,max=float(np.max(x)) if len(x) else None)
summary=dict(target=target,controller_samples=len(a),causal_prediction_samples=int(valid.sum()),reference_vs_actual_cm=stats(np.linalg.norm(err,axis=1)*100),bag_vs_actual_xy_cm=stats(np.linalg.norm((act-bag)[:,:2],axis=1)*100),observation_age_s=stats(a[valid,10]),grip_events=close,closure_samples=[],future_raw_audit_cm={str(h):stats([r['xy_error_m']*100 for r in audits if r['horizon_s']==h]) for h in [.5,1.]},notes=['Controller header is update time, not verified physical CAN measurement time.','Raw ICP is not physical ground truth; shared calibration errors are not measured.','Causal bag estimates use only messages received before controller timestamp.','Grasp FK uses configured URDF frame, not a measured contact point.'])
for e in close:
 if e['grasp']>.99:
  i=int(np.argmin(abs(a[:,0]-e['time'])));summary['closure_samples'].append(dict(time=e['time'],controller_dt_s=float(a[i,0]-e['time']),reference=cmd[i].tolist(),actual=act[i].tolist(),bag=bag[i].tolist(),ahead_cm=float(along[i]*100),tracking_cm=float(np.linalg.norm(err[i])*100)))
(out/'summary.json').write_text(json.dumps(summary,indent=2));np.savetxt(out/'aligned.csv',a,delimiter=',',header='timestamp,cmd_x,cmd_y,cmd_z,actual_x,actual_y,actual_z,bag_x,bag_y,bag_z,observation_age,vx,vy,vz',comments='');print(json.dumps(summary,indent=2))
