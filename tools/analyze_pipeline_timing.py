import json,statistics
from pathlib import Path
from collections import defaultdict

def read(p):
 out=[]
 if not p.exists():return out
 for l in p.read_text().splitlines():
  try:out.append(json.loads(l))
  except ValueError:pass
 return out

def stat(a):
 a=sorted(a)
 def q(x):
  if not a:return None
  k=(len(a)-1)*x;i=int(k);return a[i]+(a[min(i+1,len(a)-1)]-a[i])*(k-i)
 return dict(n=len(a),p50=q(.5),p95=q(.95),max=q(1))
def rate(ts):
 ts=sorted(set(ts));g=[(b-a)*1000 for a,b in zip(ts,ts[1:])]
 return dict(hz=(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 else None,period_ms=stat(g))
def icp_summary(rows):
 rows=[x for x in rows if x.get('estimator',{}).get('fits')]
 result=dict(attempts=len(rows),accepted=sum(bool(x.get('accepted')) for x in rows))
 for k in ['input_to_submit_ms','queue_wait_ms','worker_ms','completion_wait_ms','acquisition_to_finish_ms']:result[k]=stat([x[k] for x in rows if k in x])
 for k in ['roi_ms','fit_ms','filter_pack_ms']:result[k]=stat([x['estimator'][k] for x in rows if k in x['estimator']])
 for k in ['setup','coarse_search','refine','finalize']:
  result[k+'_ms']=stat([f['timing_ms'][k] for x in rows for f in x['estimator']['fits'] if k in f.get('timing_ms',{})])
 result['attempt_rate']=rate([x['acquisition_ns']/1e9+x['acquisition_to_finish_ms']/1000 for x in rows])
 result['accepted_rate']=rate([x['acquisition_ns']/1e9+x['acquisition_to_finish_ms']/1000 for x in rows if x.get('accepted')])
 return result
root=Path('.setup');historical=read(root/'icp-live-test/events.jsonl');live=read(root/'pipeline-timing/live.jsonl')
out={'latest_motion_icp':icp_summary([x for x in historical if 1793484400<=x.get('acquisition_ns',0)/1e9<=1793484407.4]),'live_icp':icp_summary([x['data'] for x in live if x['kind']=='icp'])}
for kind in ['detection','pose','prediction']:
 rows=[x for x in live if x['kind']==kind];out['live_'+kind]=dict(age_ms=stat([x['age_ms'] for x in rows]),rate=rate([x['received'] for x in rows]))
 groups=defaultdict(list)
 for row in rows:
  for track in row['tracks']:groups[track].append(row)
 out['live_'+kind]['tracks']={k:dict(n=len(v),age_ms=stat([r['age_ms'] for r in v]),rate=rate([r['received'] for r in v]),velocity_valid=sum(bool(r['valid']) for r in v)) for k,v in groups.items()}
r=json.loads((root/'moving-upper-trial/result.json').read_text());ev=r.get('async_replan',{}).get('events',[]);a=[x for x in ev if x.get('accepted')];out['last_motion_target']=r['target_id'];out['ik']={k:stat([x['planning_timing_ms'][k] for x in a]) for k in ['prepare','ik','assemble','validation','total']};out['ik']['wall_ms']=stat([x['compute_s']*1000 for x in a]);out['ik']['accepted_rate']=rate([x['time'] for x in a]);out['ik']['accepted']=len(a);out['ik']['rejected']=sum(x.get('accepted') is False for x in ev)
out['preparation_ms']=stat([x['compute_s']*1000 for x in read(root/'moving-upper-trial/entry-prediction.jsonl') if x.get('target_id')==r['target_id'] and x.get('stage') in ['rolling_prepared','preplanned_correction']]);out['seed_ik']=r.get('initial_seed_trial',{})
(root/'pipeline-timing/summary.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
