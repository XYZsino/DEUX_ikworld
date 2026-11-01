"""Compare scheduled object positions with later raw ICP, never robot ground truth."""
import numpy as np
class PredictionAudit:
 def __init__(self):self.pending=[];self.previous={}
 def add(self,result):
  for stage,time_key,pose_key in [('contact','contact_at_s','object_pose'),('close','close_at_s','close_object_pose')]:
   self.pending.append(dict(target_id=result['target_id'],stage=stage,at=result[time_key],expected=np.asarray(result[pose_key])[:3,3].tolist()))
 def observe(self,identity,stamp,position):
  last=self.previous.get(identity);self.previous[identity]=(stamp,np.asarray(position));output=[];keep=[]
  for item in self.pending:
   if item['target_id']==identity and last and last[0]<=item['at']<=stamp and 0<stamp-last[0]<=.3:
    f=(item['at']-last[0])/(stamp-last[0]);actual=(1-f)*last[1]+f*np.asarray(position);error=np.asarray(item['expected'])-actual
    output.append(dict(**item,matched=True,error_xyz_m=error.tolist(),error_m=float(np.linalg.norm(error)),actual=actual.tolist()))
   elif stamp>item['at']+2.:output.append(dict(**item,matched=False,reason='no same-track ICP bracketing scheduled time'))
   else:keep.append(item)
  self.pending=keep;self.previous={k:v for k,v in self.previous.items() if stamp-v[0]<10.};return output
