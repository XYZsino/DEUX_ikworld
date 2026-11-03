"""Bounded, passive execution trace; disk writes happen after arm cancellation."""
from collections import deque
from pathlib import Path
import csv,json,time
import numpy as np
from deux_task_core.model import ARM_NAMES,JOINT_NAMES

class TrackingTrace:
    def __init__(self, plan, t, q, start, limit, capacity=30000):
        self.indices=[JOINT_NAMES.index(n) for n in ARM_NAMES]
        self.t=np.asarray(t);self.q=np.asarray(q)[:,self.indices];self.start=start
        self.rows=deque(maxlen=capacity);self.total=0
        self.meta=dict(plan_id=plan.plan_id,stage=plan.stage,scene_version=plan.scene_version,
                       scheduled_start_ros_s=start,tracking_limit_rad=limit,joints=list(ARM_NAMES),
                       velocity_note='planned velocity is slope of sampled position path; measured velocity is driver JointState, null if missing')
    def append(self, now, measured, expected, state):
        elapsed=now-self.start;i=int(np.clip(np.searchsorted(self.t,elapsed,side='right')-1,0,len(self.t)-2))
        velocity=(self.q[i+1]-self.q[i])/(self.t[i+1]-self.t[i]) if 0<=elapsed<self.t[-1] else np.zeros(len(ARM_NAMES))
        stamp=None;v={}
        if state is not None:
            stamp=state.header.stamp.sec+state.header.stamp.nanosec*1e-9
            v=dict(zip(state.name,state.velocity))
        actual=np.asarray(measured)[self.indices];target=np.asarray(expected)[self.indices]
        self.rows.append(dict(ros_time_s=now,elapsed_s=elapsed,feedback_stamp_s=stamp,
                              feedback_age_s=now-stamp if stamp is not None else None,
                              expected_rad=target.tolist(),measured_rad=actual.tolist(),error_rad=(actual-target).tolist(),
                              planned_velocity_rad_s=velocity.tolist(),measured_velocity_rad_s=[float(v[n]) if n in v and np.isfinite(v[n]) else None for n in ARM_NAMES]))
        self.total+=1
    def save(self, directory, outcome):
        root=Path(directory);root.mkdir(parents=True,exist_ok=True)
        name=f'{time.time_ns()}-{self.meta["stage"]}-{self.meta["plan_id"]}'
        dest=root/(name+'.json');rows=list(self.rows)
        data=dict(self.meta,outcome=outcome,samples_total=self.total,samples_dropped=self.total-len(rows),
                  trajectory_times_s=self.t.tolist(),trajectory_positions_rad=self.q.tolist(),samples=rows)
        dest.write_text(json.dumps(data,allow_nan=False))
        j=ARM_NAMES.index('arm_r_joint3')
        with (root/(name+'-j3.csv')).open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['elapsed_s','feedback_age_s','expected_deg','measured_deg','error_deg','planned_velocity_deg_s','measured_velocity_deg_s'])
            for row in rows:
                vals=[row[k][j] for k in ['expected_rad','measured_rad','error_rad','planned_velocity_rad_s','measured_velocity_rad_s']]
                w.writerow([row['elapsed_s'],row['feedback_age_s']]+[None if v is None else float(np.degrees(v)) for v in vals])
        return dest
