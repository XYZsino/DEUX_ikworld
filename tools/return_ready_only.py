"""One right-arm return to configured home; no tracking or hand commands."""
import json
import numpy as np
import rclpy
from descent_trial import GraspTrial
from ready_arm_trial import Trial,ROOT
from ready_recovery import compute_ready
from right_only_trial import planning_robot
from deux_task_core.planning import scene_version
from deux_task_runtime.common import pause
from deux_task_msgs.action import MoveToReady
class ReturnReadyOnly(GraspTrial):
 async def execute(self,goal):
  try:
   actual=await self.fresh_gate(goal);self.passive_left=actual[:7].copy()
   self.config['_stationary_left_arm']=self.passive_left.tolist()
   self.robot=planning_robot(self.config);self.config_version=scene_version(self.config)
   ready=np.array(self.config['home_joints']);self.target=ready[14:21].copy()
   q=ready.copy();q[self.idx]=actual;q[:7]=self.passive_left
   self.state='ReadyRecoveryPlanning';self.reason='수집용 준비자세 복귀 · 오른팔만 · 손 명령 없음'
   job=self.pool.submit(compute_ready,self.config,q.tolist(),ready.tolist())
   while not job.done():self.gate(goal);await pause(self,.03)
   path=job.result();out=ROOT/'.setup/conveyor-spline/ready-path.json';out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(path,indent=2));self.path_file=out
   return await Trial.execute(self,goal)
  except Exception as e:
   self.state='Stopped';self.reason=f'{type(e).__name__}: {e}'
   return MoveToReady.Result(success=False,reason=self.reason)
if __name__=='__main__':
 rclpy.init();n=ReturnReadyOnly()
 try:rclpy.spin(n)
 finally:n.pool.shutdown(wait=False,cancel_futures=True);n.destroy_node();rclpy.try_shutdown()
