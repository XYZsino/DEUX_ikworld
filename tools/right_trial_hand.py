"""Right hand only, reusing the production live-profile and mode checks."""
import time
from pathlib import Path
import numpy as np
from rclpy.parameter import Parameter
from deux_hand_msgs.msg import LeaderGrip
from deux_task_runtime.leader_grip_adapter import LeaderGripHandAdapter
from deux_task_runtime.common import current,pause

class RightTrialHand(LeaderGripHandAdapter):
 def __init__(self,node):
  root=Path(__file__).resolve().parents[1]
  for name,value in [('mapping_file',str(root/'src/deux_task_bringup/config/hand_mapping.yaml')),('hardware_enabled',True),('dry_run',False)]:
   if not node.has_parameter(name):node.declare_parameter(name,value)
   else:node.set_parameters([Parameter(name,value=value)])
  super().__init__(node)
  if 'left' in self.command_pubs:node.destroy_publisher(self.command_pubs.pop('left'))
  if self.virtual_right:raise ValueError('real right hand required')
 def guard(self):
  self.expire_disconnected_profiles();now=time.monotonic()
  if self.estop:raise ValueError('hand e-stop active')
  t,m=self.mode_state.get('right',(0,None))
  if m is None or now-t>2 or m.leader_mode!=self.mapping['right']['leader_mode'] or m.teleop_mode!=0 or not m.run_flag or m.stop_flag:raise ValueError('right hand requires confirmed mode3 / RUN / teleop off')
  if not self.profile_state.get('right',(0,False))[1]:raise ValueError('right live profile unverified/mismatched: '+self.profile_reason.get('right','not yet verified'))
  if now-self.last_external.get('right',-1e9)<1:raise ValueError('competing right grip command')
  if self.node.count_publishers('/hand/right/cmd'):raise ValueError('competing direct right hand command')
  if 'right' not in self.command_pubs or not self.command_pubs['right'].get_subscription_count():raise ValueError('right hand driver unavailable')
  sample=self.raw.get('right')
  if sample is None or not current(sample[0].stamp,self.node.now(),self.node.config['max_joint_age']):raise ValueError('right hand feedback stale/missing')
 def send(self,fraction):
  self.guard()
  if not np.isfinite(fraction) or not 0 <= fraction <= 1:raise ValueError('right hand fraction must be within [0,1]')
  m=LeaderGrip();m.header.stamp=self.node.get_clock().now().to_msg();m.header.frame_id=self.tag;m.side='right';m.grasp=fraction;m.valid=True
  self.command_pubs['right'].publish(m);self.last_command['right']=fraction
 async def open_ready(self,goal):
  end=time.monotonic()+3.;last=''
  while True:
   self.node.gate(goal)
   try:self.guard();break
   except ValueError as e:
    last=str(e)
    if time.monotonic()>end:raise ValueError(last)
    await pause(self.node,.02)
  fraction=float(getattr(self.node,'right_hand_ready_fraction',self.node.config.get('right_hand_ready_fraction',0.)))
  self.node.state='OpeningRightHand';self.node.reason=f'준비자세: 오른손 개폐값 {fraction} · 0.5초'
  start=self.node.now()
  while self.node.now()-start<.5:
   self.node.gate(goal);self.send(fraction);await pause(self.node,.02)
  self.send(fraction)
  return dict(command=fraction,duration_s=self.node.now()-start,measured_rad=list(self.raw['right'][1].values()))
