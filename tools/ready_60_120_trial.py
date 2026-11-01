"""One authorized right-arm ready motion; no tracking, descent or hand commands."""
import json,os
import numpy as np
import rclpy
from ready_arm_trial import Trial,ROOT
from right_only_trial import RightOnlyClient

class Ready60120(Trial):
 def __init__(self):
  super().__init__()
  self.target=np.array(json.loads((ROOT/'.setup/selected-ready.json').read_text())['chosen']['new_home'])[14:21]
  self.path_file=ROOT/os.environ.get('DEUX_READY_ROUTE','.setup/ready-60-120-live-route.json')
  self.target=np.array(json.loads(self.path_file.read_text())['right_waypoints'][-1])
  self.client=RightOnlyClient(self.client)
  self.reason='60~120도용 준비자세 · 검증된 현재 출발 경로 대기'

if __name__=='__main__':
 rclpy.init();node=Ready60120()
 try:rclpy.spin(node)
 finally:node.destroy_node();rclpy.shutdown()
