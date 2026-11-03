"""Start missing state publisher and require an admitted status before BT startup."""
import json
import os
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
import yaml

root = Path(__file__).resolve().parents[1]
config = yaml.safe_load((root / 'src/deux_task_bringup/config/installation.yaml').read_text())
rclpy.init()
node = rclpy.create_node('bt_state_startup_check')
received = []
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
def on_status(msg):
    try:
        received.append(json.loads(msg.data))
    except (ValueError, TypeError):
        pass
sub = node.create_subscription(String, '/state/status', on_status, qos)
def wait_status(seconds):
    end = time.monotonic() + seconds
    while not received and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

try:
    wait_status(3.0)
    if not received and node.count_publishers('/state/status') == 0:
        # Avoid duplicating an existing publisher process that is still starting.
        existing = subprocess.run(['pgrep', '-f',
            '[/]lib/deux_state/state_node|[r]os2 launch deux_state state.launch.py'],
            stdout=subprocess.DEVNULL).returncode == 0
        if not existing:
            state_file = os.environ.get('DEUX_STATE_FILE', '/home/xyz_thor/DEUX/deux_state.env')
            if not Path(state_file).is_file():
                raise RuntimeError(f'state file missing: {state_file}')
            with (root / '.setup/state-current.log').open('a') as log:
                subprocess.Popen(['ros2', 'launch', 'deux_state', 'state.launch.py',
                    f'state_file:={state_file}'], cwd=root, stdin=subprocess.DEVNULL,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                    close_fds=True)
            print('State publisher started; waiting for /state/status', flush=True)
    wait_status(12.0)
    if not received:
        raise RuntimeError('No /state/status received; see .setup/state-current.log')
    status = received[-1]
    if status.get('teleop') != 'off' or status.get('mode') not in config['allowed_state_modes']:
        raise RuntimeError(f'BT state not admitted: {status}; allowed modes={config["allowed_state_modes"]}, teleop=off required')
    print(f'BT state confirmed: {status}', flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
