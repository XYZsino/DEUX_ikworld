"""Start camera/perception/monitor/grid PREVIEW; no arm/hand command process.

Never stops existing robot nodes or enables execution. Re-running reuses this
launcher's live processes and does not create duplicate camera/perception nodes.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

root = Path(__file__).resolve().parents[1]
os.chdir(root); out = root/'.setup/grid-grasp-test'; out.mkdir(parents=True, exist_ok=True)
record_file = out/'processes.json'
records = json.loads(record_file.read_text()) if record_file.exists() else {}
nodes = set(subprocess.check_output(['ros2', 'node', 'list'], text=True, timeout=20).splitlines())
commands = [
    ('camera', '/thor_calibrated_camera_tf', ['bash', str(root/'run_calibrated_zed.bash')]),
    ('perception', '/perception_node', ['ros2', 'run', 'deux_task_perception', 'tracked_perception_node', '--ros-args',
        '--params-file', str(root/'tracked_perception.thor.yaml'), '-p', 'config_file:='+str(root/'src/deux_task_bringup/config/installation.yaml')]),
    ('monitor', '/tracking_icp_monitor', [sys.executable, str(root/'tools/tracking_icp_monitor.py')]),
    ('grid-preview', '/grid_grasp_trial', [sys.executable, str(root/'tools/grid_grasp_trial.py')]),
]
for name, node, cmd in commands:
    previous = records.get(name, {})
    try:
        live = Path(f'/proc/{previous["pid"]}/cmdline').read_bytes().replace(b'\0', b' ').decode()
        if previous.get('identity') and previous['identity'] in live:
            print(name, 'already started', previous['pid']); continue
    except (KeyError, OSError): pass
    if node in nodes:
        print(name, 'existing external node reused:', node); continue
    with (out/(name+'.log')).open('ab') as log:
        process = subprocess.Popen(cmd, cwd=root, env=os.environ.copy(), stdin=subprocess.DEVNULL,
                                   stdout=log, stderr=log, start_new_session=True)
    # bash execs ros2 for the camera command; track the resulting launch filename.
    identity = 'calibrated_zed.thor.launch.py' if name == 'camera' else cmd[-1] if name in ('monitor', 'grid-preview') else 'tracked_perception_node'
    records[name] = dict(pid=process.pid, identity=identity, command=cmd, started_at=time.time())
    record_file.write_text(json.dumps(records, indent=2)); print(name, process.pid)
print('Preview only: /task/grid_grasp/preview; files:', out)
