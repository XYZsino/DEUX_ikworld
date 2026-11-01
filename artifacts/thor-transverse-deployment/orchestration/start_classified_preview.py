import json,os,subprocess,time
from pathlib import Path
root=Path('/home/xyz_thor/DEUX_ikworld');out=root/'.setup/grid-grasp-classified-test'
for proc in Path('/proc').iterdir():
 if not proc.name.isdigit():continue
 try:
  args=(proc/'cmdline').read_bytes().decode().split('\0')
  if any(a.endswith(('/grid_grasp_trial.py','/roi_grasp_trial.py','/classified_grasp_trial.py')) for a in args):
   raise RuntimeError('existing motion server; refuse duplicate')
 except (OSError,UnicodeError):pass
env=os.environ.copy();env['ROS_DOMAIN_ID']='12'
command=['bash',str(root/'run_rotation_grasp_trial.bash')]
with (out/'preview-server.log').open('ab') as log:
 p=subprocess.Popen(command,cwd=root,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
record=dict(pid=p.pid,command=command,identity='tools/classified_grasp_trial.py',output_directory=str(out),
            started_at=time.time(),execution_enabled=False,action_requested=False)
(out/'process.json').write_text(json.dumps(record,indent=2));print(json.dumps(record))
