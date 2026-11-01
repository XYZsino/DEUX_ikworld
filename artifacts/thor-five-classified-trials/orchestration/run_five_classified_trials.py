"""Exactly five operator-authorized trials; production normal/transverse routing."""
import datetime,json,os,signal,subprocess,sys,time,shutil
from pathlib import Path
root=Path('/home/xyz_thor/DEUX_ikworld');base=root/'.setup/grid-grasp-classified-test'
batch=base/('repeat-all-angles-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'));batch.mkdir()
env=os.environ.copy();env['ROS_DOMAIN_ID']='12'
state=dict(batch=str(batch),requested_trials=5,completed_trials=0,trial=0,phase='starting',results=[],transverse_only=False)
def update(**values):
 state.update(values);tmp=batch/'status.tmp';tmp.write_text(json.dumps(state,indent=2));tmp.replace(batch/'status.json')
 (base/'repeat-latest.json').write_text(json.dumps(dict(batch=str(batch),status=str(batch/'status.json')),indent=2))
 short={k:state[k] for k in ['batch','trial','phase','completed_trials','transverse_only']}
 for key in ['error','last_mode','last_cell']:
  if key in state:short[key]=state[key]
 with (batch/'events.jsonl').open('a') as log:log.write(json.dumps(dict(time_s=time.time(),**short))+'\n')
 print(json.dumps(short),flush=True)
def run(args,log):
 with log.open('w') as stream:
  result=subprocess.run(args,cwd=root,env=env,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT)
 if result.returncode:raise RuntimeError('step failed: '+str(args)+'; log='+str(log))
def stop_idle_server():
 record=json.loads((base/'process.json').read_text());previous=Path(record['output_directory'])
 deadline=time.monotonic()+5
 while json.loads((previous/'preview.json').read_text()).get('motion_requested'):
  if time.monotonic()>deadline:raise RuntimeError('existing action is still active; refuse overlap')
  time.sleep(.1)
 pid=record['pid']
 try:
  command=Path(f'/proc/{pid}/cmdline').read_bytes().decode().replace('\0',' ')
  if record['identity'] not in command:raise RuntimeError('server PID identity mismatch')
  os.kill(pid,signal.SIGINT)
 except (FileNotFoundError,ProcessLookupError):return
 until=time.monotonic()+5
 while time.monotonic()<until:
  try:
   if Path(f'/proc/{pid}/status').read_text().split('State:')[1].splitlines()[0].strip().startswith('Z'):return
  except FileNotFoundError:return
  time.sleep(.1)
 raise RuntimeError('old server did not stop')
try:
 for number in range(1,6):
  if (batch/'STOP').exists():raise InterruptedError('operator stopped before next trial')
  trial=batch/f'trial-{number:02d}';trial.mkdir()
  update(trial=number,phase='preparing',trial_directory=str(trial),selected=None,error=None)
  stop_idle_server()
  run([sys.executable,'tools/prepare_grid_ready.py','--execute','--log-errors'],trial/'ready-execution.log')
  ready=json.loads((root/'.setup/grid-grasp-test/ready-result.json').read_text())
  if not ready.get('success') or ready['final_max_error_rad']>.03:raise RuntimeError('ready did not settle within 0.03rad')
  shutil.copy2(root/'.setup/grid-grasp-test/ready-result.json',trial/'ready-result.json')
  run([sys.executable,'tools/configure_right_trial_hand.py'],trial/'hand-mode.log')
  args=['bash',str(root/'run_rotation_grasp_trial.bash'),'--ros-args','-p','enable_execution:=true',
   '-p','target_wait_timeout_s:=300.','-p','grid_output_directory:='+str(trial),
   '-p','stationary_left_file:='+str(base/'stationary-left.json')]
  with (trial/'server.log').open('w') as log:
   server=subprocess.Popen(args,cwd=root,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
  record=dict(pid=server.pid,command=args,output_directory=str(trial),identity='tools/classified_grasp_trial.py',
   batch=str(batch),trial=number,execution_enabled=True,transverse_only=False,started_at=time.time())
  (base/'process.json').write_text(json.dumps(record,indent=2));(trial/'process.json').write_text(json.dumps(record,indent=2))
  update(phase='waiting_for_hand_profile',server_pid=server.pid)
  deadline=time.monotonic()+120;preview={}
  while time.monotonic()<deadline:
   if server.poll() is not None:raise RuntimeError('execution server exited before admission')
   try:preview=json.loads((trial/'preview.json').read_text())
   except FileNotFoundError:pass
   if preview.get('execution_admission_ready'):break
   time.sleep(.1)
  else:raise RuntimeError('execution admission unavailable: '+json.dumps(preview))
  with (trial/'request.log').open('w') as log:
   client=subprocess.Popen([sys.executable,'tools/request_grid_grasp.py','--execute'],cwd=root,env=env,
    stdin=subprocess.DEVNULL,stdout=log,stderr=log)
  update(phase='waiting_for_object',request_pid=client.pid,ready_error_rad=ready['final_max_error_rad'])
  committed=False
  while client.poll() is None:
   try:
    preview=json.loads((trial/'preview.json').read_text())
    if preview.get('locked_cell_id') and not committed:
     committed=True;update(phase='committed',selected=preview)
   except FileNotFoundError:pass
   time.sleep(.1)
  if not (trial/'result.json').exists():raise RuntimeError('request failed before result; '+str(trial/'request.log'))
  result=json.loads((trial/'result.json').read_text());state['results'].append(result)
  shutil.copy2(root/'.setup/grid-grasp-test/request-result.json',trial/'request-result.json')
  if client.returncode or not result.get('success'):raise RuntimeError('trial did not complete: '+result.get('reason','unknown'))
  update(completed_trials=number,phase='trial_completed',last_mode=result.get('grasp_mode'),last_cell=result.get('cell_id'))
 update(phase='completed',success=True,no_automatic_sixth_trial=True,physical_grasp_verified=False)
except BaseException as error:
 update(phase='stopped',success=False,error=f'{type(error).__name__}: {error}')
 raise
