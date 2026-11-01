"""Operator-authorized exactly three additional real one-shot trials."""
import datetime,json,os,signal,subprocess,sys,time,shutil
from pathlib import Path
root=Path('/home/xyz_thor/DEUX_ikworld');base=root/'.setup/grid-grasp-rotation-test'
batch=base/('repeat-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'));batch.mkdir()
env=os.environ.copy();env['ROS_DOMAIN_ID']='12'
state=dict(batch=str(batch),requested_trials=3,completed_trials=0,phase='starting',trial=0,results=[])
def update(**values):
 state.update(values);temp=batch/'status.tmp';temp.write_text(json.dumps(state,indent=2));temp.replace(batch/'status.json')
 (base/'repeat-latest.json').write_text(json.dumps(dict(batch=str(batch),status=str(batch/'status.json')),indent=2))
 print(json.dumps({k:v for k,v in state.items() if k!='results'}),flush=True)
def run(args,log):
 with log.open('w') as stream:
  result=subprocess.run(args,cwd=root,env=env,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT)
 if result.returncode:raise RuntimeError('step failed: '+str(args)+'; log='+str(log))
def stop_idle():
 record=json.loads((base/'process.json').read_text());previous=Path(record.get('output_directory',str(base)))
 deadline=time.monotonic()+3
 while json.loads((previous/'preview.json').read_text()).get('motion_requested'):
  if time.monotonic()>deadline:raise RuntimeError('previous action still active; refuse overlapping trial')
  time.sleep(.1)
 pid=record['pid']
 try:
  cmd=Path(f'/proc/{pid}/cmdline').read_bytes().decode().replace('\0',' ')
  if 'tools/grid_grasp_trial.py' not in cmd:raise RuntimeError('server PID identity changed')
  os.kill(pid,signal.SIGINT)
 except ProcessLookupError:return
 until=time.monotonic()+5
 while time.monotonic()<until:
  try:
   if Path(f'/proc/{pid}/status').read_text().split('State:')[1].splitlines()[0].strip().startswith('Z'):return
  except FileNotFoundError:return
  time.sleep(.1)
 raise RuntimeError('old server did not stop')
try:
 for number in range(1,4):
  if (batch/'STOP').exists():raise InterruptedError('operator requested stop before next trial')
  trial=batch/f'trial-{number:02d}';trial.mkdir()
  update(trial=number,phase='preparing',trial_directory=str(trial))
  stop_idle()
  run([sys.executable,'tools/prepare_grid_ready.py','--execute','--log-errors'],trial/'ready-execution.log')
  ready=json.loads((root/'.setup/grid-grasp-test/ready-result.json').read_text())
  if not ready.get('success') or ready['final_max_error_rad']>.03:raise RuntimeError('ready did not settle')
  shutil.copy2(root/'.setup/grid-grasp-test/ready-result.json',trial/'ready-result.json')
  run([sys.executable,'tools/configure_right_trial_hand.py'],trial/'hand-mode.log')
  args=['bash',str(root/'run_rotation_grasp_trial.bash'),'--ros-args','-p','enable_execution:=true',
        '-p','target_wait_timeout_s:=300.','-p','stationary_left_file:='+str(base/'stationary-left.json'),
        '-p','grid_output_directory:='+str(trial)]
  with (trial/'server.log').open('w') as log:
   server=subprocess.Popen(args,cwd=root,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
  record=dict(pid=server.pid,command=args,identity='tools/grid_grasp_trial.py',started_at=time.time(),
              execution_enabled=True,output_directory=str(trial),batch=str(batch),trial=number)
  (base/'process.json').write_text(json.dumps(record,indent=2));(trial/'process.json').write_text(json.dumps(record,indent=2))
  deadline=time.monotonic()+20;preview={}
  while time.monotonic()<deadline:
   if server.poll() is not None:raise RuntimeError('server exited before admission')
   try:preview=json.loads((trial/'preview.json').read_text())
   except FileNotFoundError:pass
   if preview.get('execution_admission_ready'):break
   time.sleep(.1)
  else:raise RuntimeError('execution conditions not admitted: '+json.dumps(preview))
  with (trial/'request.log').open('w') as log:
   client=subprocess.Popen([sys.executable,'tools/request_grid_grasp.py','--execute'],cwd=root,env=env,
                           stdin=subprocess.DEVNULL,stdout=log,stderr=log)
  update(phase='waiting_for_object',request_pid=client.pid,server_pid=server.pid,ready_error_rad=ready['final_max_error_rad'])
  committed=False
  while client.poll() is None:
   try:
    preview=json.loads((trial/'preview.json').read_text())
    if preview.get('locked_cell_id') and not committed:
     committed=True;update(phase='committed',selected=preview)
   except FileNotFoundError:pass
   time.sleep(.1)
  if not (trial/'result.json').exists():raise RuntimeError('request failed before result; see '+str(trial/'request.log'))
  result=json.loads((trial/'result.json').read_text());state['results'].append(result)
  shutil.copy2(root/'.setup/grid-grasp-test/request-result.json',trial/'request-result.json')
  if client.returncode or not result.get('success'):raise RuntimeError('trial did not complete: '+result.get('reason','unknown'))
  update(completed_trials=number,phase='trial_completed',last_result=result)
 update(phase='completed',success=True,no_automatic_fourth_trial=True)
except BaseException as error:
 update(phase='stopped',success=False,error=f'{type(error).__name__}: {error}')
 raise
