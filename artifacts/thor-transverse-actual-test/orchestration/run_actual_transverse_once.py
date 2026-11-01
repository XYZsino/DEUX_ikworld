import datetime,json,os,signal,subprocess,sys,time,shutil
from pathlib import Path
root=Path('/home/xyz_thor/DEUX_ikworld');base=root/'.setup/grid-grasp-classified-test'
ready=json.loads((root/'.setup/grid-grasp-test/ready-result.json').read_text())
if not ready.get('success') or ready['final_max_error_rad']>.03:raise RuntimeError('right ready not settled')
record=json.loads((base/'process.json').read_text());previous=Path(record['output_directory'])
preview=json.loads((previous/'preview.json').read_text())
if preview.get('motion_requested') or not preview.get('preview_only'):raise RuntimeError('expected idle preview')
if not preview.get('arm_ready'):raise RuntimeError('combined library ready snapshot is not matched')
pid=record['pid'];command=Path(f'/proc/{pid}/cmdline').read_bytes().decode().replace('\0',' ')
if 'tools/classified_grasp_trial.py' not in command:raise RuntimeError('preview PID identity changed')
os.kill(pid,signal.SIGINT)
until=time.monotonic()+5
while time.monotonic()<until:
 try:
  if Path(f'/proc/{pid}/status').read_text().split('State:')[1].splitlines()[0].strip().startswith('Z'):break
 except FileNotFoundError:break
 time.sleep(.1)
else:raise RuntimeError('old preview did not stop')
out=base/('actual-transverse-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'));out.mkdir()
env=os.environ.copy();env['ROS_DOMAIN_ID']='12'
state=dict(output_directory=str(out),phase='configuring_hand',actual_hardware_test=True,transverse_only=True)
def update(**values):
 state.update(values);temporary=out/'status.tmp';temporary.write_text(json.dumps(state,indent=2));temporary.replace(out/'status.json')
 (base/'actual-latest.json').write_text(json.dumps(dict(directory=str(out),status=str(out/'status.json')),indent=2))
 print(json.dumps(state),flush=True)
try:
 update()
 shutil.copy2(root/'.setup/grid-grasp-test/ready-result.json',out/'ready-result.json')
 shutil.copy2(base/'actual-ready-execution.log',out/'ready-execution.log')
 shutil.copy2('/tmp/transverse_only_actual.py',out/'transverse_only_actual.py')
 with (out/'hand-mode.log').open('w') as log:
  subprocess.run([sys.executable,'tools/configure_right_trial_hand.py'],cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
 args=[sys.executable,'/tmp/transverse_only_actual.py','--ros-args',
  '-p','grid_library_directory:='+str(root/'artifacts/right_grasp_grid_classified'),
  '-p','use_observed_yaw:=true','-p','enable_execution:=true','-p','target_wait_timeout_s:=300.',
  '-p','grid_output_directory:='+str(out),'-p','stationary_left_file:='+str(base/'stationary-left.json')]
 with (out/'server.log').open('w') as log:
  server=subprocess.Popen(args,cwd=root,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
 record=dict(pid=server.pid,command=args,output_directory=str(out),identity='/tmp/transverse_only_actual.py',
  execution_enabled=True,transverse_only=True,started_at=time.time())
 (base/'process.json').write_text(json.dumps(record,indent=2));(out/'process.json').write_text(json.dumps(record,indent=2))
 update(phase='waiting_for_hand_profile',server_pid=server.pid)
 deadline=time.monotonic()+90;preview={}
 while time.monotonic()<deadline:
  if server.poll() is not None:raise RuntimeError('execution server exited')
  try:preview=json.loads((out/'preview.json').read_text())
  except FileNotFoundError:pass
  if preview.get('execution_admission_ready'):break
  time.sleep(.1)
 else:raise RuntimeError('execution admission not ready: '+json.dumps(preview))
 with (out/'request.log').open('w') as log:
  client=subprocess.Popen([sys.executable,'tools/request_grid_grasp.py','--execute'],cwd=root,env=env,
   stdin=subprocess.DEVNULL,stdout=log,stderr=log)
 update(phase='waiting_for_transverse_bag',request_pid=client.pid)
 committed=False
 while client.poll() is None:
  try:
   preview=json.loads((out/'preview.json').read_text())
   if preview.get('locked_cell_id') and not committed:
    committed=True;update(phase='committed',selected=preview)
  except FileNotFoundError:pass
  time.sleep(.1)
 if not (out/'result.json').exists():raise RuntimeError('request failed before result')
 result=json.loads((out/'result.json').read_text())
 shutil.copy2(root/'.setup/grid-grasp-test/request-result.json',out/'request-result.json')
 if not result.get('success') or client.returncode:raise RuntimeError('test did not complete: '+result.get('reason','unknown'))
 if result.get('grasp_mode')!='transverse_axis_locked':raise RuntimeError('unexpected non-transverse result')
 update(phase='completed',success=True,result=result,physical_grasp_verified=False,no_automatic_repeat=True)
except BaseException as error:
 update(phase='stopped',success=False,error=f'{type(error).__name__}: {error}')
 raise
