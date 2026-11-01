"""Start perception-only test. Removes BT action servers; never sends arm/hand goals."""
from pathlib import Path
import os,signal,time,subprocess
root=Path(__file__).resolve().parents[1];os.chdir(root);out=root/'.setup/icp-live-test';out.mkdir(parents=True,exist_ok=True)
rows=[s.split(None,2) for s in subprocess.check_output(['ps','-eo','pid,ppid,args'],text=True).splitlines()[1:]]
env=os.environ.copy();env['ROS_DOMAIN_ID']='12'
parents=[]
for pid,ppid,args in rows:
 if args.startswith('/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch '+str(root/'real_bt.thor.launch.py')) or args.startswith('/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch deux_task_gui control.launch.py'):
  parents.append(int(pid))
  if 'deux_task_gui' in args:
   try:
    for item in Path('/proc/'+pid+'/environ').read_bytes().split(b'\0'):
     if item.startswith(b'DEUX_GUI_TOKEN='):env['DEUX_GUI_TOKEN']=item.split(b'=',1)[1].decode()
   except OSError:pass
for pid in parents:
 try:os.kill(pid,signal.SIGTERM)
 except ProcessLookupError:pass
 for child,ppid,args in rows:
  if int(ppid)==pid:
   try:os.kill(int(child),signal.SIGTERM)
   except ProcessLookupError:pass
pidfile=out/'pids'
if pidfile.exists():
 for pid in pidfile.read_text().split():
  try:
   cmd=Path('/proc/'+pid+'/cmdline').read_bytes()
   if b'tracking_icp_monitor' in cmd or b'tracked_perception_node' in cmd or b'grasp_preview_node' in cmd:os.killpg(int(pid),signal.SIGTERM)
  except (OSError,ValueError):pass
time.sleep(3)
commands=[('perception',['ros2','run','deux_task_perception','tracked_perception_node','--ros-args','--params-file',str(root/'tracked_perception.thor.yaml'),'-p','config_file:='+str(root/'src/deux_task_bringup/config/installation.yaml')]),('monitor',['python',str(root/'tools/tracking_icp_monitor.py')]),('grasp-preview',['python',str(root/'tools/grasp_preview_node.py')]),('gui',['ros2','launch','deux_task_gui','control.launch.py','deployment:=real'])]
pids=[]
for name,cmd in commands:
 log=(out/(name+'.log')).open('a');p=subprocess.Popen(cmd,cwd=root,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True);pids.append(p.pid);print(name,p.pid)
pidfile.write_text('\n'.join(map(str,pids)))
