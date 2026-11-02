"""Start ZED/effort/perception/lookup BT, then request repetition once.

Existing hardware bringup is required. Default starts real repeating motion.
No builds, trajectory audits, process termination, or automatic goal retries.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]


def commands(root):
    return dict(gravity=['ros2','launch','deux_gravity','deux_gravity.launch.py'],
        zed=['bash','run_calibrated_zed.bash'],
        perception=['ros2','run','deux_task_perception','tracked_perception_node','--ros-args',
            '-r','__node:=perception_node','--params-file',str(root/'tracked_perception.thor.yaml'),
            '-p','config_file:='+str(root/'src/deux_task_bringup/config/installation.yaml')],
        bt=['bash','run_lookup_bt.bash','enable_execution:=true','repeat_execution:=true'],
        repeat=[sys.executable,'tools/request_lookup_repeat.py'],
        gui=['ros2','launch','deux_task_gui','control.launch.py','deployment:=real'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--servers-only',action='store_true',help='start servers without submitting motion')
    parser.add_argument('--gui',action='store_true',help='include the loopback web GUI on port 8080')
    parser.add_argument('--dry-run',action='store_true',help='print commands without ROS access or execution')
    args=parser.parse_args();cmd=commands(ROOT)
    if args.dry_run:
        print(json.dumps(dict(domain=os.environ.get('ROS_DOMAIN_ID','12'),commands=cmd,
            submit_repeat=not args.servers_only,start_gui=args.gui,hardware_bringup_required=True),indent=2));return
    for relative in ['.setup/lookup-bt/home-left-conveyor.json','tracked_perception.thor.yaml',
                     'artifacts/dataset-barcode-pose/reading-pose.json',
                     'artifacts/dataset-barcode-pose/replay-height-baseline.json',
                     'artifacts/right-high-scan-waypoint/library-validation.json']:
        if not (ROOT/relative).is_file():raise RuntimeError(f'missing setup file: {relative}; see LOOKUP_BT_SETUP.rst')
    if (ROOT/'.setup/camera-calibration-lift-invalid.json').exists():
        raise RuntimeError('lift/calibration snapshot invalidated; update calibration first')

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String,Float64MultiArray
    from std_srvs.srv import Trigger
    from deux_task_msgs.action import RunTask,LookupGrasp,MoveToReady,PlanMotion,ExecuteMotion
    from deux_task_msgs.msg import TaskStatus
    from deux_task_runtime.common import LATCHED

    out=ROOT/'.setup/lookup-bt/live-start';out.mkdir(parents=True,exist_ok=True)
    registry=out/'processes.json'
    records=json.loads(registry.read_text()) if registry.exists() else {}
    rclpy.init();node=rclpy.create_node('lookup_stack_startup');seen={};children=[];subscriptions=[]

    def names():return {name for name,_ in node.get_node_names_and_namespaces()}
    def spin(duration):
        until=time.monotonic()+duration
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
    def wait(predicate,label,timeout=45.):
        end=time.monotonic()+timeout;report_at=0.
        while not predicate():
            for name,process in children:
                if process.poll() is not None:raise RuntimeError(f'{name} exited; see {out/(name+".log")}')
            if time.monotonic()>=end:raise RuntimeError(f'{label} timed out; inspect {out}')
            if time.monotonic()>=report_at:print('Waiting:',label,flush=True);report_at=time.monotonic()+10.
            rclpy.spin_once(node,timeout_sec=.1)
    def start(name):
        log=out/(name+'.log')
        with log.open('a') as stream:
            process=subprocess.Popen(cmd[name],cwd=ROOT,stdin=subprocess.DEVNULL,
                stdout=stream,stderr=subprocess.STDOUT,start_new_session=True,close_fds=True)
        children.append((name,process));records[name]=dict(pid=process.pid,command=cmd[name],log=str(log))
        registry.write_text(json.dumps(records,indent=2));print(f'{name}: PID {process.pid}; {log}',flush=True)
        return process
    def received(key,msg):seen[key]=(time.monotonic(),msg)
    def fresh(key):return key in seen and time.monotonic()-seen[key][0]<1.
    def repeat_status(msg):
        try:received('repeat',json.loads(msg.data))
        except ValueError:pass

    def ensure_gui():
        if not args.gui:return
        from urllib.request import urlopen
        from urllib.error import HTTPError
        def http_ready():
            try:
                with urlopen('http://127.0.0.1:8080/',timeout=.5) as response:return response.status==200
            except HTTPError as error:return error.code in (401,403)
            except OSError:return False
        if 'task_gui' not in names():start('gui')
        wait(lambda:'task_gui' in names() and http_ready(),'GUI HTTP server',30.)
        print('GUI ready: http://127.0.0.1:8080/ (Thor loopback)',flush=True)
        print('Laptop tunnel: ssh -N -L 18080:127.0.0.1:8080 '+
              os.environ.get('DEUX_THOR_SSH','xyz_thor@192.168.1.10'),flush=True)
        print('Laptop URL: http://127.0.0.1:18080/ ; authentication token: existing GUI log or '+str(out/'gui.log'),flush=True)

    try:
        subscriptions.append(node.create_subscription(String,'/task/repeat_status',repeat_status,LATCHED))
        subscriptions.append(node.create_subscription(TaskStatus,'/task/status',lambda m:received('task',m),LATCHED))
        spin(3.)
        status=seen.get('repeat',(0,{}))[1]
        if fresh('repeat') and status.get('active'):
            ensure_gui()
            print(f'Existing lookup BT active: {status.get("state")}; task preserved, no duplicate request.',flush=True);return
        if fresh('task') and seen['task'][1].active:raise RuntimeError('another task is active; no startup changes made')
        if 'task_node' in names() and 'grid_grasp_trial' not in names():
            raise RuntimeError('existing non-lookup BT detected; stop it while idle first')
        # Check the controller manager before launching or activating anything.
        subprocess.run(['bash','enable_arm_effort.bash','status'],cwd=ROOT,check=True,timeout=20.)
        subscriptions.append(node.create_subscription(Float64MultiArray,'/arm/deux_arm_effort_controller/commands',
            lambda m:received('effort',m),qos_profile_sensor_data))
        if 'deux_gravity' not in names():
            from ament_index_python.packages import get_package_prefix
            get_package_prefix('deux_gravity')
            start('gravity')
        wait(lambda:fresh('effort'),'fresh gravity torque stream',20.)
        subprocess.run(['bash','enable_arm_effort.bash'],cwd=ROOT,check=True,timeout=20.)
        subprocess.run([sys.executable,'tools/ensure_bt_state.py'],cwd=ROOT,check=True,timeout=30.)
        subprocess.run([sys.executable,'tools/configure_hand_modes.py'],cwd=ROOT,check=True,timeout=20.)
        subscriptions.append(node.create_subscription(Image,'/zed/color/image_raw',lambda m:received('rgb',m),qos_profile_sensor_data))
        if 'zed2i' not in names():start('zed')
        wait(lambda:fresh('rgb'),'ZED RGB stream')
        if 'perception_node' not in names():start('perception')
        wait(lambda:'perception_node' in names(),'perception node')
        clients=[ActionClient(node,kind,name) for kind,name in [
            (RunTask,'/task/run'),(LookupGrasp,'/task/lookup_grasp'),(MoveToReady,'/task/lookup_prepare'),
            (PlanMotion,'/task/plan_motion'),(ExecuteMotion,'/task/execute_motion')]]
        if 'grid_grasp_trial' not in names():start('bt')
        wait(lambda:all(c.server_is_ready() for c in clients),'lookup BT action servers',90.)
        ensure_gui()
        health=node.create_client(Trigger,'/task/execution_health')
        wait(health.service_is_ready,'execution health service',10.)
        future=health.call_async(Trigger.Request());wait(future.done,'execution feedback health',10.)
        if not future.result().success:raise RuntimeError(future.result().message)
        if args.servers_only:
            print('Lookup servers ready; no RunTask submitted. Services remain running.',flush=True);return
        # Re-read live status just before submitting; never duplicate an active goal.
        spin(1.)
        if fresh('repeat') and seen['repeat'][1].get('active'):
            print('Lookup already active; no duplicate request.',flush=True);return
        request=start('repeat')
        (out/'repeat-process.json').write_text(json.dumps(records['repeat'],indent=2))
        # Request helper sends exactly one goal and never resubmits it.
        wait(lambda:fresh('repeat') and seen['repeat'][1].get('active'),
             'repeat RunTask accepted (no automatic resubmission)',20.)
        print('Repeating lookup BT started. Log:',records['repeat']['log'],flush=True)
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
