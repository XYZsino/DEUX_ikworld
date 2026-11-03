"""Cancel lookup tasks, return arms to attention, then stop the BT launcher.

Default shutdown commands current -> ready -> reversed simultaneous ready route.
No finger commands or automatic task restart. --no-stow skips arm return.
If cancellation or arm return fails, keep servers running and report failure.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import time

ROOT = Path(__file__).resolve().parents[1]
ACTIONS = ('/task/run', '/task/move_to_ready', '/task/lookup_grasp',
           '/task/lookup_prepare', '/task/execute_motion',
           '/task/return_to_attention', '/task/lookup_attention','/task/lookup_empty_return')


def is_lookup_launcher(argv, cwd, root):
    if 'launch' not in argv or not any(Path(arg).name == 'ros2' for arg in argv):
        return False
    index = argv.index('launch')
    if len(argv) <= index+1:
        return False
    launch = Path(argv[index+1])
    if not launch.is_absolute():
        launch = Path(cwd)/launch
    return launch.resolve() == root/'lookup_bt.thor.launch.py'


def process(pid):
    try:
        directory = Path('/proc')/str(pid)
        fields = (directory/'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        return dict(pid=int(pid), parent=int(fields[1]), started=fields[19],
                    argv=(directory/'cmdline').read_bytes().decode().rstrip('\0').split('\0'),
                    cwd=os.readlink(directory/'cwd'))
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        return None


def same_process(record):
    current = process(record['pid'])
    return current is not None and current['started'] == record['started']


def find_launchers(root, domain):
    found=[]
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():continue
        entry=process(int(path.name))
        if not entry or not is_lookup_launcher(entry['argv'],entry['cwd'],root):continue
        try:
            env=dict(item.split(b'=',1) for item in (path/'environ').read_bytes().split(b'\0') if b'=' in item)
        except (FileNotFoundError, PermissionError):continue
        if env.get(b'ROS_DOMAIN_ID',b'0').decode()==domain:found.append(entry)
    return found


def descendants(parents):
    entries=[process(int(p.name)) for p in Path('/proc').iterdir() if p.name.isdigit()]
    ids={p['pid'] for p in parents};found=[]
    while True:
        children=[p for p in entries if p and p['pid'] not in ids and p['parent'] in ids]
        if not children:return found
        found.extend(children);ids.update(p['pid'] for p in children)


def cancel_actions(timeout):
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from action_msgs.srv import CancelGoal
    from std_srvs.srv import SetBool
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    rclpy.init();node=rclpy.create_node('lookup_bt_shutdown')
    statuses={};subscriptions=[]
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    def receive(name,msg):
        statuses[name]={bytes(s.goal_info.goal_id.uuid):s.status for s in msg.status_list}
    for name in ACTIONS:
        subscriptions.append(node.create_subscription(GoalStatusArray,name+'/_action/status',
            lambda msg,name=name:receive(name,msg),qos))
    def call(kind,name,request):
        client=node.create_client(kind,name)
        if not client.wait_for_service(timeout_sec=5.):raise RuntimeError(name+' unavailable; no process signals sent')
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
        if not future.done():raise TimeoutError(name+' response timeout; no process signals sent')
        return future.result()
    try:
        result=call(SetBool,'/task/repeat_execution',SetBool.Request(data=False))
        if not result.success:raise RuntimeError(result.message)
        for name in ACTIONS:
            response=call(CancelGoal,name+'/_action/cancel_goal',CancelGoal.Request())
            if response.return_code!=0:raise RuntimeError(name+' cancel rejected; no process signals sent')
            goals={bytes(g.goal_id.uuid) for g in response.goals_canceling}
            deadline=time.monotonic()+timeout
            while any(statuses.get(name,{}).get(g) not in (4,5,6) for g in goals):
                if time.monotonic()>=deadline:raise TimeoutError(name+' terminal status unconfirmed; servers preserved')
                rclpy.spin_once(node,timeout_sec=.1)
            print(name+': inactive / cancellation confirmed',flush=True)
        # Drain final status changes; refuse shutdown if another client started work.
        until=time.monotonic()+.5
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
        if any(s in (1,2,3) for values in statuses.values() for s in values.values()):
            raise RuntimeError('BT action still active; servers preserved')
    finally:
        node.destroy_node();rclpy.shutdown()


def return_to_attention(timeout):
    """Send one return goal; only successful completion permits server shutdown."""
    import rclpy
    from rclpy.action import ActionClient
    from action_msgs.srv import CancelGoal
    from deux_task_msgs.action import MoveToReady
    from unique_identifier_msgs.msg import UUID
    from uuid import uuid4
    rclpy.init();node=rclpy.create_node('lookup_bt_shutdown_return')
    client=ActionClient(node,MoveToReady,'/task/return_to_attention')
    identity=UUID(uuid=list(uuid4().bytes));submitted=False;completed=False
    def wait(future,seconds,label):
        rclpy.spin_until_future_complete(node,future,timeout_sec=seconds)
        if not future.done():raise TimeoutError(label)
        return future.result()
    try:
        if not client.wait_for_server(timeout_sec=5.):raise RuntimeError('attention server unavailable; BT left running')
        submitted=True
        handle=wait(client.send_goal_async(MoveToReady.Goal(),goal_uuid=identity),5.,'attention goal response unknown')
        if not handle.accepted:
            submitted=False;raise RuntimeError('attention return rejected; BT left running')
        response=wait(handle.get_result_async(),timeout,'attention return timed out')
        completed=True
        if response.status!=4 or not response.result.success:raise RuntimeError('attention return failed: '+response.result.reason)
        print('Both arms returned to attention. Stopping BT servers.',flush=True)
    finally:
        if submitted and not completed:
            # UUID-specific cancel also handles a delayed goal acceptance.
            cancel=node.create_client(CancelGoal,'/task/return_to_attention/_action/cancel_goal')
            if cancel.wait_for_service(timeout_sec=3.):
                request=CancelGoal.Request();request.goal_info.goal_id=identity
                future=cancel.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=5.)
                print('Attention return cancellation requested; BT servers preserved.',flush=True)
        node.destroy_node();rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run',action='store_true',help='list target launchers without ROS calls or signals')
    parser.add_argument('--timeout',type=float,default=30.,help='seconds to confirm each canceled action and shutdown')
    parser.add_argument('--return-timeout',type=float,default=120.,help='seconds allowed for return-to-attention action')
    parser.add_argument('--no-stow',action='store_true',help='cancel and stop BT without returning the arms to attention')
    args=parser.parse_args()
    if not 0<args.timeout<=300:parser.error('--timeout must be in (0, 300]')
    if not 0<args.return_timeout<=600:parser.error('--return-timeout must be in (0, 600]')
    domain=os.environ.get('ROS_DOMAIN_ID','12');launchers=find_launchers(ROOT,domain)
    if args.dry_run:
        print(json.dumps(dict(domain=domain,launchers=launchers,actions=ACTIONS,return_to_attention=not args.no_stow),indent=2));return
    if not launchers:
        print('Lookup BT launcher is already stopped. No processes changed.');return
    if len(launchers)!=1:raise RuntimeError('Multiple lookup launchers found; refusing ambiguous shutdown')
    cancel_actions(args.timeout)
    if not args.no_stow:return_to_attention(args.return_timeout)
    children=descendants(launchers)
    for entry in launchers:
        if same_process(entry):os.kill(entry['pid'],signal.SIGINT)
    deadline=time.monotonic()+args.timeout
    while any(same_process(p) for p in launchers+children):
        if time.monotonic()>=deadline:raise TimeoutError('BT shutdown incomplete; no additional force-kill sent')
        time.sleep(.1)
    directory=ROOT/'.setup/lookup-bt';directory.mkdir(parents=True,exist_ok=True)
    (directory/'shutdown.json').write_text(json.dumps(dict(stopped=True,domain=domain,
        launcher_pids=[p['pid'] for p in launchers],cancel_confirmed=True,
        returned_to_attention=not args.no_stow),indent=2))
    print('Lookup BT stopped. Camera, perception, GUI and hardware drivers preserved.')


if __name__=='__main__':main()
