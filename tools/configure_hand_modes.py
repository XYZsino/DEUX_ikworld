"""Set hand UI modes before BT startup; never sends grip or arm commands."""
import argparse
from pathlib import Path
import time
import yaml
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from deux_hand_msgs.msg import HandUiMessage
from deux_task_msgs.msg import TaskStatus


def confirmed(msg, mode):
    return msg.leader_mode == mode and msg.teleop_mode == 0 and msg.run_flag and not msg.stop_flag


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=8.)
    args = parser.parse_args()
    if not 0 < args.timeout <= 60:parser.error('timeout must be between 0 and 60 seconds')
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root/'src/deux_task_bringup/config/installation.yaml').read_text())
    mapping = yaml.safe_load((root/'src/deux_task_bringup/config/hand_mapping.yaml').read_text())
    modes = {s:int(mapping[s]['leader_mode']) for s in ('left','right')}
    required = {'left'} if config.get('right_hand_disabled',False) else {'left','right'}
    rclpy.init();node = rclpy.create_node('ikworld_hand_mode_startup')
    state = {};last_sent = {};pubs = {};subs = [];stop = [False];task_active = [False]
    def receive(side,msg):state[side]=(time.monotonic(),msg)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    try:
        for side in modes:
            pubs[side]=node.create_publisher(HandUiMessage,f'/hand/{side}/ui_request',10)
            subs.append(node.create_subscription(HandUiMessage,f'/hand/{side}/ui_response',lambda m,s=side:receive(s,m),10))
        subs.append(node.create_subscription(Bool,'/deux/estop',lambda m:stop.__setitem__(0,m.data),10))
        subs.append(node.create_subscription(TaskStatus,'/task/status',lambda m:task_active.__setitem__(0,m.active),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        spin(2.)
        end=time.monotonic()+args.timeout
        while time.monotonic()<end:
            if stop[0]:raise RuntimeError('hand e-stop active; mode setup stopped')
            if task_active[0]:raise RuntimeError('BT already active; mode setup stopped')
            now=time.monotonic()
            good={s for s,(t,m) in state.items() if now-t<1. and confirmed(m,modes[s])}
            if good==set(modes):break
            for side in modes.keys()-good:
                # Require a real UI response before enabling RUN. Adapter subscriptions
                # alone are not evidence that a hand driver is connected.
                if side not in state or now-state[side][0]>1.:continue
                if now-last_sent.get(side,-1e9)<1.:continue
                pubs[side].publish(HandUiMessage(run_flag=True,stop_flag=False,teleop_mode=0,leader_mode=modes[side]))
                last_sent[side]=now
            spin(.1)
        now=time.monotonic()
        good={s for s,(t,m) in state.items() if now-t<1. and confirmed(m,modes[s])}
        for side in modes:
            if side in good:print(f'{side}: mode {modes[side]} / RUN confirmed',flush=True)
            elif side in required:print(f'{side}: required mode/RUN not confirmed',flush=True)
            else:print(f'WARNING: {side} mode/RUN not confirmed; right_hand_disabled=true, BT uses virtual right hand',flush=True)
        if required-good:raise RuntimeError('required hand mode setup failed; BT not started')
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
