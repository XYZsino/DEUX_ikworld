"""Explicit one-shot REAL grid action request. Requires --execute; never retries."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from deux_task_msgs.action import MoveToReady


def main():
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not args.execute: parser.error('--execute is required to request real motion')
    rclpy.init(); node = Node('requested_grid_grasp_once'); client = ActionClient(node, MoveToReady, '/task/grid_grasp')
    handle = None; terminal = False
    def wait(future, timeout):
        deadline = float('inf') if timeout is None else time.monotonic()+timeout
        while not future.done():
            rclpy.spin_once(node, timeout_sec=.05)
            if time.monotonic() > deadline: raise TimeoutError('grid action response timeout')
        return future.result()
    try:
        if not client.wait_for_server(timeout_sec=5.): raise RuntimeError('grid action server unavailable')
        pending = client.send_goal_async(MoveToReady.Goal()); handle = wait(pending, 5.)
        if not handle.accepted: raise RuntimeError('grid execution admission rejected; inspect preview execution_reason')
        print('Real one-shot grid request accepted; waiting for eligible moving object.', flush=True)
        response = wait(handle.get_result_async(), None)
        terminal = True
        report = dict(actual_hardware_request=True, action_status=response.status, success=response.result.success,
                      reason=response.result.reason, physical_grasp_verified=False)
        out = Path(__file__).resolve().parents[1]/'.setup/grid-grasp-test/request-result.json'
        out.write_text(json.dumps(report, indent=2)); print(json.dumps(report, indent=2), flush=True)
        if not response.result.success: raise RuntimeError(response.result.reason)
    finally:
        if handle is not None and handle.accepted and not terminal:
            try: wait(handle.cancel_goal_async(), 3.)
            except Exception: pass
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
