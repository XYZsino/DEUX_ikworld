"""Explicitly enable repetition and submit ONE long-lived RunTask goal.

No goal retries and no implicit cancellation on client shutdown. Stop via the
repeat service (finish current cycle) or the RunTask action cancel service.
"""
import json,time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from std_srvs.srv import SetBool
from std_msgs.msg import String
from deux_task_msgs.action import RunTask,LookupGrasp,MoveToReady,PlanMotion,ExecuteMotion
from deux_task_runtime.common import LATCHED


def main():
    rclpy.init();node=rclpy.create_node('lookup_repeat_explicit_request')
    root=Path(__file__).resolve().parents[1];out=root/'.setup/lookup-bt/repeat';out.mkdir(parents=True,exist_ok=True)
    record=dict(started_wall_s=time.time(),accepted=False,automatic_goal_resubmission=False)
    def save():
        temporary=out/'request.tmp';temporary.write_text(json.dumps(record,indent=2));temporary.replace(out/'request.json')
    def status(msg):
        data=json.loads(msg.data)
        previous=record.get('repeat_status',{})
        record['repeat_status']=data;save()
        if any(data.get(k)!=previous.get(k) for k in ('state','cycle','completed')):
            print(json.dumps(data),flush=True)
    node.create_subscription(String,'/task/repeat_status',status,LATCHED)
    action=ActionClient(node,RunTask,'/task/run')
    control=node.create_client(SetBool,'/task/repeat_execution')
    dependencies=[ActionClient(node,kind,name) for kind,name in [
        (LookupGrasp,'/task/lookup_grasp'),(MoveToReady,'/task/lookup_prepare'),
        (PlanMotion,'/task/plan_motion'),(ExecuteMotion,'/task/execute_motion')]]
    try:
        if not control.wait_for_service(timeout_sec=30.) or not action.wait_for_server(timeout_sec=30.):raise RuntimeError('repeat BT unavailable')
        for dependency in dependencies:
            if not dependency.wait_for_server(timeout_sec=30.):raise RuntimeError('BT dependency not ready; no goal submitted')
        future=control.call_async(SetBool.Request(data=True));rclpy.spin_until_future_complete(node,future,timeout_sec=5.)
        if not future.done() or not future.result().success:raise RuntimeError('repeat enable rejected or timed out')
        pending=action.send_goal_async(RunTask.Goal(target_id=''));rclpy.spin_until_future_complete(node,pending,timeout_sec=5.)
        if not pending.done():raise RuntimeError('goal response unknown; do not resubmit')
        goal=pending.result();record['accepted']=goal.accepted;save()
        if not goal.accepted:raise RuntimeError('RunTask rejected; do not resubmit')
        record['goal_uuid']=[int(v) for v in goal.goal_id.uuid];save();print('repeat RunTask accepted',flush=True)
        result=goal.get_result_async();rclpy.spin_until_future_complete(node,result)
        response=result.result();record.update(status=response.status,success=response.result.success,reason=response.result.reason,completed_wall_s=time.time());save();print(json.dumps(record,indent=2),flush=True)
    except Exception as error:
        record['error']=str(error);save();raise
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
