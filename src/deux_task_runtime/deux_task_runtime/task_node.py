"""Bounded async memory behavior tree; motion is never grasp/read/place proof."""

from copy import deepcopy
from uuid import uuid4
import time
import numpy as np
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from action_msgs.msg import GoalStatus
from deux_task_msgs.action import PlanMotion, ExecuteMotion, RunTask
from deux_task_msgs.msg import BarcodeRead, ScanSession, TaskStatus, PerceptionRequest, BarcodeTarget
from .behavior_tree import Leaf, Sequence, Retry, Failure
from .common import (
    RuntimeNode,
    ExclusiveAction,
    LATCHED,
    seconds,
    ros_time,
    pose_matrix,
    pause,
    wait,
    cancel_children,
    run,
)


class TaskNode(RuntimeNode):
    def __init__(self, **kwargs):
        super().__init__("task_node", observe_state=True, **kwargs)
        self.planner = ActionClient(
            self, PlanMotion, "/task/plan_motion", callback_group=self.group
        )
        self.executor_client = ActionClient(
            self, ExecuteMotion, "/task/execute_motion", callback_group=self.group
        )
        self.session_pub = self.create_publisher(
            ScanSession, "/task/scan_session", LATCHED
        )
        self.status_pub = self.create_publisher(TaskStatus, "/task/status", LATCHED)
        self.barcode_sub = self.create_subscription(
            BarcodeRead, "/task/barcode_read", self.read, 10
        )
        self.perception_pub = self.create_publisher(PerceptionRequest, "/task/perception_request", LATCHED)
        self.perception_request = None
        self.label = None
        self.label_sub = self.create_subscription(BarcodeTarget, "/task/barcode_target", self.on_label,
                                                  qos_profile_sensor_data)
        self.state = "Idle"
        self.target_id = ""
        self.session_id = ""
        self.barcode = None
        self.grasp_attempt = self.scan_attempt = self.ik_calls = 0
        self.reason = ""
        self.child = None
        self.action = ExclusiveAction(
            self, RunTask, "/task/run", self.execute, self.validate
        )
        self.timer = self.create_timer(0.2, self.publish_status)

    def on_label(self, msg):
        r = self.perception_request
        if (r and r.mode == 'barcode' and msg.request_id == r.request_id
                and msg.target_id == self.target_id and msg.orientation_valid
                and msg.header.frame_id == self.config['base_frame']
                and seconds(r.header.stamp) <= seconds(msg.header.stamp) <= self.now()
                and self.now() < seconds(r.expires_at)
                and self.now()-seconds(msg.header.stamp) <= self.config['max_object_age']
                and np.isfinite([msg.plane_rmse_m, msg.valid_fraction]).all()
                and 0 <= msg.plane_rmse_m <= .003 and .7 <= msg.valid_fraction <= 1.):
            try: pose_matrix(msg.pose)
            except ValueError: return
            self.label = msg

    def request_perception(self, mode):
        self.label = None
        self.perception_request = PerceptionRequest(
            header=Header(stamp=self.get_clock().now().to_msg()),
            request_id=uuid4().hex, target_id=self.target_id, mode=mode,
            expires_at=ros_time(self.now()+60.))
        self.perception_pub.publish(self.perception_request)

    def validate(self, req):
        self.measured()
        if (
            not self.planner.server_is_ready()
            or not self.executor_client.server_is_ready()
        ):
            raise ValueError("planner/executor unavailable")
        if req.target_id:
            self.target(req.target_id)

    def read(self, msg):
        if (
            self.state != "WaitBarcode"
            or msg.session_id != self.session_id
            or msg.track_id != self.target_id
        ):
            return
        if msg.simulated and not self.config.get("synthetic", False):
            return
        if (
            self.scan_start <= seconds(msg.header.stamp) <= self.now()
            and self.now() - seconds(msg.header.stamp) < self.config["scan_timeout"]
            and msg.value
        ):
            self.barcode = msg

    def publish_status(self):
        m = TaskStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.state = self.state
        m.target_id = self.target_id
        m.session_id = self.session_id
        m.reason = self.reason
        m.grasp_attempt = self.grasp_attempt
        m.scan_attempt = self.scan_attempt
        m.ik_calls = self.ik_calls
        m.active = self.action.busy
        self.status_pub.publish(m)

    def guard(self, goal, version):
        if goal.is_cancel_requested:
            raise InterruptedError("task canceled")
        self.measured()
        if self.config_version != version:
            raise ValueError("scene changed during task")
        if self.now() < self.previous_clock:
            raise ValueError("clock reset")
        self.previous_clock = self.now()
        if self.perception_request and self.now() >= seconds(self.perception_request.expires_at)-10.:
            self.request_perception(self.perception_request.mode)

    async def call(self, client, request, goal, version, timeout):
        send = client.send_goal_async(request)
        handle = None
        try:
            handle = await wait(self, send, 3.0)
            if not handle.accepted:
                raise ValueError("action goal rejected")
            self.child = handle
            future = handle.get_result_async()
            wall = time.monotonic()
            while not future.done():
                self.guard(goal, version)
                if time.monotonic() - wall > self.wall_timeout(timeout):
                    raise TimeoutError("task child timeout")
                await pause(self)
            result = future.result()
            if (
                result.status != GoalStatus.STATUS_SUCCEEDED
                or not result.result.success
            ):
                self.get_logger().error(f"{self.state}: {result.result.reason}")
                raise ValueError(result.result.reason)
            return result.result
        finally:
            if handle is not None:
                await cancel_children(self, [handle])
            elif not send.done():
                send.add_done_callback(
                    lambda f: (
                        f.result().cancel_goal_async() if f.result().accepted else None
                    )
                )
            self.child = None

    async def stage(self, name, goal, version):
        self.guard(goal, version)
        self.state = name
        self.publish_status()
        result = await self.call(
            self.planner,
            PlanMotion.Goal(
                stage=name,
                target_id=self.target_id,
                scene_version=version,
                observation=deepcopy(self.observation),
                barcode_target=deepcopy(self.label) if name == "AlignBarcode" and self.label else BarcodeTarget(),
            ),
            goal,
            version,
            35.0,
        )
        self.ik_calls += result.plan.ik_calls
        duration = seconds(result.plan.trajectory.points[-1].time_from_start)
        executed = await self.call(
            self.executor_client,
            ExecuteMotion.Goal(plan=result.plan),
            goal,
            version,
            duration + 10.0,
        )
        if executed.reason == "dry_run_only":
            raise ValueError("dry-run plan checked; no motion or task success claimed")

    async def verify(self, predicate, after, goal, version):
        wall = time.monotonic()
        identities = set()
        started = self.now()
        while self.now()-started < self.config["verification_timeout"] and time.monotonic() - wall < self.wall_timeout(self.config["verification_timeout"]):
            self.guard(goal, version)
            try:
                obj = self.target(self.target_id)
                stamp = seconds(self.observation.objects.header.stamp)
                if stamp > after and predicate(pose_matrix(obj.pose.pose)):
                    identities.add((stamp, self.observation.objects.sequence))
                    if len(identities) >= 2:
                        return True
            except ValueError:
                pass
            await pause(self)
        return False

    def close_session(self):
        s = ScanSession()
        s.header.stamp = self.get_clock().now().to_msg()
        s.session_id = self.session_id
        s.track_id = self.target_id
        s.active = False
        self.session_pub.publish(s)

    async def execute(self, goal):
        version = self.config_version
        self.previous_clock = self.now()
        self.grasp_attempt = self.scan_attempt = self.ik_calls = 0
        self.reason = ""
        self.barcode = None
        self.target_id = goal.request.target_id
        if not self.target_id:
            if (
                not self.observation.objects_valid
                or not self.observation.objects.detections
            ):
                raise ValueError("no target")
            self.target_id = self.observation.objects.detections[0].track_id
        self.target(self.target_id)
        self.request_perception("locate")
        async def grasp_and_verify():
            for attempt in range(1, 3):
                self.grasp_attempt = attempt
                original = pose_matrix(self.target(self.target_id).pose.pose)
                try:
                    for name in ("Approach", "Descend", "Close"):
                        await self.stage(name, goal, version)
                    self.request_perception("track")
                    after = self.now()
                    await self.stage("Lift", goal, version)
                    self.state = "VerifyLift"
                    if await self.verify(
                        lambda p: p[2, 3] - original[2, 3]
                        >= self.config["lift_height"] * 0.5,
                        after,
                        goal,
                        version,
                    ):
                        break
                    raise ValueError("lift not independently observed")
                except (ValueError, TimeoutError) as error:
                    if "dry-run" in str(error):
                        raise
                    self.guard(goal, version)
                    # Retry only when fresh observation shows the object still on its original support.
                    current = pose_matrix(self.target(self.target_id).pose.pose)
                    if attempt == 2 or abs(current[2, 3] - original[2, 3]) > 0.02:
                        raise
                    await self.stage("Recover", goal, version)
                    await self.stage("Release", goal, version)
        async def search_label():
            self.scan_attempt += 1
            self.state = 'ScanSearch'
            self.request_perception('barcode')
            wall = time.monotonic()
            started = self.now()
            while self.now()-started < self.config['scan_timeout'] and time.monotonic()-wall < self.wall_timeout(self.config['scan_timeout']):
                self.guard(goal, version)
                if self.label is not None:
                    return
                await pause(self)
            raise Failure('barcode surface missing')

        async def align_label():
            # Capture fresh label at planning admission; right hand remains still.
            await self.stage('AlignBarcode', goal, version)

        async def read_label():
            self.state = "WaitBarcode"
            self.session_id = uuid4().hex
            self.scan_start = self.now()
            self.barcode = None
            s = ScanSession()
            s.header.stamp = ros_time(self.scan_start)
            s.session_id = self.session_id
            s.track_id = self.target_id
            s.expires_at = ros_time(self.scan_start + self.config["scan_timeout"])
            s.active = True
            self.session_pub.publish(s)
            try:
                wall = time.monotonic()
                while self.barcode is None and self.now() < seconds(s.expires_at) and time.monotonic()-wall < self.wall_timeout(self.config['scan_timeout']):
                    self.guard(goal, version)
                    await pause(self)
                if self.barcode is None: raise Failure('barcode missing after scan attempt')
            finally:
                self.close_session()

        async def place_and_verify():
            self.request_perception('track')
            await self.stage("Place", goal, version)
            after = self.now()
            await self.stage("Release", goal, version)
            await self.stage("Retreat", goal, version)
            self.state = "VerifyPlace"
            bounds = self.config["destination_bounds"]

            def placed(p):
                obs = self.observation
                if (
                    not obs.right_ee_valid
                    or obs.base_from_right_ee.header.frame_id
                    != self.config["base_frame"]
                ):
                    return False
                age = self.now() - seconds(obs.base_from_right_ee.header.stamp)
                if not 0 <= age <= self.config["max_observation_age"]:
                    return False
                ee = obs.base_from_right_ee.transform.translation
                separation = np.linalg.norm(p[:3, 3] - [ee.x, ee.y, ee.z])
                return bool(
                    np.all(p[:3, 3] >= bounds["min"])
                    and np.all(p[:3, 3] <= bounds["max"])
                    and separation > self.config["attached_radius"]
                )

            if not await self.verify(placed, after, goal, version):
                raise ValueError("placement/separation not independently observed")

        self.scan_tree = Retry('ScanRetry', Sequence('ScanSequence', [
            Leaf('ScanSearch', search_label), Leaf('AlignBarcode', align_label),
            Leaf('WaitBarcode', read_label)]), 3)
        self.tree = Sequence('Transfer', [Leaf('GraspAndVerify', grasp_and_verify),
            self.scan_tree, Leaf('PlaceAndVerify', place_and_verify)])
        try:
            await self.tree.run()
            self.state = "Succeeded"
            self.reason = "grasp, barcode and placement observed"
            return RunTask.Result(
                success=True, reason=self.reason, barcode=self.barcode.value
            )
        except Exception as error:
            self.state = "Canceled" if goal.is_cancel_requested else "Failed"
            self.reason = str(error)
            raise
        finally:
            self.close_session()
            self.request_perception("locate")
            self.publish_status()


def main(args=None):
    run(TaskNode, args)
