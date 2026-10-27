"""Concurrency and lease isolation in the combined perception node."""
import threading
import time
from concurrent.futures import Future
import numpy as np
import pytest
rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from deux_task_msgs.msg import ObjectDetectionArray, PerceptionRequest
from deux_task_perception.perception_core import LatestWorker
from deux_task_perception.tracked_perception_node import TrackedPerceptionNode


def test_worker_starts_and_notifies_without_polling_and_keeps_only_latest():
    started, release, completed = (threading.Event() for _ in range(3))
    calls = []
    def work(n):
        calls.append(n)
        if n == 1:
            started.set()
            assert release.wait(3)
        return n
    worker = LatestWorker(work, completed.set)
    try:
        worker.submit(1)
        assert started.wait(1)  # No poll or timer is needed to start.
        for n in range(2, 101): worker.submit(n)
        assert worker.pending == 100 and worker.dropped == 98
        release.set()
        assert completed.wait(1)
        completed.clear()
        assert worker.poll().result() == 1
        assert completed.wait(1)
        assert worker.poll().result() == 100
        assert calls == [1, 100]
    finally:
        release.set()
        worker.close()


class EmptyDetector:
    identity = 'test'
    def track(self, rgb, stamp): return []


def test_old_icp_completion_is_suppressed_after_mode_change_expiry_and_reset():
    rclpy.init(domain_id=218)
    node = TrackedPerceptionNode(detector=EmptyDetector(),
        model_data=(np.zeros((40,3)),np.tile([0.,0.,-1.],(40,1)),'test'),
        parameter_overrides=[Parameter('reference_frame',value='base_link')])
    ex = SingleThreadedExecutor(); ex.add_node(node)
    now = [10_000_000_000]
    node.last_clock = now[0]
    # Exercise request admission without relying on wall-clock timing.
    original_clock = node.get_clock().now
    from rclpy.time import Time
    node.get_clock().now = lambda: Time(nanoseconds=now[0])
    published = []
    class Sink:
        def publish(self, msg): published.append(msg)
    node.icp_publisher = Sink()
    def request(mode, ident):
        req = PerceptionRequest(request_id=ident, target_id='bag', mode=mode)
        req.header.stamp = Time(nanoseconds=now[0]).to_msg()
        req.expires_at = Time(nanoseconds=now[0]+1_000_000_000).to_msg()
        node.set_request(req)
    def done(token, ident):
        result = ObjectDetectionArray(request_id=ident)
        result.header.stamp = Time(nanoseconds=now[0]).to_msg()
        f = Future(); f.set_result((token, result)); node.icp_worker.future = f
        node.finish_icp()
    try:
        request('locate','a'); token=(node.epoch,node.generation)
        done(token,'a'); assert len(published)==1
        now[0]+=1_000_000; request('barcode','b')
        done(token,'a'); assert len(published)==1
        now[0]+=1_000_000; request('locate','c'); token=(node.epoch,node.generation)
        now[0]+=2_000_000_000
        done(token,'c'); assert len(published)==1
        now[0]+=1_000_000; request('locate','d'); token=(node.epoch,node.generation)
        now[0]=1_000_000_000
        done(token,'d'); assert len(published)==1 and node.request is None
        assert node.icp_rejected==3
    finally:
        node.get_clock().now = original_clock
        ex.shutdown(); node.destroy_node(); rclpy.shutdown()
