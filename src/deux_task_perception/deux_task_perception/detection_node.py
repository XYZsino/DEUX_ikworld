"""One YOLO model, latest RGB frame, attached original registered depth."""
from copy import deepcopy
from pathlib import Path
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from deux_task_msgs.msg import Observation, DetectionFrame, BoundingBox
from .core import normalize_rgb, validate_bundle
from .pipeline_node import PipelineNode, run
from .yolo_backend import YoloDetector


class DetectionNode(PipelineNode):
    def __init__(self, detector=None, **kwargs):
        super().__init__('detection_node', {
            'input_topic': '/task/observation', 'output_topic': '/task/detection_frames',
            'weights': '', 'device': 'cpu', 'confidence': .25,
            'target_classes': ['s1', 's1_1'], 'width': 640, 'height': 480,
            'sync_slop_s': .02}, **kwargs)
        self.positive('confidence', 'sync_slop_s', 'width', 'height')
        if self.cfg['confidence'] > 1:
            raise ValueError('confidence must be <= 1')
        if detector is None:
            if not self.cfg['weights'] or not Path(self.cfg['weights']).is_file():
                raise ValueError('weights must be an explicit existing YOLO model file')
            detector = YoloDetector(self.cfg['weights'], self.cfg['device'], self.cfg['confidence'])
        self.detector = detector
        self.publisher = self.create_publisher(DetectionFrame, self.cfg['output_topic'], qos_profile_sensor_data)
        self.start_worker(self.detect)
        self.subscription = self.create_subscription(Observation, self.cfg['input_topic'],
                                                      self.enqueue, qos_profile_sensor_data)

    def detect(self, obs):
        if obs.header != obs.rgb.header:
            raise ValueError('Observation acquisition header differs from RGB')
        validate_bundle(obs.rgb, obs.depth, obs.rgb_info, obs.depth_info,
                        self.cfg['width'], self.cfg['height'], int(self.cfg['sync_slop_s']*1e9))
        out = DetectionFrame()
        out.header = deepcopy(obs.header)
        out.sequence = obs.sequence
        out.detector_id = self.detector.identity
        out.depth, out.depth_info = obs.depth, obs.depth_info
        for cls, name, confidence, box in self.detector(normalize_rgb(obs.rgb)):
            if name not in self.cfg['target_classes']:
                continue
            if (not np.isfinite([confidence, *box]).all() or len(box) != 4 or
                    not self.cfg['confidence'] <= confidence <= 1 or
                    box[2] <= box[0] or box[3] <= box[1]):
                raise ValueError('invalid detector result')
            out.boxes.append(BoundingBox(class_id=cls, class_name=name,
                                         confidence=confidence, xyxy=list(map(float, box))))
        return out


def main(args=None):
    run(DetectionNode, args)
