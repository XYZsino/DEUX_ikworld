"""Scan presence retains current-frame raw boxes without changing ROI output."""
import unittest
from unittest.mock import patch
import numpy as np
from deux_task_perception.single_target_backend import SingleTargetDetector
from deux_task_perception.yolo_backend import YoloDetector


class RawScanBagTests(unittest.TestCase):
    def test_raw_bags_survive_roi_rejection_and_refresh_each_frame(self):
        detector=SingleTargetDetector.__new__(SingleTargetDetector)
        detector.bag_ids=[0];detector.high=.5
        detector.update_rows=lambda *args:[]  # ROI rejects this bag.
        with patch.object(YoloDetector,'__call__',return_value=[
                (0,'bread_bag',.5,[1,2,31,42]),(0,'bread_bag',.2204,[1,2,30,40]),
                (0,'bread_bag',.2029,[1,2,30,40]),
                (1,'barcode',.9,[1,2,8,9])]):
            self.assertEqual(detector.track(np.zeros((100,100,3)),10.),[])
            self.assertEqual(len(detector.scan_bags),1)
            self.assertEqual(detector.scan_bags[0]['confidence'],.5)
            self.assertEqual(detector.scan_bags[0]['xyxy'],[1,2,31,42])
        with patch.object(YoloDetector,'__call__',return_value=[]):
            detector.track(np.zeros((100,100,3)),11.)
            self.assertEqual(detector.scan_bags,[])


if __name__=='__main__':unittest.main()
