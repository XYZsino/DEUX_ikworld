import numpy as np
import pytest
from deux_task_perception.single_bytetrack import SingleByteTracker


def rows(score=.9, x=0):
    return np.array([[x, 0, x+100, 100, score, 0], [400, 0, 500, 100, .99, 0]])


def test_requires_selection_and_no_extra_tracks():
    t = SingleByteTracker()
    assert t.update(rows()) is None
    t.select(rows(), 0)
    for x in range(1, 20):
        out = t.update(rows(x=x))
        assert out.detection_index == 0 and out.xyxy[0] < 100
    assert not hasattr(t, 'tracked_stracks')


def test_low_confidence_continuation_and_high_priority():
    t = SingleByteTracker()
    t.select(rows(), 0)
    assert t.update(rows(.15)).score == pytest.approx(.15)
    detections = np.array([[0, 0, 100, 100, .15, 0], [5, 0, 105, 100, .9, 0]])
    assert t.update(detections).detection_index == 1


def test_lost_requires_high_confidence_and_expiry_is_terminal():
    t = SingleByteTracker(buffer=3)
    t.select(rows(), 0)
    assert t.update([]) is None and t.status == 'lost'
    assert t.update(rows(.15)) is None
    assert t.update(rows()) is not None
    for _ in range(4):
        assert t.update([]) is None
    assert t.status == 'removed' and t.track is None
    assert t.update(rows()) is None
    t.select(rows(), 1)
    assert t.update(rows()).detection_index == 1


def test_invalid_inputs_and_zero_buffer():
    t = SingleByteTracker(buffer=0)
    with pytest.raises(ValueError):
        t.select_bbox([0, 0, 0, 20])
    t.select_bbox([0, 0, 100, 100])
    assert t.update([]) is None and t.status == 'removed'
    with pytest.raises(ValueError):
        t.update([[0, 0, 1, 1, float('nan'), 0]])


def test_yolo_results_adapter():
    from ultralytics.engine.results import Results
    result = Results(np.zeros((120, 600, 3), dtype=np.uint8), 'synthetic', {0: 'object'}, boxes=rows())
    t = SingleByteTracker()
    t.select(result, 1)
    assert t.update(result).detection_index == 1


def test_prediction_moves_through_gap_and_recovers():
    t = SingleByteTracker(buffer=10)
    t.select(rows(), 0)
    for x in range(5, 55, 5):
        assert t.update(rows(x=x)) is not None
    before = t.track.xyxy.copy()
    assert t.update([]) is None
    assert t.track.xyxy[0] > before[0]
    assert t.update(rows(x=60)) is not None


def test_detection_index_keeps_original_order_and_threshold_boundary():
    t = SingleByteTracker()
    t.select(rows(), 0)
    assert t.update(rows(.1)) is None  # low threshold is exclusive
    assert t.update(rows(.25)) is not None  # high threshold is inclusive
    t.reset()
    t.select(rows(), 0)
    reversed_rows = rows()[::-1].copy()
    assert t.update(reversed_rows).detection_index == 1


def test_tensor_results_adapter():
    import torch
    from ultralytics.engine.results import Results
    result = Results(np.zeros((120, 600, 3), dtype=np.uint8), 'synthetic', {0: 'object'}, boxes=torch.tensor(rows()))
    t = SingleByteTracker()
    t.select(result, 0)
    assert t.update(result).detection_index == 0


def test_visualization_preserves_prediction_and_measurement():
    t = SingleByteTracker()
    t.select(rows(), 0)
    out = t.update(rows(x=10))
    assert t.predicted_xyxy[0] == pytest.approx(0)
    assert t.observed_xyxy[0] == 10
    assert 0 < out.xyxy[0] < 10
    assert t.update([]) is None
    assert t.predicted_xyxy is not None and t.observed_xyxy is None
    t.reset()
    assert t.predicted_xyxy is None
