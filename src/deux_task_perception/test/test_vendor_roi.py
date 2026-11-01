from deux_task_perception.roi_tracker import RoiBagTracker


def test_entry_exit():
    t = RoiBagTracker([100, 0, 200, 200], [2])
    def box(x): return [[x, 0, x+100, 100, .9, 2]]
    assert t.update(box(40)) is None
    assert t.update(box(60)) is not None
    for x in range(70, 170, 10):
        out = t.update(box(x))
    assert out is None and t.tracker.track is None
    assert t.update(box(80)) is not None
