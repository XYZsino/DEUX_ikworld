from test_single_target import tracker,roi,update

def test_locate_lease_does_not_bypass_roi_exit(tmp_path):
 t=tracker(tmp_path);roi(t);update(t,15,1.);first=update(t,21,1.1)[0]['track_id'];t.hold_target=True
 assert update(t,24,1.2)[0]['track_id']==first
 assert update(t,15,1.3)==[]
 assert t.exit_event['target_id']==first and t.tracker.tracker.track is None
 assert update(t,21,1.4)==[] # old locate lease must never reacquire
 t.hold_target=False
 assert update(t,15,1.5)==[]
 second=update(t,21,1.6)[0]['track_id'];assert first!=second

def test_center_is_roi_boundary_criterion(tmp_path):
 t=tracker(tmp_path);roi(t);update(t,15,1.)
 # Box overlaps ROI while center remains outside: no entry.
 assert update(t,19,1.1)==[]
 assert update(t,20,1.2) # center on boundary counts as inside
 t.hold_target=True
 assert update(t,19,1.3)==[]
