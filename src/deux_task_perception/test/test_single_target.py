import json
from types import SimpleNamespace
import numpy as np
import pytest
from deux_task_perception.single_target_backend import SingleTargetDetector
from deux_task_perception.tracking_roi import write_roi,validate_roi

def tracker(tmp_path):
 t=SingleTargetDetector.__new__(SingleTargetDetector);t.model=SimpleNamespace(names={0:'s1',1:'bar'});t.bag_ids=[0];t.barcode_name='bar';t.high=.9;t.roi_file=str(tmp_path/'roi.json');t.max_gap_s=.75;t.tracker=None;t.roi_key=None;t.last_stamp=None;t.last_seen=None;t.public_id='';t.session='test';t.serial=0;t.hold_target=False
 return t

def rows(x,score=.95):return np.array([[x,10,x+20,30,score,0]],float)
def roi(t):write_roi(t.roi_file,dict(width=100,height=100,points=[[30,0],[90,0],[90,90],[30,90]]))
def update(t,x,stamp,score=.95):return t.update_rows(rows(x,score),(100,100),stamp)
def test_requires_roi_and_entry(tmp_path):
 t=tracker(tmp_path);assert update(t,21,1)==[];roi(t)
 assert update(t,21,2)==[] # starts inside: not an entry
 assert update(t,15,2.1)==[]
 out=update(t,21,2.2);assert len(out)==1 and out[0]['track_id'].startswith('single-')
 assert t.tracker.tracker.track is not None

def test_low_confidence_recovers_and_expiry_does_not_replace_task(tmp_path):
 t=tracker(tmp_path);roi(t);update(t,15,1);first=update(t,21,1.1)[0];t.hold_target=True
 out=update(t,23,1.2,.4);assert out[0]['track_id']==first['track_id']
 assert t.update_rows(np.empty((0,6)),(100,100),1.3)==[]
 assert update(t,24,2.1)==[] and update(t,24,2.2)==[]

def test_roi_edit_invalidates_identity_and_barcode_stays_independent(tmp_path):
 t=tracker(tmp_path);roi(t);update(t,15,1);first=update(t,21,1.1)[0]['track_id']
 write_roi(t.roi_file,dict(width=100,height=100,points=[[40,0],[90,0],[90,90],[40,90]]))
 assert update(t,25,1.2)==[];second=update(t,31,1.3)[0]['track_id'];assert second!=first
 out=t.update_rows(np.array([[10,10,15,15,.6,1]]),(100,100),1.4);assert out[0]['class_name']=='bar'

def test_roi_rejects_crossed_points():
 with pytest.raises(ValueError):validate_roi(dict(width=100,height=100,points=[[0,0],[99,99],[0,99],[99,0]]))

def test_inside_policy_acquires_stationary_bag_and_preserves_identity(tmp_path):
 t=tracker(tmp_path);t.require_entry=False;roi(t)
 first=update(t,79,1)[0]['track_id']
 assert update(t,79,1.1)[0]['track_id']==first
 assert update(t,81,1.2)==[]
 assert update(t,79,1.3)[0]['track_id']!=first

def test_inside_policy_requires_roi_confidence_and_bag_class(tmp_path):
 t=tracker(tmp_path);t.require_entry=False
 assert update(t,21,1)==[]
 roi(t)
 assert update(t,21,1.1,.4)==[]
 assert update(t,0,1.2)==[]
 assert len(update(t,21,1.3))==1
