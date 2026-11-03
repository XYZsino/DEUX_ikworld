import numpy as np
import pytest
from fixed_interception import select,arrival,schedule,target,close_offset
from scheduled_descent import profile_for

P=dict(xy_min_m=[.52,-.25],xy_max_m=[.67,.02],entry_margin_m=.005,descent_speed_m_s=.06,max_descent_m=.05,close_height_m=.015,close_duration_s=.5,close_xy_m=.02)
def fixture():
 obj=np.eye(4);obj[:3,3]=[.6,.1,.9]
 v=np.array([0.,-.07,0.,0.]);g=np.eye(4)
 frozen,fixed=select(obj,v,g,1.2,.05,P)
 return obj,v,g,fixed

def test_fixed_point_no_moving_lookahead():
 obj,v,g,f=fixture();initial=np.eye(4);initial[:3,3]=[.58,-.1,1.]
 a,va=target(initial,f,.05,1.2,1.3)
 b,vb=target(initial,f,.05,1.2,2.2)
 np.testing.assert_allclose(a[:3,3],b[:3,3]);np.testing.assert_allclose(va[:3],0);np.testing.assert_allclose(vb[:3],0)

def test_arrival_updates_without_changing_point():
 obj,v,g,f=fixture();before=np.array(f['pose']).copy()
 t,_=arrival(f,obj,v,g,1.)
 moved=obj.copy();moved[1,3]+=.014
 later,_=arrival(f,moved,v,g,1.)
 assert later==pytest.approx(t+.2)
 np.testing.assert_array_equal(before,f['pose'])

def test_schedule_hits_close_height_at_contact_and_waits():
 obj,v,g,f=fixture();p=schedule(f,obj,v,g,1.3,.05,P)
 initial=np.eye(4)
 a,va=target(initial,f,.05,1.2,p['start']-.01,p)
 b,vb=target(initial,f,.05,1.2,p['contact_s'],p)
 assert a[2,3]==pytest.approx(.95)
 assert b[2,3]==pytest.approx(.915)
 np.testing.assert_allclose(b[:2,3],np.array(f['pose'])[:2,3])
 c,vc=target(initial,f,.05,1.2,p['contact_s']+.3,p)
 assert vc[1]==pytest.approx(v[1]);assert c[1,3]<b[1,3]

def test_missed_and_lateral_miss_rejected():
 obj,v,g,f=fixture()
 with pytest.raises(ValueError,match='too late'):schedule(f,obj,v,g,5.,.05,P)
 obj[0,3]+=.03
 with pytest.raises(ValueError,match='misses'):schedule(f,obj,v,g,1.3,.05,P)

def test_workspace_and_unreachable_height():
 obj,v,g,f=fixture();obj[0,3]=.8
 with pytest.raises(ValueError,match='workspace'):select(obj,v,g,1.2,.05,P)
 with pytest.raises(ValueError,match='cannot reach'):close_offset(profile_for(.1,0.,.06,.05),.015)

def test_candidates_spatial_invariance_and_rebase():
 from fixed_interception import candidates,rebase
 obj,v,g,f=fixture()
 a=candidates(obj,v,g,1.2,.05,P)
 newer=obj.copy();newer[:3,3]+=v[:3]*.1
 b=candidates(newer,v,g,1.2,.05,P)
 np.testing.assert_allclose(a[0][1]['pose'],b[0][1]['pose'])
 plan=dict(fixed_interception=a[0][1],grasp=g.tolist(),approach_s=1.2,times=[0.,1.],path=[[1],[2]])
 updated=rebase(plan,newer,v,100.,100.3,P,.05)
 np.testing.assert_array_equal(updated['fixed_interception']['pose'],plan['fixed_interception']['pose'])
 assert updated['path']==plan['path']
 assert updated['fixed_interception']['nominal_contact_s']==pytest.approx(a[0][1]['nominal_contact_s']-.4)
 with pytest.raises(ValueError,match='no longer reachable'):rebase(plan,newer,v,100.,104.,P,.05)
