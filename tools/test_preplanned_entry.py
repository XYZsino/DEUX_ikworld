import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path('tools').resolve()))
from progressive_descent import target

def test_one_second_upper_lead_and_zero_lead_at_contact():
 initial=np.eye(4);obj=np.eye(4);obj[:3,3]=[.6,0.,.9];v=np.array([0.,-.08,0.,0.]);grasp=np.eye(4)
 pose,feed=target(initial,obj,v,grasp,.08,1.2,2.,None,lookahead=1.)
 assert np.isclose(pose[1,3],-.24)
 profile=dict(start=2.,duration=1.,height0=.08,height1=0.,velocity0=0.)
 pose,feed=target(initial,obj,v,grasp,.08,1.2,3.,profile,lookahead=1.)
 assert np.isclose(pose[1,3],-.24)
 assert np.isclose(pose[2,3],.9)
 assert np.isclose(feed[1],-.08)
 t=2.5;eps=1e-5
 a,_=target(initial,obj,v,grasp,.08,1.2,t-eps,profile,lookahead=1.)
 b,_=target(initial,obj,v,grasp,.08,1.2,t+eps,profile,lookahead=1.)
 _,feed=target(initial,obj,v,grasp,.08,1.2,t,profile,lookahead=1.)
 np.testing.assert_allclose((b[:3,3]-a[:3,3])/(2*eps),feed[:3],atol=1e-6)
