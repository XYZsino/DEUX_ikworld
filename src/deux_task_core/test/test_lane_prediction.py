import numpy as np
from deux_task_core.lane_prediction import LanePrediction

def test_tangent_velocity_and_constant_height():
 n=np.array([-.1,0,1.]);n/=np.linalg.norm(n);d=np.array([1.,0,.1]);d/=np.linalg.norm(d)
 model=LanePrediction([*n,-.75]);rng=np.random.default_rng(12)
 for t in np.linspace(0,2,21):
  result=model.update('a',t,np.array([.4,0,1.])+d*.08*t+n*rng.normal(0,.002))
 assert result is not None
 v=np.array(result['velocity']);p=np.array(result['position'])
 assert abs(v@n)<1e-10
 assert np.isclose(np.linalg.norm(v),.08,atol=.002)
 assert np.isclose((p+v*7)@n,p@n)
 assert model.update('b',2.1,[.5,0,1.]) is None
 assert model.update('a',4.,[.5,0,1.]) is None

def test_stationary_and_warmup():
 model=LanePrediction([0,0,1,-.75])
 for t in [0,.1,.2,.3,.4]:assert model.update('a',t,[.5,0,.9]) is None
 for t in [.5,.6,.7,.8]:out=model.update('a',t,[.5,0,.9])
 assert out is not None and np.allclose(out['velocity'],0)
