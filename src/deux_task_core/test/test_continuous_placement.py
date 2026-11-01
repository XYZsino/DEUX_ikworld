import numpy as np
from test_fixed_placement import configured
from deux_task_core.planning import continuous_joint_path,Planner
from deux_task_core.placement import stages

def test_arc_timing_avoids_slow_internal_knots_and_obeys_limits():
 v=np.ones(2)*2;a=np.ones(2)*3
 t,q=continuous_joint_path([[0,0],[.1,.05],[.2,.15],[1,.5]],v,a,.002)
 speed=np.diff(q,axis=0)/.002;acc=np.diff(speed,axis=0)/.002
 assert np.max(abs(speed)/v)<1 and np.max(abs(acc)/a)<1
 norm=np.linalg.norm(speed,axis=1);mid=norm[int(len(norm)*.22):int(len(norm)*.78)]
 assert mid.min()/mid.max()>.99
 np.testing.assert_allclose(q[[0,-1]],[[0,0],[1,.5]],atol=1e-9)

def test_single_plan_includes_release_hold_and_return(tmp_path):
 c=configured(tmp_path);c['placement']['continuous_motion']=True
 c['hand_open_duration_s']=.5
 p=Planner(c)
 def forbidden(*a,**kw):raise AssertionError('IK invoked')
 p.robot.ik=forbidden
 q=np.zeros(28);q[7:14]=c['hand_closed'][:7];q[21:]=c['hand_closed'][7:]
 result=p.plan('PlaceApproach',q,None);t=result['times'];path=result['path'];a,b,end=result['phase_end_times']
 assert stages(c)==('ClearReader','PlaceApproach')
 assert abs(b-a-.5)<1e-9 and abs(end-t[-1])<1e-9
 hold=path[(t>=a-1e-8)&(t<=b+1e-8)]
 np.testing.assert_allclose(hold[:,14:21],np.repeat(hold[:1,14:21],len(hold),axis=0))
 np.testing.assert_allclose(path[-1,14:21],c['placement']['fixed_joint_waypoints']['Retreat'][-1],atol=1e-9)
 np.testing.assert_allclose(path[-1,21:],c['hand_open'][7:])
 np.testing.assert_allclose(path[t<=a+1e-8,21:],np.tile(c['hand_closed'][7:],(sum(t<=a+1e-8),1)))
