import numpy as np
import pytest
from recover_right_wrist import validate_recovery


def data():
    limits=np.tile([-2.,2.],(14,1)); i=12
    start=np.zeros(14); start[i]=2+np.deg2rad(1.8)
    goal=start.copy();goal[i]=2-np.deg2rad(5)
    t=np.linspace(0,4,201);u=t/4
    path=start+(3*u*u-2*u*u*u)[:,None]*(goal-start)
    return t,path,limits,np.ones(14),np.ones(14),i


def test_inward_recovery():validate_recovery(*data())


@pytest.mark.parametrize('kind',['outward','other','oversized','endpoint','speed','nonfinite'])
def test_reject_unsafe_recovery(kind):
    t,p,l,v,a,i=data()
    if kind=='outward':p[30,i]=p[29,i]+.01
    elif kind=='other':p[30,0]+=.01
    elif kind=='oversized':p[0,i]+=.1
    elif kind=='endpoint':p[-1,i]=l[i,1]+.01
    elif kind=='speed':t=t/10
    elif kind=='nonfinite':p[10,i]=np.nan
    with pytest.raises(ValueError):validate_recovery(t,p,l,v,a,i)
