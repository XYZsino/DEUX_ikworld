"""Offline skeleton preview and before/after metrics; no hardware interfaces."""
import os
os.environ.setdefault('MPLCONFIGDIR','/tmp/deux-ready-mpl')
import json
from copy import deepcopy
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation,PillowWriter
from build_right_grasp_grid import Builder,ROOT
from lookup_bt_support import prepare_plan
from deux_task_core.prepare_route import knots


def main():
    b=Builder(ROOT/'src/deux_task_bringup/config/installation.yaml');r=b.robot
    start=b.ready.copy();start[:7]=0;start[14:21]=0
    c=deepcopy(b.config);c['robot']['collision_check_enabled']=False;c['obstacles']=[]
    from deux_task_core.model import RobotModel
    old=np.asarray(knots(c,start,b.ready,RobotModel(c['robot'])))
    plan=prepare_plan(b.config,b.ready,np.zeros(14),velocity_cap=.45,acceleration_cap=.9)
    path=plan['path'];times=plan['times']
    def skeleton(q,side):
        ts=r.fk(q);prefix='arm_l_joint' if side=='left' else 'arm_r_joint'
        points=[ts[j['child']][:3,3] for n,j in r.ee_chains[side] if n.startswith(prefix)]
        return np.array([*points,r.ee_fk(side,q)[:3,3]])
    def trace(path):
        return np.array([r.ee_fk('right',(1-u)*a+u*z)[:3,3] for a,z in zip(path[:-1],path[1:]) for u in np.linspace(0,1,101)])
    traces=[trace(old),trace(path)]
    def metrics(points):
        return dict(xy_length_m=float(np.linalg.norm(np.diff(points[:,:2],axis=0),axis=1).sum()),
                    xyz_range_m=np.ptp(points,axis=0).tolist())
    out=ROOT/'artifacts/compact-attention-ready';out.mkdir(exist_ok=True)
    summary=dict(old=metrics(traces[0]),new=metrics(traces[1]),duration_s=float(times[-1]),
                 minimum_clearance_m=plan['minimum_clearance'],times_s=times.tolist(),positions_rad=path.tolist(),
                 verification='Offline measured-URDF/conveyor capsule model. No hardware execution; self/inter-arm collision not certified.')
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    views=[(0,2,'Side (X/Z)',(-.15,.85),(.6,1.45)),(1,2,'Front (Y/Z)',(-.8,.6),(.6,1.45)),(0,1,'Top (X/Y)',(-.15,.85),(-.8,.6))]
    fig,axes=plt.subplots(1,3,figsize=(12,5));lines=[]
    for ax,(i,j,title,xlim,ylim) in zip(axes,views):
        ax.plot(traces[0][:,i],traces[0][:,j],'--',color='#aaa',label='Old right-hand path')
        ax.plot(traces[1][:,i],traces[1][:,j],color='#ed9b39',alpha=.55,label='New right-hand path')
        right,=ax.plot([],[],'o-',color='#d96818',lw=3,ms=4,label='Right arm')
        left,=ax.plot([],[],'o-',color='#277ea3',lw=3,ms=4,label='Left arm')
        lines.append((right,left));ax.set(title=title,xlim=xlim,ylim=ylim,xlabel='XYZ'[i]+' (m)',ylabel='XYZ'[j]+' (m)');ax.set_aspect('equal');ax.grid(alpha=.2)
    axes[0].legend(fontsize=7,loc='upper right')
    title=fig.suptitle('');fig.text(.5,.015,'Offline skeleton model | shoulders retract + elbows fold, lift close, then extend | NO hardware motion',ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.04,1,.91))
    def draw(t):
        index=min(np.searchsorted(times,t,side='right')-1,len(times)-2);index=max(0,index)
        u=np.clip((t-times[index])/(times[index+1]-times[index]),0,1);u=3*u*u-2*u*u*u
        q=(1-u)*path[index]+u*path[index+1]
        for pair,(i,j,*_) in zip(lines,views):
            for line,side in zip(pair,('right','left')):
                p=skeleton(q,side);line.set_data(p[:,i],p[:,j])
        stage=['Retract + fold','Lift close to body','Extend to right ready','Left to ready'][index]
        title.set_text(f'Attention -> ready | {stage} | {t:.1f} / {times[-1]:.1f} s')
    draw(times[2]);fig.savefig(out/'skeleton.png',dpi=140)
    animation=FuncAnimation(fig,draw,frames=np.linspace(0,times[-1],int(np.ceil(times[-1]*12))+1),interval=1000/12)
    animation.save(out/'skeleton.gif',writer=PillowWriter(fps=12))
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
