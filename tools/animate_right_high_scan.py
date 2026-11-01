"""Export model skeleton GIF and three-pose image; no robot interfaces."""
import json,os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR','/tmp/deux-high-scan-matplotlib')
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation,PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[1];out=ROOT/'artifacts/right-high-scan-waypoint';d=json.loads((out/'proposal.json').read_text());frames=d['frames']

def setup(ax):
    ax.set(xlim=(-.1,.88),ylim=(-.65,.4),zlim=(.68,1.65),xlabel='X (m)',ylabel='Y (m); right = -Y',zlabel='Z (m)')
    ax.set_box_aspect((1.,1.05,1.));ax.view_init(elev=23,azim=-52)
    for polygon in d['conveyor_polygons']:ax.add_collection3d(Poly3DCollection([polygon],facecolor='#879e9c',edgecolor='#7a8e8c',alpha=.17))
    new=np.asarray(d['ee_path_m']);old=np.asarray(d['old_ee_path_m'])
    ax.plot(*new.T,c='#139677',linewidth=2,label='New high/right route')
    ax.plot(*old.T,c='#b96d76',linewidth=1.6,linestyle='--',alpha=.6,label='Previous route')
    for name,key,color in [('Grasp','grasp_xyz_m','#d5752a'),('Via point','waypoint_xyz_m','#139677'),('Scan','scan_xyz_m','#377bc4')]:
        p=np.asarray(d[key]);ax.scatter(*p,color=color,s=30);ax.text(*p,name,fontsize=8,color=color)
    a=np.asarray(frames[0]['left'][0]);b=np.asarray(frames[0]['right'][0]);mid=(a+b)/2
    ax.plot([0,mid[0]],[0,mid[1]],[.68,mid[2]],c='#a0aab4',lw=5);ax.plot(*np.stack([a,b]).T,c='#a0aab4',lw=5)

def pose(ax,f):
    for side,col in [('left','#94a2b0'),('right','#e78029')]:
        p=np.asarray(f[side]);ax.plot(*p.T,'o-',color=col,linewidth=3,markersize=4)
fig=plt.figure(figsize=(15,5))
for i,(name,t) in enumerate([('Grasp',0.),('Right/lower via',d['waypoint_at_s']),('Scan',frames[-1]['t'])]):
    ax=fig.add_subplot(1,3,i+1,projection='3d');setup(ax);f=min(frames,key=lambda f:abs(f['t']-t));pose(ax,f);ax.set_title(f'{name} | contact + {t:.2f}s')
fig.suptitle('Offline FK skeleton | smooth right-side exit; below scan height | not deployed; incoming bag volume unmeasured',fontsize=12);fig.tight_layout();fig.savefig(out/'skeleton-poses.png',dpi=150);plt.close(fig)
fig=plt.figure(figsize=(9,6));ax=fig.add_subplot(projection='3d');setup(ax)
left,=ax.plot([],[],[],'o-',color='#94a2b0',lw=3,ms=5);right,=ax.plot([],[],[],'o-',color='#e78029',lw=4,ms=5)
ax.legend(loc='lower left',fontsize=8);title=ax.set_title('');fig.text(.5,.02,'Offline model only. Contact remains at original 3s; no dynamic bread-bag collision certificate.',ha='center',fontsize=8)
def update(i):
    f=frames[i]
    for artist,side in [(left,'left'),(right,'right')]:
        p=np.asarray(f[side]);artist.set_data(p[:,0],p[:,1]);artist.set_3d_properties(p[:,2])
    title.set_text(f"Grasp -> right/lower via -> scan | contact + {f['t']:.2f}s")
    return left,right,title
anim=FuncAnimation(fig,update,frames=len(frames),interval=60,blit=False);anim.save(out/'motion.gif',writer=PillowWriter(fps=round(len(frames)/frames[-1]['t'])))
print(out/'motion.gif');print(out/'skeleton-poses.png')
