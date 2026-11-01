"""Export the actually compiled dual-arm scan approach; no ROS interfaces."""
import json,os
from pathlib import Path
import numpy as np
os.environ.setdefault('MPLCONFIGDIR','/tmp/deux-parallel-scan-plot')
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation,PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[1];out=ROOT/'artifacts/dataset-barcode-pose';d=json.loads((out/'visualization-data.json').read_text());frames=[f for f in d['applied'] if 3.-1e-8<=f['t']<=d['scan_at_s']+1e-8]
def setup(ax):
 ax.set(xlim=(-.05,.85),ylim=(-.55,.55),zlim=(.65,1.65),xlabel='base X (m)',ylabel='base Y (m)',zlabel='base Z (m)');ax.set_box_aspect((.9,1.1,1));ax.view_init(23,-52)
 for p in d['polygons']:ax.add_collection3d(Poly3DCollection([p],facecolor='#8ba19d',alpha=.12))
 for side,color in [('left','#4089bd'),('right','#e47e2d')]:
  pts=np.array([f[side][-1] for f in frames]);ax.plot(*pts.T,c=color,alpha=.35)
 a=np.array(frames[0]['left'][0]);b=np.array(frames[0]['right'][0]);ax.plot(*np.stack([a,b]).T,c='#a2afba',lw=4);ax.plot([0,0],[0,0],[.65,(a[2]+b[2])/2],c='#a2afba',lw=4)
def draw(ax,frame):
 for side,color in [('left','#4089bd'),('right','#e47e2d')]:
  p=np.array(frame[side]);ax.plot(*p.T,'o-',c=color,lw=3)
  for i,v in enumerate(p[:7]):ax.text(*v,f'{side[0].upper()}J{i+1}',color=color,fontsize=7)
fig=plt.figure(figsize=(15,5.5))
for j,time in enumerate([4.,5.,6.]):
 ax=fig.add_subplot(1,3,j+1,projection='3d');setup(ax);f=min(frames,key=lambda f:abs(f['t']-time));draw(ax,f);ax.set_title({4.:'Parallel transport starts',5.:'Both arms moving',6.:'Right via / left scan reached'}[time]+f'\nt={time:.1f}s from departure')
fig.suptitle('Offline compiled dual-arm path | contact 3s; left moves 4-6s; right reaches scan at 8s');fig.tight_layout(rect=(0,0,1,.91));fig.savefig(out/'parallel-scan.png',dpi=150);plt.close(fig)
fig=plt.figure(figsize=(9,6));ax=fig.add_subplot(projection='3d');setup(ax);artists=[]
for side,color in [('left','#4089bd'),('right','#e47e2d')]:
 line,=ax.plot([],[],[],'o-',c=color,lw=3,label=side);artists.append(line)
ax.legend();title=ax.set_title('');fig.text(.5,.02,'Offline model preview. No hardware motion; no inter-arm or bread-bag volume collision certificate.',ha='center',fontsize=8)
def update(i):
 f=frames[i]
 for artist,side in zip(artists,['left','right']):
  p=np.array(f[side]);artist.set_data(p[:,0],p[:,1]);artist.set_3d_properties(p[:,2])
 title.set_text(f"Contact -> parallel via/left scan -> right scan | t={f['t']:.2f}s")
 return artists
anim=FuncAnimation(fig,update,frames=len(frames),interval=60);anim.save(out/'parallel-scan.gif',writer=PillowWriter(fps=round(len(frames)/5.)));print(out/'parallel-scan.gif')
