"""Generate interactive skeleton and GIF for the combined, synthetic placement."""
import json,sys,os
from pathlib import Path
import numpy as np,yaml
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/deux_task_core'))
from deux_task_core.model import RobotModel
from deux_task_core.conveyor_screen import surfaces
out=ROOT/'artifacts/parallel-placement-offline';d=json.loads((out/'example-plan.json').read_text());t=np.asarray(d['times_s']);q=np.asarray(d['positions_rad'])
p=ROOT/'src/deux_task_bringup/config/installation.yaml';config=yaml.safe_load(p.read_text());config['robot']['urdf_path']=str((p.parent/config['robot']['urdf_path']).resolve());robot=RobotModel(config['robot'])
frames=[]
for idx in sorted(set(list(range(0,len(t),3))+[len(t)-1])):
    fk=robot.fk(q[idx]);frame={'t':float(t[idx])}
    for side,prefix in [('left','arm_l_joint'),('right','arm_r_joint')]:
        frame[side]=[fk[robot.joints[prefix+str(i)]['child']][:3,3].tolist() for i in range(1,8)]+[fk[robot.ee[side]][:3,3].tolist()]
    frames.append(frame)
visual=dict(frames=frames,polygons=[v.tolist() for v,_,_ in surfaces()[3]],report=d['report'],phases=d['phase_end_times'])
html='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>병렬 후퇴·적재 스켈레톤</title><style>
body{background:#eef2f6;color:#183043;font:16px system-ui;max-width:1180px;margin:24px auto;padding:0 20px}h1{font-size:26px}p{line-height:1.7}canvas{background:#fff;border-radius:14px;width:100%;height:580px;touch-action:none}.controls{display:flex;gap:12px;align-items:center;margin:16px 0}button,select{padding:8px 14px;border:1px solid #aab9c4;border-radius:7px;background:white}input{flex:1}.badge{background:#fff1cc;padding:6px 12px;border-radius:7px;font-size:13px}.bar{height:17px;background:#dce4ea;border-radius:5px;position:relative;overflow:hidden;margin:8px 0 16px}.fill{height:100%;position:absolute;left:0}.cursor{position:absolute;top:0;height:100%;border-left:3px solid #193d53}.row{display:grid;grid-template-columns:130px 1fr;align-items:center}.stage{font-weight:650;color:#187d69}small{color:#5e7180;line-height:1.7}
</style><span class="badge">오프라인 FK · 合成 시작 자세 · 실기 미검증</span><h1>왼손 후퇴 + 오른손 적재·후퇴</h1><p>스캔 정렬 후 양팔을 동시에 시작합니다. 왼손이 물러나는 동안 오른손이 적재 위치로 이동하고,<br>왼손 후퇴가 끝난 뒤 오른손을 열고 후퇴합니다. 드래그: 시점 회전 · 휠: 확대.</p><canvas id="scene"></canvas><div class="controls"><button id="play">일시정지</button><button id="reset">처음</button><input id="time" type="range" min="0" step=".01"><span id="clock"></span><select id="speed"><option value=".5">0.5×</option><option selected value="1">1×</option></select><button id="front">정면</button><button id="top">위에서</button></div><div class="stage" id="stage"></div><div id="timelines"></div><p id="stats"></p><small>청색: 왼팔 / 주황: 오른팔 / 회색 면: 실측 컨베이어. 왼팔 정렬 종료 자세는 예시로 구성했습니다. 실제 시작 자세에 따라 시간이 달라집니다. 손가락과 빵 봉투의 부피는 표시하지 않았습니다. 양팔 상호 충돌 검사는 포함하지 않습니다.</small><script>
const D=__DATA__,cv=document.querySelector('canvas'),ctx=cv.getContext('2d'),sl=document.querySelector('#time'),total=D.frames.at(-1).t;
sl.max=total;let t=0,playing=true,last=0,az=-.95,el=.42,zoom=1,W=900,H=580,drag=null;
const phases=D.phases,leftEnd=D.report.left_retreat_s;
for(let [label,end,color]of [['왼손 후퇴',leftEnd,'#428ac0'],['오른손 적재',phases[0],'#e4812f'],['손 열기',phases[1],'#39a581'],['오른손 후퇴',total,'#bb7a3c']]){let start=label==='손 열기'?phases[0]:label==='오른손 후퇴'?phases[1]:0;document.querySelector('#timelines').innerHTML+=`<div class="row"><span>${label}</span><div class="bar"><div class="fill" style="left:${start/total*100}%;width:${(end-start)/total*100}%;background:${color}"></div><div class="cursor"></div></div></div>`}
document.querySelector('#stats').textContent=`동시 시작 · 왼손 후퇴 완료 ${leftEnd.toFixed(2)}s · 적재 위치 ${phases[0].toFixed(2)}s · 손 열기 ${phases[0].toFixed(2)}~${phases[1].toFixed(2)}s · 전체 ${total.toFixed(2)}s`;
function project(p){let x=p[0]-.24,y=p[1],z=p[2]-.8,s=Math.min(W/1.9,H/1.7)*zoom;return[W/2+(-Math.sin(az)*x+Math.cos(az)*y)*s,H/2-(-Math.cos(az)*Math.sin(el)*x-Math.sin(az)*Math.sin(el)*y+Math.cos(el)*z)*s]}
function line(ps,c,w=2){ctx.beginPath();ps.forEach((p,i)=>{let v=project(p);i?ctx.lineTo(...v):ctx.moveTo(...v)});ctx.strokeStyle=c;ctx.lineWidth=w;ctx.lineJoin='round';ctx.lineCap='round';ctx.stroke()}
function dot(p,c,r=4){ctx.beginPath();ctx.arc(...project(p),r,0,Math.PI*2);ctx.fillStyle=c;ctx.fill()}
function pose(time){let i=0;while(i<D.frames.length-2&&D.frames[i+1].t<time)i++;let a=D.frames[i],b=D.frames[i+1],u=Math.max(0,Math.min(1,(time-a.t)/(b.t-a.t)));return Object.fromEntries(['left','right'].map(s=>[s,a[s].map((p,j)=>p.map((v,k)=>v+(b[s][j][k]-v)*u))]))}
function draw(){ctx.clearRect(0,0,W,H);for(let poly of D.polygons){ctx.beginPath();poly.forEach((p,i)=>{let v=project(p);i?ctx.lineTo(...v):ctx.moveTo(...v)});ctx.closePath();ctx.fillStyle='#8a9d9d33';ctx.fill();ctx.strokeStyle='#82999988';ctx.lineWidth=1;ctx.stroke()}
let f=pose(t),a=f.left[0],b=f.right[0],mid=a.map((v,i)=>(v+b[i])/2);line([[0,0,0],[0,0,mid[2]],mid],'#9aa8b4',7);line([a,b],'#9aa8b4',6);
for(let [side,color]of [['left','#428ac0'],['right','#e4812f']]){line(D.frames.map(f=>f[side].at(-1)),color+'66',2);line(f[side],color,6);f[side].forEach(p=>dot(p,color));dot(f[side].at(-1),color,7)}
sl.value=t;document.querySelector('#clock').textContent=t.toFixed(2)+' / '+total.toFixed(2)+' s';document.querySelectorAll('.cursor').forEach(e=>e.style.left=t/total*100+'%');document.querySelector('#stage').textContent=t<leftEnd?'양팔 동시 이동: 왼손 후퇴 + 오른손 적재 접근':t<phases[0]?'왼손 후퇴 완료 · 오른손 적재 접근':t<phases[1]?'양팔 정지 · 오른손 열기':t<total-.01?'왼손 후퇴 자세 유지 · 오른손 후퇴':'완료';}
function resize(){W=cv.clientWidth;H=cv.clientHeight;let d=devicePixelRatio||1;cv.width=W*d;cv.height=H*d;ctx.setTransform(d,0,0,d,0,0);draw()}addEventListener('resize',resize);
document.querySelector('#play').onclick=()=>{if(t>=total)t=0;playing=!playing;document.querySelector('#play').textContent=playing?'일시정지':'재생'};document.querySelector('#reset').onclick=()=>{t=0;draw()};sl.oninput=()=>{t=+sl.value;playing=false;draw()};document.querySelector('#front').onclick=()=>{az=0;el=.1;draw()};document.querySelector('#top').onclick=()=>{az=-Math.PI/2;el=Math.PI/2;draw()};cv.onpointerdown=e=>{drag=[e.clientX,e.clientY,az,el];cv.setPointerCapture(e.pointerId)};cv.onpointermove=e=>{if(drag){az=drag[2]-(e.clientX-drag[0])*.008;el=Math.max(.02,Math.min(Math.PI/2,drag[3]+(e.clientY-drag[1])*.006));draw()}};cv.onpointerup=()=>drag=null;cv.onwheel=e=>{e.preventDefault();zoom=Math.max(.5,Math.min(2.5,zoom*Math.exp(-e.deltaY*.001)));draw()};window.seekTo=s=>{t=Math.min(total,Math.max(0,s));playing=false;draw()};function tick(n){if(playing&&last){t=Math.min(total,t+Math.min(.1,(n-last)/1000)*+document.querySelector('#speed').value);if(t===total)playing=false}last=n;draw();requestAnimationFrame(tick)}resize();requestAnimationFrame(tick);
</script></html>'''
(out/'preview.html').write_text(html.replace('__DATA__',json.dumps(visual,separators=(',',':'))).replace('合成','예시'))
(out/'skeleton-frames.json').write_text(json.dumps(visual,separators=(',',':')))
os.environ.setdefault('MPLCONFIGDIR','/tmp/deux-parallel-plot');import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation,PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
fig=plt.figure(figsize=(9,6));ax=fig.add_subplot(projection='3d');ax.set(xlim=(-.1,1.),ylim=(-.7,.55),zlim=(.65,1.65),xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)');ax.set_box_aspect((1.1,1.25,1.));ax.view_init(elev=23,azim=-52)
for poly in visual['polygons']:ax.add_collection3d(Poly3DCollection([poly],facecolor='#879e9c',alpha=.18))
artists=[]
for side,color in [('left','#428ac0'),('right','#e4812f')]:
    path=np.array([f[side][-1] for f in frames]);ax.plot(*path.T,c=color,alpha=.4);artist,=ax.plot([],[],[],'o-',c=color,lw=3,label=side);artists.append(artist)
a=np.asarray(frames[0]['left'][0]);b=np.asarray(frames[0]['right'][0]);mid=(a+b)/2;ax.plot(*np.stack([a,b]).T,c='#9aa8b4',lw=5);ax.plot([0,0],[0,0],[.65,mid[2]],c='#9aa8b4',lw=5);ax.legend();title=ax.set_title('');fig.text(.5,.02,'Offline FK | synthetic reader start | not hardware verified',ha='center',fontsize=9)
leftEnd=d["report"]["left_retreat_s"];phases=d["phase_end_times"]
def update(i):
    f=frames[i]
    for artist,side in zip(artists,['left','right']):
        p=np.asarray(f[side]);artist.set_data(p[:,0],p[:,1]);artist.set_3d_properties(p[:,2])
    stage='Both arms move' if f['t']<leftEnd else 'Right approach' if f['t']<phases[0] else 'Open hand (stationary)' if f['t']<phases[1] else 'Right retreat'
    title.set_text(f"Parallel reader retreat + placement | {f['t']:.2f}s | {stage}");return artists
anim=FuncAnimation(fig,update,frames=len(frames),interval=60);anim.save(out/'motion.gif',writer=PillowWriter(fps=round(len(frames)/t[-1])));print((out/'preview.html').as_uri())
