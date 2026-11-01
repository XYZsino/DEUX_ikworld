"""Offline example IK / reader-target comparison, with named URDF joints."""
import sys,json,os
from pathlib import Path
import numpy as np,yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src/deux_task_core'),str(ROOT/'src/deux_task_runtime')]
from deux_task_core.model import RobotModel
from deux_task_core.barcode_position import reader_look_at
from deux_task_runtime.reader_yaw import rotate_target
p=ROOT/'src/deux_task_bringup/config/installation.yaml';c=yaml.safe_load(p.read_text());c['robot']['urdf_path']=str((p.parent/c['robot']['urdf_path']).resolve());robot=RobotModel(c['robot'])
q0=np.array(c['home_joints']);q0[14:21]=c['scan_right_joints'];offset=np.array(c['left_ee_from_reader']);current=robot.ee_fk('left',q0)@offset
# Explicitly hypothetical barcode location for a direction illustration, not camera data.
point=robot.ee_fk('right',q0)[:3,3];target=reader_look_at(current,point,c['read_distance'],c.get('barcode_aim_offset_base_m',[0,0,0]),yaw_offset_deg=-30.)
out=ROOT/'artifacts/reader-yaw-comparison';out.mkdir(exist_ok=True);poses=[];rng=np.random.default_rng(17)
for extra in [0.,-15.]:
 desired=rotate_target(target,extra)
 def residual(v):
  q=q0.copy();q[:7]=v;actual=robot.ee_fk('left',q)@offset
  return np.r_[20*(actual[:3,3]-desired[:3,3]),3*(actual[:3,2]-desired[:3,2]),.0002*(v-q0[:7])]
 seeds=[q0[:7]]+[np.clip(q0[:7]+rng.normal(0,.7,7),robot.limits[:7,0]+.001,robot.limits[:7,1]-.001) for _ in range(12)]
 best=None
 for seed in seeds:
  result=least_squares(residual,seed,bounds=(robot.limits[:7,0]+1e-5,robot.limits[:7,1]-1e-5),max_nfev=180,ftol=1e-9,xtol=1e-9,gtol=1e-9)
  score=np.linalg.norm(residual(result.x)[:6])
  if best is None or score<best[0]:best=(score,result.x)
  if score<.0001:break
 q=q0.copy();q[:7]=best[1];pre=robot.ee_fk('left',q)@offset
 pos_error=float(np.linalg.norm(pre[:3,3]-desired[:3,3]));angle_error=float(np.degrees(np.arccos(np.clip(pre[:3,2]@desired[:3,2],-1,1))))
 if pos_error>.003 or angle_error>2:raise ValueError(f'offline IK failed {pos_error}m/{angle_error}deg')
 solved=q.copy();q[5]+=np.deg2rad(30.)
 if not robot.limits[5,0]<=q[5]<=robot.limits[5,1]:raise ValueError('existing J6 bend outside joint limits')
 fk=robot.fk(q);arms={}
 for side,prefix in [('left','arm_l_joint'),('right','arm_r_joint')]:arms[side]=[fk[robot.joints[prefix+str(i)]['child']][:3,3].tolist() for i in range(1,8)]+[fk[robot.ee[side]][:3,3].tolist()]
 reader=robot.ee_fk('left',q)@offset
 poses.append(dict(extra_yaw_deg=extra,target_reader=desired.tolist(),pre_bend_reader=pre.tolist(),final_reader=reader.tolist(),pre_bend_joints_rad=solved[:7].tolist(),joints_rad=q[:7].tolist(),joints_deg=np.degrees(q[:7]).tolist(),position_error_m=pos_error,axis_error_deg=angle_error,**arms))
d=dict(schema='deux.reader_yaw_example.v1',actual_hardware_executed=False,ik_backend='offline SciPy position + reader Z-axis; different from production PyRoKi',hypothetical_barcode_xyz_m=point.tolist(),hypothetical_barcode=True,existing_j6_bend_deg=30.,poses=poses)
(out/'comparison.json').write_text(json.dumps(d,indent=2))
os.environ.setdefault('MPLCONFIGDIR','/tmp/deux-reader-yaw-plot');import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig=plt.figure(figsize=(15,6));colors=['#788b9b','#e47e2d']
for i,pose in enumerate(poses):
 ax=fig.add_subplot(1,3,i+1,projection='3d');ax.set(xlim=(0,.85),ylim=(-.4,.65),zlim=(.65,1.6),xlabel='base X (m)',ylabel='base Y (m)',zlabel='base Z (m)');ax.set_box_aspect((.85,1.05,.95));ax.view_init(23,-52)
 for side,col in [('left',colors[i]),('right','#bac4cb')]:
  pts=np.array(pose[side]);ax.plot(*pts.T,'o-',color=col,lw=3)
  if side=='left':
   for j,p in enumerate(pts[:7]):ax.text(*(p+np.array([.008,.008,.012])),f'J{j+1}',color='#163950',fontsize=10,weight='bold')
 for key,col,label in [('target_reader','#298e79','Target Z'),('final_reader','#ca4880','After J6 bend')]:
  mat=np.array(pose[key]);o=mat[:3,3];v=mat[:3,2]*.15;ax.quiver(*o,*v,color=col,linewidth=2);ax.text(*(o+v),label,fontsize=8,color=col)
 ax.set_title(['Before: yaw -30 deg','After: yaw -45 deg'][i]+'\nExample IK + unchanged J6 +30 deg')
ax=fig.add_subplot(1,3,3);origin=np.array(poses[0]['target_reader'])[:3,3]
for i,pose in enumerate(poses):
 z=np.array(pose['target_reader'])[:3,2];yaw=np.degrees(np.arctan2(z[1],z[0]));ax.arrow(0,0,z[0]*.22,z[1]*.22,color=colors[i],width=.003,length_includes_head=True,label=f"Target {'before' if i==0 else 'after'} ({yaw:.1f} deg)")
ax.set(xlim=(-.25,.25),ylim=(-.25,.25),xlabel='base X',ylabel='base Y',title='Top view: reader target axis\nSame target origin; extra base yaw -15 deg');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8);fig.suptitle('Offline illustration | hypothetical barcode point | J1-J7 = arm_l_joint1-arm_l_joint7',fontsize=12);fig.tight_layout();fig.savefig(out/'comparison.png',dpi=150)
html='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>바코드 리더 우측 15° · 관절 번호</title><style>body{font:15px system-ui;background:#eef2f6;color:#183043;max-width:1200px;margin:24px auto;padding:0 20px}h1{font-size:25px}p{line-height:1.7}.tag{background:#fff0c9;padding:6px 10px;display:inline-block;border-radius:6px}canvas{background:#fff;border-radius:14px;width:100%;height:560px;touch-action:none}button{padding:9px 16px;border:1px solid #b7c5ce;background:white;border-radius:7px;margin:8px 6px 8px 0}table{border-collapse:collapse;width:100%;background:white;margin:16px 0}td,th{padding:9px;border-bottom:1px solid #dce4eb;text-align:right}td:first-child,th:first-child{text-align:left}.note{color:#856011}img{width:100%}</style><span class="tag">오프라인 예시 IK · 실측 바코드 좌표/실기 자세 아님</span><h1>왼팔 리더: 기존 방향 → 우측으로 15° 추가</h1><p>기존 목표 yaw −30° → 변경 목표 yaw −45° (base_link +Z축 기준). 리더 목표 위치를 유지하고 방향만 회전합니다.<br><b>J1~J7은 arm_l_joint1~arm_l_joint7입니다. J1은 어깨 쪽, J7은 손목 끝 쪽입니다.</b><br>초록 화살표: IK에 전달하는 리더 목표 Z축 / 분홍 화살표: 기존 J6 +30° 굽힘 이후의 예시 리더 Z축.</p><button id="before">기존 −30°</button><button id="after">변경 −45°</button><button id="ghost">두 자세 겹쳐 보기</button><button id="front">정면</button><button id="top">위에서</button><span id="status"></span><canvas></canvas><p class="note">실기 로그에 정렬 종료 관절값이 없어, 오른팔 스캔 파지점 위치를 가상 바코드 지점으로 놓고 SciPy 예시 IK를 구했습니다. 실제 PyRoKi 결과와 관절값이 다를 수 있습니다. 계획 궤적이나 충돌 검증 결과가 아닙니다. 드래그하면 시점이 회전합니다.</p><div id="table"></div><p><b>주의:</b> 추가 15°는 reader-axis IK 목표에 적용됩니다. 이후 기존 코드의 J6 +30° 굽힘이 별도로 적용되므로 최종 리더 방향 차이가 정확히15°라고 보장되지는 않습니다.</p><img src="comparison.png"><script>
const D=__DATA__,cv=document.querySelector('canvas'),ctx=cv.getContext('2d');let which=1,ghost=false,az=-.95,el=.42,W=900,H=560,drag=null;
function proj(p){let x=p[0]-.3,y=p[1]-.05,z=p[2]-1.05,s=Math.min(W/1.4,H/1.2);return[W/2+(-Math.sin(az)*x+Math.cos(az)*y)*s,H/2-(-Math.cos(az)*Math.sin(el)*x-Math.sin(az)*Math.sin(el)*y+Math.cos(el)*z)*s]}
function line(ps,c,w=3,dash=[]){ctx.beginPath();ps.forEach((p,i)=>{let v=proj(p);i?ctx.lineTo(...v):ctx.moveTo(...v)});ctx.strokeStyle=c;ctx.lineWidth=w;ctx.lineJoin='round';ctx.lineCap='round';ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([])}
function label(p,t,c,dx=9,dy=-9){let v=proj(p);ctx.font='bold 14px system-ui';ctx.strokeStyle='white';ctx.lineWidth=4;ctx.strokeText(t,v[0]+dx,v[1]+dy);ctx.fillStyle=c;ctx.fillText(t,v[0]+dx,v[1]+dy)}
function arrow(m,col,name){let o=m.slice(0,3).map(r=>r[3]),v=m.slice(0,3).map(r=>r[2]),end=o.map((x,i)=>x+.17*v[i]);line([o,end],col,4);let a=proj(o),b=proj(end),ang=Math.atan2(b[1]-a[1],b[0]-a[0]);ctx.beginPath();ctx.moveTo(...b);ctx.lineTo(b[0]-12*Math.cos(ang-.45),b[1]-12*Math.sin(ang-.45));ctx.lineTo(b[0]-12*Math.cos(ang+.45),b[1]-12*Math.sin(ang+.45));ctx.closePath();ctx.fillStyle=col;ctx.fill();label(end,name,col)}
function draw(){ctx.clearRect(0,0,W,H);let p=D.poses[which];line(D.poses[0].right,'#c5cdd3',5);let a=p.left[0],b=p.right[0];line([a,b],'#aab7c0',7);line([[0,0,.6],[0,0,(a[2]+b[2])/2]],'#aab7c0',7);
if(ghost)line(D.poses[1-which].left,'#8a98a277',4,[5,5]);line(p.left,which?'#e47e2d':'#788b9b',6);p.left.forEach((v,i)=>{let s=proj(v);ctx.beginPath();ctx.arc(...s,5,0,Math.PI*2);ctx.fillStyle='#23628a';ctx.fill();if(i<7)label(v,'J'+(i+1),'#143f5b',i%2?12:-27,i%2?-12:17)});
arrow(p.target_reader,'#298e79','목표 Z');arrow(p.final_reader,'#ca4880','J6 굽힘 후');let origin=[.1,-.35,.75];for(let [v,n,c]of [[[.12,0,0],'+X','#be5151'],[[0,.12,0],'+Y','#4a8d5b'],[[0,0,.12],'+Z','#477cbd']]){let e=origin.map((x,i)=>x+v[i]);line([origin,e],c,2);label(e,n,c)}document.querySelector('#status').textContent=which?'변경 목표 −45° / 예시 IK':'기존 목표 −30° / 예시 IK';}
let table='<table><tr><th>관절 이름</th><th>기존 예시 (°)</th><th>변경 예시 (°)</th><th>차이 (°)</th></tr>';for(let i=0;i<7;i++){let a=D.poses[0].joints_deg[i],b=D.poses[1].joints_deg[i];table+=`<tr><td>J${i+1} · arm_l_joint${i+1}</td><td>${a.toFixed(2)}</td><td>${b.toFixed(2)}</td><td>${(b-a).toFixed(2)}</td></tr>`}document.querySelector('#table').innerHTML=table+'</table>';
function resize(){W=cv.clientWidth;H=cv.clientHeight;let d=devicePixelRatio||1;cv.width=W*d;cv.height=H*d;ctx.setTransform(d,0,0,d,0,0);draw()}document.querySelector('#before').onclick=()=>{which=0;draw()};document.querySelector('#after').onclick=()=>{which=1;draw()};document.querySelector('#ghost').onclick=()=>{ghost=!ghost;draw()};document.querySelector('#top').onclick=()=>{az=-Math.PI/2;el=Math.PI/2;draw()};document.querySelector('#front').onclick=()=>{az=0;el=.1;draw()};cv.onpointerdown=e=>{drag=[e.clientX,e.clientY,az,el];cv.setPointerCapture(e.pointerId)};cv.onpointermove=e=>{if(drag){az=drag[2]-(e.clientX-drag[0])*.008;el=Math.max(.02,Math.min(Math.PI/2,drag[3]+(e.clientY-drag[1])*.006));draw()}};cv.onpointerup=()=>drag=null;addEventListener('resize',resize);window.previewData=D;resize();
</script></html>'''
(out/'preview.html').write_text(html.replace('__DATA__',json.dumps(d,separators=(',',':'))));print((out/'preview.html').as_uri());print(json.dumps([{k:p[k] for k in ['extra_yaw_deg','position_error_m','axis_error_deg','joints_deg']} for p in poses],indent=2))
