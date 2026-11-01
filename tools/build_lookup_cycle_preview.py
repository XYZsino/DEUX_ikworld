"""One-session reconstruction of updated paths, explicitly marked as offline.

An optional measured alignment endpoint can replace the declared synthetic example.
No ROS imports, camera extrinsics, or hardware commands.
"""
import argparse,json,sys,hashlib
from pathlib import Path
import numpy as np,yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'src/deux_task_core'),str(ROOT/'src/deux_task_runtime')]
from classified_grid_runtime import ClassifiedGridLibrary
from lookup_lift_scan import compile_path
from lookup_conveyor import validate_conveyor_path
from deux_task_runtime.parallel_placement import build_parallel
from deux_task_core.planning import hermite
from deux_task_core.conveyor_screen import surfaces


def build(aligned_file=None):
 out=ROOT/'artifacts/lookup-cycle-preview';out.mkdir(exist_ok=True)
 source=ROOT/'artifacts/thor-lookup-bt-first-run';record=json.loads((source/'result.json').read_text())
 p=ROOT/'src/deux_task_bringup/config/installation.yaml';c=yaml.safe_load(p.read_text());c['robot']['urdf_path']=str((p.parent/c['robot']['urdf_path']).resolve())
 lib=ClassifiedGridLibrary(c,directory=ROOT/'artifacts/right_grasp_grid_classified',recommended=False,yaw_secondary=True,load_rotation_paths=False);lib.ready[:7]=c['home_joints'][:7];robot=lib.robot
 plan,curve,info=compile_path(lib,record['cell_id'],c);times=np.array(plan['times_s']);q=np.tile(lib.ready,(len(times),1));q[:,14:21]=plan['positions_rad']
 end=q[-1].copy();aligned=end.copy()
 if aligned_file:
  values=json.loads(Path(aligned_file).read_text());values=np.asarray(values,float)
  if values.shape!=(7,) or not np.isfinite(values).all():raise ValueError('alignment endpoint must contain seven finite left joint angles (rad)')
  aligned[:7]=values;alignment_source=str(Path(aligned_file).resolve());synthetic=False
 else:
  example=json.loads((ROOT/'artifacts/parallel-placement-offline/example-plan.json').read_text());aligned[:7]=np.array(example['positions_rad'])[0,:7]
  alignment_source='previous explicitly synthetic example; installation home + [0.15,0.1,0,-0.1,0,0.5,0]rad';synthetic=True
 scale=c['placement'].get('joint_motion_scale',1.)
 at,aq=hermite([end,aligned],robot.velocity*scale,robot.acceleration*scale**2,c['sample_period'])
 align_margin=validate_conveyor_path(robot,aq)
 pp=build_parallel(c,aligned);parallel_margin=validate_conveyor_path(robot,pp['path'])
 stages=[dict(name='접근 · 하강 · 손 닫기',start=0.,end=3.,kind='updated_plan'),dict(name='추종 상승',start=3.,end=4.,kind='updated_plan'),dict(name='우측 경유점 → 스캔',start=4.,end=float(times[-1]),kind='updated_plan')]
 align_start=float(times[-1]);parallel_start=align_start+float(at[-1])
 stages += [dict(name='왼손 정렬 — 예시' if synthetic else '왼손 정렬 — 주어진 종료값',start=align_start,end=parallel_start,kind='synthetic_alignment' if synthetic else 'reconstructed_alignment')]
 release=parallel_start+pp['phase_end_times'][0];opened=parallel_start+pp['phase_end_times'][1];total=parallel_start+pp['times'][-1]
 stages += [dict(name='왼손 후퇴 + 오른손 적재',start=parallel_start,end=float(release),kind='updated_parallel_plan'),dict(name='양팔 정지 · 오른손 열기',start=float(release),end=float(opened),kind='updated_parallel_plan'),dict(name='오른손 후퇴',start=float(opened),end=float(total),kind='updated_parallel_plan')]
 all_t=np.r_[times,align_start+at[1:],parallel_start+pp['times'][1:]];all_q=np.vstack([q,aq[1:],pp['path'][1:]])
 assert np.all(np.diff(all_t)>1e-8)
 np.testing.assert_allclose(aq[0],q[-1],atol=1e-12)
 # build_parallel projects finger joints into command-only closed posture.
 indices=np.r_[np.arange(7),np.arange(14,21)]
 np.testing.assert_allclose(aq[-1,indices],pp['path'][0,indices],atol=1e-12)
 np.testing.assert_allclose(all_q[-1,:7],c['placement']['clear_left_joints'],atol=1e-10)
 left_path=np.array([robot.ee_fk('left',x)[:3,3] for x in pp['path']]);left_distance=float(np.linalg.norm(np.diff(left_path,axis=0),axis=1).sum())
 metadata=dict(type='metadata',schema='deux.updated_cycle_plan_log.v1',actual_hardware_executed=False,
  source_run='259d07b646304644b151ab34be31b1b8',source_log=str((source/'result.json').relative_to(ROOT)),source_sha256=hashlib.sha256((source/'result.json').read_bytes()).hexdigest(),
  source_cell=record['cell_id'],source_target_id=record.get('target_id'),timeline='new planned seconds from ready departure; not historical wall clock',
  actual_joint_replay=False,synthetic_alignment=synthetic,alignment_source=alignment_source,
  missing_historical_data=['left alignment endpoint','left arm measured trajectory after grasp','barcode position during alignment'],
  stages=stages,contact_s=3.,waypoint_s=info['waypoint_at_s'],scan_s=align_start,parallel_start_s=parallel_start,left_retreat_end_s=parallel_start+pp['report']['left_retreat_s'],
  release_s=float(release),open_end_s=float(opened),total_s=float(total),left_retreat_arc_m=left_distance,
  left_alignment_joints_rad=aligned[:7].tolist(),left_retreat_target_joints_rad=c['placement']['clear_left_joints'],parallel_timing=pp['report'],
  minimum_changed_path_conveyor_clearance_m=min(info['minimum_conveyor_clearance_m'],align_margin,parallel_margin),robot_commands=False)
 rows=[]
 for t,x in zip(all_t,all_q):
  stage=next((s for s in stages if s['start']-1e-8<=t<s['end']-1e-8),stages[-1])
  rows.append(dict(type='sample',time_s=float(t),stage=stage['name'],evidence=stage['kind'],joints_rad=x.tolist()))
 log=out/'motion.jsonl';log.write_text(json.dumps(metadata,ensure_ascii=False)+'\n'+''.join(json.dumps(r,ensure_ascii=False,separators=(',',':'))+'\n' for r in rows))
 # All visualization input is read back from this ONE log.
 loaded=[json.loads(line) for line in log.read_text().splitlines()];metadata=loaded[0];samples=loaded[1:];frames=[]
 special=[3.,4.,info['waypoint_at_s'],align_start,parallel_start,float(release),float(opened),float(total)]
 pick=set(range(0,len(samples),3))|{len(samples)-1}|{int(np.argmin(abs(all_t-value))) for value in special}
 for i in sorted(pick):
  r=samples[i];fk=robot.fk(r['joints_rad']);frame=dict(t=r['time_s'],stage=r['stage'],evidence=r['evidence'])
  for side,prefix in [('left','arm_l_joint'),('right','arm_r_joint')]:frame[side]=[fk[robot.joints[prefix+str(j)]['child']][:3,3].tolist() for j in range(1,8)]+[fk[robot.ee[side]][:3,3].tolist()]
  frames.append(frame)
 visual=dict(metadata=metadata,frames=frames,polygons=[v.tolist() for v,_,_ in surfaces()[3]])
 (out/'summary.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2))
 return out,visual


def render(out,d):
 # Reuse the established local, self-contained viewer; replace its content and timeline.
 import ast
 tree=ast.parse((ROOT/'tools/render_parallel_placement.py').read_text());html=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(a,ast.Name) and a.id=='html' for a in n.targets))
 html=html.replace('병렬 후퇴·적재 스켈레톤','전체 BT · 단일 계획 로그').replace('오프라인 FK · 合成 시작 자세 · 실기 미검증','단일 오프라인 계획 로그 · 실기 재생 아님')
 html=html.replace('왼손 후퇴 + 오른손 적재·후퇴</h1>','파지 → 우측 경유점 → 스캔 → 병렬 후퇴·적재</h1>')
 a=html.index('<p>스캔 정렬 후');b=html.index('<canvas',a)
 html=html[:a]+'''<p>실제 첫 BT 기록의 파지 셀을 선택해 새 경로를 한 시간축으로 재구성했습니다.<br><b style="color:#a15c00">왼손 정렬 종료 관절값은 원본 로그에 없어 예시값을 사용했습니다. 왼손 후퇴 길이는 실측값이 아닙니다.</b><br>아래 모든 자세와 시간축은 <a href="motion.jsonl">motion.jsonl 한 파일</a>에서 생성했습니다.</p>'''+html[b:]
 html=html.replace('const phases=D.phases,leftEnd=D.report.left_retreat_s;', 'const M=D.metadata,phases=[M.release_s,M.open_end_s,M.total_s],leftEnd=M.left_retreat_end_s;')
 a=html.index("for(let [label,end,color]of");b=html.index('function project',a)
 html=html[:a]+'''for(let s of M.stages){let color=s.kind==='synthetic_alignment'?'#c79440':s.kind==='updated_parallel_plan'?'#428ac0':'#e4812f';document.querySelector('#timelines').innerHTML+=`<div class="row"><span>${s.name}</span><div class="bar"><div class="fill" style="left:${s.start/total*100}%;width:${(s.end-s.start)/total*100}%;background:${color}"></div><div class="cursor"></div></div></div>`}
document.querySelector('#stats').innerHTML=`<b>출발 기준</b> · 접촉 ${M.contact_s.toFixed(2)}s · 경유점 ${M.waypoint_s.toFixed(2)}s · 스캔 ${M.scan_s.toFixed(2)}s · 병렬 시작 ${M.parallel_start_s.toFixed(2)}s · 완료 ${total.toFixed(2)}s<br>예시 왼손 후퇴 궤적 길이 ${(M.left_retreat_arc_m*100).toFixed(1)}cm · 이 값은 실제 로그로 확인되지 않았습니다.<br>원본 회차 ${M.source_run} / 셀 ${M.source_cell}`;
''' + html[b:]
 a=html.index("document.querySelector('#stage').textContent=");b=html.index(';}',a)
 html=html[:a]+"document.querySelector('#stage').textContent=(M.stages.find(s=>t>=s.start&&t<s.end)||M.stages.at(-1)).name"+html[b:]
 html=html.replace('grid-template-columns:130px 1fr','grid-template-columns:230px 1fr')
 html=html.replace("window.seekTo=s=>", "window.previewData=D;window.seekTo=s=>")
 # Per-stage jump buttons avoid scrubbing through the complete cycle.
 html=html.replace('<div id="timelines">','<div id="jumps" class="controls"></div><div id="timelines">')
 html=html.replace('function project(p)',"for(let s of M.stages){let b=document.createElement('button');b.textContent=s.name;b.onclick=()=>window.seekTo(s.start);document.querySelector('#jumps').appendChild(b)}\nfunction project(p)")
 (out/'preview.html').write_text(html.replace('__DATA__',json.dumps(d,ensure_ascii=False,separators=(',',':'))))
 print((out/'preview.html').as_uri());print(json.dumps(d['metadata'],ensure_ascii=False,indent=2))

if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('--aligned-joints-json');args=parser.parse_args();out,d=build(args.aligned_joints_json);render(out,d)
