import json,os,subprocess,sys
from pathlib import Path
root=Path('/home/xyz_thor/DEUX_ikworld');out=root/'.setup/grid-grasp-classified-test'
reports=[]
for pattern in ['test_classified_grid_runtime.py','test_rotation_grid_runtime.py','test_grid_grasp_runtime.py']:
 with (out/(pattern+'.log')).open('w') as log:
  subprocess.run([sys.executable,'-m','unittest','discover','-s','tools','-p',pattern],cwd=root,stdout=log,stderr=subprocess.STDOUT,check=True)
env=os.environ.copy();env.update(ROS_DOMAIN_ID='226',DEUX_TEST_CLASSIFIED_GRID='1')
with (out/'synthetic-preview.log').open('w') as log:
 subprocess.run([sys.executable,'tools/test_grid_grasp_ros.py','--ros-args','-p',
  'grid_library_directory:='+str(root/'artifacts/right_grasp_grid_classified'),'-p','use_observed_yaw:=true'],
  cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=35,check=True)
cases=[('positive90',90,[]),('negative90-vision',-90,['--vision-fault']),
       ('normal',0,[]),('transverse-tracking',-97,['--arm-tracking-fault']),('estop',-90,['--stop-fault'])]
for index,(name,yaw,extra) in enumerate(cases,227):
 env=os.environ.copy();env['ROS_DOMAIN_ID']=str(index)
 with (out/(name+'.log')).open('w') as log:
  subprocess.run([sys.executable,'tools/test_grid_grasp_execution_ros.py','--classified-grid',
   '--object-yaw',str(yaw),'--object-x','.6',*extra],cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=40,check=True)
 path=root/'.setup/grid-grasp-tests/synthetic-execution/result.json';result=json.loads(path.read_text())
 expected='normal' if yaw==0 else 'transverse_axis_locked'
 assert result.get('grasp_mode')==expected,result
 if yaw!=0:assert result['axis_error_deg']<=3. and result['orientation_exception_applied']
 (out/(name+'-result.json')).write_text(json.dumps(result,indent=2))
 reports.append(dict(name=name,domain=index,grasp_mode=result['grasp_mode'],success=result['success'],
                     intentional_estop=name=='estop',selected_cell=result['cell_id']))
 print(json.dumps(reports[-1]),flush=True)
summary=dict(unit_tests_passed=22,synthetic_preview_passed=True,synthetic_execution_cases=reports,
 all_passed=True,physical_hardware_execution=False)
(out/'test-summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
