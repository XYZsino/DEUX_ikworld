이동 물체 격자 파지 연동 시험
============================

ROI 진입부터 X를 정렬하는 새 경로는 README.rst의 run_roi_grasp_trial.bash를
사용한다. 이 문서는 준비자세부터 3초 경로를 재생하는 기존 시험 경로 안내다.

구현 범위
---------

installation.yaml / 실측 URDF / 실측 컨베이어 표면을 해시로 확인한 저장 궤적이다.
속도 base_link -Y 0.09m/s, 실제 파지점 Z=0.97m, 전체 회전 허용 30도.
준비자세 기준 3초 파지, 2.5~3초 하강·손 닫힘 병렬, 3~4.5초 상승,
4.5~5초 감속. 손 닫힘 0.5초는 설정값이며 실측 완료 보장은 아니다.

시작 — 명령 없는 실제 인식 연동
-------------------------------

Thor에서::

  cd ~/DEUX_ikworld
  bash run_grid_grasp_test.bash
  source setup_env.bash
  export ROS_DOMAIN_ID=12
  ros2 topic echo /task/grid_grasp/preview
  cat .setup/grid-grasp-test/preview.json

이 진입점은 기존 보정 ZED/TF, tracked_perception, tracking_icp_monitor,
grid_grasp_trial 미리보기를 시작한다. 실행 중인 같은 인식 노드는 재사용하며,
기존 하드웨어 노드를 정지하거나 팔/손 명령을 보내지 않는다.
기존 ROI 파일 .setup/single-track-roi.json 및 카메라 보정을 유지한다.
새 외부 카메라 보정/리프트 높이를 임의 생성하지 않는다.

selectable은 실제 포즈·속도에서 선택 가능한 격자가 있다는 뜻이다.
arm_ready는 양팔이 검증된 준비 스냅샷과 0.03rad 이내라는 뜻이다.
dispatch_ready는 이 두 조건만을 조합한 미리보기이며 하드웨어 실행 허가는 아니다.
실제 피드백이 준비자세와 다르면 파지 서버는 자동 준비동작 없이 실행을 거부한다.
오른팔 준비 이동은 별도 도구로 현재 관절에서 경로를 새로 검사한 뒤 요청한다::

  python tools/prepare_grid_ready.py
  python tools/prepare_grid_ready.py --execute

왼팔은 현재 실측 자세에 고정한다. 위 도구는 그 자세로 권장 파지 궤적25개를
다시 충돌 검사하고 .setup/grid-grasp-test/stationary-left.json에 결과를 저장한다.
원래 저장된 오른팔 준비자세와 파지 궤적을 바꾸지 않는다. 준비 이동에는 손 명령이 없다.

사용자 최신 지시: 궤적 선택은 관측 XY만 사용한다. 물체 모델 중심 Z 및 추정
회전/속도를 선정 조건으로 사용하지 않는다. 원래 모델 중심 Z는 진단값으로 남긴다.
실제 파지 Z는 사용자가 지정한 base_link 0.97m, 물체 방향은 고정 가정,
예측 이동 속도는 사용자가 지정한 (0,-0.09,0)m/s다. 저장 경로 자체의 회전
30도/관절/충돌 검증과 실제 팔의 추종 검사는 유지한다.
취득 타임스탬프의 지연을 고정 벨트 속도로 보정하고, 격자 Y까지의 ETA에서
3초를 뺀 절대 시각에 재생한다. 선정·전송 전 XY 오차30mm 이내, 동일ID의 안정된 관측,
최신 카메라 보정/팔 피드백이 필요하다. 고정Z와 추정 물체 중심은 다른 좌표 원점이다.

실행 가능한 시험 서버 — 서버 시작만으로 동작 없음
------------------------------------------------

실기 이동은 별도의 명시적 Action 요청이 필요하다. 기본 미리보기 서버는
해당 요청을 거부한다. 기존 미리보기 grid 프로세스만 종료한 뒤 운영자가
실행 가능한 시험 서버를 시작할 수 있다::

  python tools/stop_grid_grasp_preview.py
  bash run_grid_grasp_trial.bash --ros-args -p enable_execution:=true \
    -p stationary_left_file:=$PWD/.setup/grid-grasp-test/stationary-left.json

기본 후보는 검증된39개 전체이며 recommended_only:=true로 권장25개만 선택할 수 있다.
전체39개를 현재 고정 왼팔 자세로 검사하는 명령은 다음과 같다::

  python tools/audit_grid_stationary_left.py

기존 /task/run, /task/move_to_ready, /task/execute_motion 서버와 함께 사용하지 않는다.
하드웨어 상태/teleop off, 컨트롤러, 양팔 준비자세, 오른손 mode/profile/RUN,
비상정지, 명령 경합을 검사한다. 이 설정 작업은 그 상태를 변경하지 않는다.
조건 확인 후 운영자가 명시적으로 1회 요청하는 명령은 다음과 같다::

  ros2 action send_goal /task/grid_grasp deux_task_msgs/action/MoveToReady '{}' --feedback

이 문서의 배포/검증 과정에서는 위 실기 Action을 요청하지 않는다.
요청하면 오른손 준비 개폐값을 설정한 후 기본 최대120초 동안 대상/격자를 기다린다.
target_wait_timeout_s 파라미터로 대기 시간을 지정할 수 있다.
팔과 오른손 예약은 같은 실제 ROS 시작 시각을 사용한다. 선정 및 컨트롤러 전송
직전까지 XY30mm 조건을 검사한다. 전송 후에는 셀·시작 시각·팔·손 일정을 고정하고,
추정 XY 변화/대상ID 변경/ROI 이탈/추적 끊김으로 취소하지 않는다.
추정값은 진단·표시에만 사용한다. 비상정지/명시적 Action 취소는 유지한다. 실행 중 상태 수신 오류,
관절 추종 오류, 손 드라이버의 명령 거부는 기록하며 팔을 취소하지 않는다.
제어기 자체 고장이나 드라이버 정지는 이 노드가 무시할 수 없다.
추가 Cartesian 정렬 오차와 손 명령 예약 지연은 로그만 남기며 중단하지 않는다.
중간에 IK를 다시 풀거나 궤적을
임의 평활화하지 않는다. 손 닫힘은 실제 완료 증명이 아니라 명령 시간 기록이다.
일정대로 움직였다는 성공 결과와 물체를 실제로 잡았다는 성공은 구분한다.
한 번 실행한 서버는 다시 시작해야 하며 자동 재시도·복귀·손 개방은 없다.

취소는 Action 취소 또는 기존 비상정지를 사용한다. 취소 후 손을 자동 개방하거나
토크를 끄지 않는다. 제어기 취소 수락은 감속 경로의 물리적 검증을 대체하지 않는다.

검증
----

  python tools/test_fast_grasp_grid.py
  python tools/test_parallel_grid_hand.py
  python tools/test_grid_grasp_runtime.py
  ROS_DOMAIN_ID=217 python tools/test_grid_grasp_ros.py
  ROS_DOMAIN_ID=218 python tools/test_grid_grasp_execution_ros.py
  ROS_DOMAIN_ID=219 python tools/test_grid_grasp_execution_ros.py --vision-fault
  ROS_DOMAIN_ID=220 python tools/test_grid_grasp_execution_ros.py --arm-tracking-fault

217 테스트는 격리 도메인에서 합성 인식/팔 피드백을 발행하고 미리보기 선택,
실행 요청 거부, 팔·손 명령 발행자 0개를 검증한다. 실기 도메인12에서는 거부한다.
218 테스트는 가짜 팔 Action/가짜 손을 사용해 실제 실행 코루틴의 타이밍을 확인한다.
실제 상태/손 모드 검사는 이 가짜 시험에서 대체되므로 실기 검증으로 해석하지 않는다.
219 테스트는 실행 중15cm XY 변화·대상ID 변경·추적 완전 끊김에도 저장 경로 및
병렬 닫힘이 완료됨을 검증한다. 220 테스트는 합성 팔 추종 오류에도 일정을 유지하고 오류를 기록하는지 검증한다.
변경된 관절 오차 시험은 원격 Jazzy의 격리 도메인224에서 재실행하여 통과했다.
새 ROI 경로는 격리 도메인221~223에서 정상/인식 오류/관절 오차 시험을 통과했다.
실제 물체/벨트에서의 인식 연결 증거와 합성 테스트 증거를 따로 기록한다.
전체 로봇 자기충돌/가려진 설비/파지 물체 형상/토크·실제 파지는 미검증이다.

자료
----

* artifacts/right_grasp_grid_fast/manifest.json: 전체 입력 해시와 39개 통과 궤적.
* artifacts/right_grasp_grid_fast/recommended_cells.json: 권장25개 격자.
* .setup/grid-grasp-test/: 프로세스 목록, 실제 미리보기, 로그, 실행 결과.
* ready-result.json: 실제 오른팔 준비 이동 결과. stationary-left.json: 실측 왼팔 고정 검사.
* motion-trace-*.jsonl: 실행 전송 후20Hz 실측/저장 관절·파지점·추정XY·추적상실·손 피드백.
* result.json: 마지막 종료 결과, 로그 파일명, 최대 닫힘 경로 오차, 손 명령 시각/지연 경고.
* 백업은 ~/DEUX_ikworld_backups/ 아래에 전체 사본으로 보존한다.

실제 실행 기록 (2026-11-01)
--------------------------

사용자의 실제 실행 요청 후 오른팔을 33.05초 경로로 저장 준비자세에 이동했다.
Action 성공과 실제 관절 피드백으로 확인했으며 최대 최종 오차는 0.02060rad였다.
왼팔/손 이동 명령 없이 왼팔 실측 고정 상태로 파지 궤적25개 재검증을 통과했다.
이후 오른손 mode3/RUN·실시간 프로파일 확인 후 실제1회 파지 Action을 요청했다.
물체 인식 후 파지·상승 완료 여부는 .setup/grid-grasp-test/result.json과
request-result.json을 확인한다. 준비 이동 성공을 실제 물체 파지 성공으로 해석하지 않는다.
