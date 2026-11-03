lookup 파지 기반 전체 BT
=======================

한 번에 실행
------------

실제 하드웨어 bringup 및 아래의 최초 빌드/설치 설정을 마친 Thor에서::

  cd /home/xyz_thor/DEUX_ikworld
  bash run_lookup_stack.bash

이 명령은 gravity/effort → 상태·손 모드 → 외부 ZED → 추적 인식 →
lookup BT 서버 → 반복 RunTask 한 번 요청까지 진행한다.
실제 준비 복귀·파지·스캔·적재 반복을 시작하므로 물체 투입 전 준비를 마친다.
기본 scan_timing_profile은 via_preserved_fast_scan이다. 오른팔 파지→경유점은
기존 궤적/시각을 유지하고 경유점→스캔만1.8초로 단축한다. 대표 경로는
접촉3초/경유점6초를 유지하며 스캔 도착8초→7.8초다. 이전6.8초 후보는 적용하지 않는다.
왼팔은 경유점 도착까지 준비자세에서 기다린 뒤 함께 스캔으로 이동한다.
전체2,177개 중2,175개가 모델 검사를 통과했고 기존 연결 불가2개는 제외한다.
이미 실행 중인 노드는 유지하고, 기존 lookup 작업이 active면 아무 변경 없이 종료한다.
하드웨어 bringup/빌드/전체 궤적 재검사는 수행하지 않는다.
누락된 설치 파일이나 실패가 있으면 해당 단계에서 종료하고, 기존/시작된 서비스는
남겨둔다. 실패한 RunTask를 자동 재전송하거나 동작 중 프로세스를 종료하지 않는다.

로봇 작업 요청 없이 서버까지만 실행하거나, 실행 계획만 확인할 때::

  bash run_lookup_stack.bash --servers-only
  bash run_lookup_stack.bash --dry-run

``--servers-only``도 effort 활성화와 손 모드 설정은 수행한다.
``--dry-run``은 ROS 접속·프로세스 실행·로봇 명령 없이 실행 목록만 출력한다.
구현은 ``tools/start_lookup_stack.py``다. 시작 단계별 로그/PID는
``.setup/lookup-bt/live-start/`` 아래에 기록한다.
아래 개별 실행 명령은 문제 진단이나 단계별 실행이 필요할 때 사용한다.

GUI까지 한 번에 실행
--------------------

위 통합 실행에 웹 GUI까지 포함하려면 Thor에서::

  cd /home/xyz_thor/DEUX_ikworld
  bash run_lookup_stack_gui.bash

이는 ``bash run_lookup_stack.bash --gui``와 같다. 기본 동작은 실제 반복 실행이며,
기존 반복 BT가 active일 때는 작업을 재요청하지 않고 누락된 GUI만 시작한다.
GUI가 이미 실행 중이면 재사용한다. GUI는 Thor의127.0.0.1:8080에 바인딩한다.
GUI를 포함하되 새 작업 요청 없이 서버만 준비하거나, 실행 계획만 확인할 때::

  bash run_lookup_stack_gui.bash --servers-only
  bash run_lookup_stack_gui.bash --dry-run

노트북의 별도 터미널에서 SSH 터널을 연다::

  ssh -N -L 18080:127.0.0.1:8080 xyz_thor@192.168.1.10

브라우저 접속 주소는 ``http://127.0.0.1:18080/``다.
새 GUI의 인증 토큰 포함 URL은 Thor의
``.setup/lookup-bt/live-start/gui.log``에 ``GUI:``로 출력된다.
그 URL의 포트를8080에서18080으로 바꾸고 ``#token=...`` 부분을 유지해 접속한다.
기존 GUI를 재사용하면 기존 실행 로그의 토큰을 사용한다.
GUI는 기존 상태/카메라를 표시하며 추가 파지 Action을 자동 제출하지 않는다.
이 스크립트의 문법/모의 실행을 확인했으며 전체 실제 기동 시험은 아직 하지 않았다.

Thor 실행 명령: ZED / effort / 반복 BT
------------------------------------

Thor 접속 후 아래 명령을 각각 별도 터미널에서 실행한다. ROS 2 Jazzy,
ROS_DOMAIN_ID=12와 기존 실제 하드웨어 bringup을 사용한다.
이미 실행 중인 ZED/인식/gravity/BT는 중복 실행하지 않는다.
이 절은 실제 사용한 실행 명령을 정리한 것이며 문서 수정 시 로봇을 재시작하지 않는다.

1. Effort 컨트롤러 활성화 (하드웨어 bringup과 기존 gravity 실행 후)::

     cd /home/xyz_thor/DEUX_ikworld
     bash enable_arm_effort.bash status
     bash enable_arm_effort.bash

   이 스크립트는 ``/arm/controller_manager``의
   ``deux_arm_effort_controller``만 활성화하며, 이미 active면 유지한다.
   중력보상 노드 자체는 실행하지 않는다. 현재 Thor에서 사용하는 별도
   gravity 실행 명령은 아래와 같다. 이미 실행 중이면 다시 실행하지 않는다::

     source /opt/ros/jazzy/setup.bash
     source /home/xyz_thor/deux_leader/install/setup.bash
     export ROS_DOMAIN_ID=12
     ros2 launch deux_gravity deux_gravity.launch.py

2. 외부 ZED와 설치 보정 TF::

     cd /home/xyz_thor/DEUX_ikworld
     bash run_calibrated_zed.bash

   ``calibrated_zed.thor.launch.py``를 실행한다. 기존 헤드 카메라와 별도다.

3. 추적/포즈 인식 (기존 인식 노드가 없을 때만 별도 터미널에서 실행)::

     cd /home/xyz_thor/DEUX_ikworld
     source setup_env.bash
     source install_lookup_bt/local_setup.bash
     export ROS_DOMAIN_ID=12
     ros2 run deux_task_perception tracked_perception_node --ros-args \
       -r __node:=perception_node \
       --params-file /home/xyz_thor/DEUX_ikworld/tracked_perception.thor.yaml \
       -p config_file:=/home/xyz_thor/DEUX_ikworld/src/deux_task_bringup/config/installation.yaml

4. 실제 BT 서버를 반복 모드로 시작::

     cd /home/xyz_thor/DEUX_ikworld
     bash run_lookup_bt.bash enable_execution:=true repeat_execution:=true

5. 별도 터미널에서 반복 작업 요청 (한 번만 실행)::

     cd /home/xyz_thor/DEUX_ikworld
     source setup_env.bash
     source install_lookup_bt/local_setup.bash
     export ROS_DOMAIN_ID=12
     python tools/request_lookup_repeat.py

   4번은 서버 실행이며, 실제 반복 동작은5번 요청이 승인된 뒤 시작한다.
   준비 복귀 후 ``LookupAcquire``에서 봉투를 기다린다.
   서버 빌드/설치 및 준비 프로파일 생성은 아래 ``빌드와 실행`` 절을 따른다.

반복 상태 확인 및 현재 회차를 마친 뒤 반복 해제::

  cat /home/xyz_thor/DEUX_ikworld/.setup/lookup-bt/repeat-status.json
  ros2 service call /task/repeat_execution std_srvs/srv/SetBool '{data: false}'

현재 백그라운드 실행의 PID/로그 위치는
``.setup/lookup-bt/live-start/processes.json``에 있고,
반복 요청 프로세스 정보는 같은 폴더의 ``repeat-process.json``에 있다.
세션에 사용한 ``/tmp/deux_start_requested.bash``와
``/tmp/deux_restart_bt_fast.bash``는 임시 실행 도우미다.
재현할 때는 위 저장소 스크립트/명령을 사용한다.

구현된 경로
-----------

LookupPrepare → LookupAcquire → 저장 궤적의 접근/하강/병렬 손 닫힘/상승/정지
→ ScanPose → VerifyVisualPick → TrackBarcode/AlignBarcode/WaitBarcode
→ ClearReader → 기존 적재/손 열기/후퇴.

``tools/lookup_bt_node.py``가 기존 HeadTaskNode의 후속 플래너/실행기를 재사용한다.
파지 전에는 ICP/예측 XY와 yaw로 일반/횡방향 경로를 선택한다. 파지 Z=0.97m,
접촉3초, 손 닫힘2.5~3초, 상승4.5초, 정지5초는 기존 저장 경로 그대로다.
전송 후 인식 소실/관측 오차는 기록만 한다. 명시적 취소·비상정지는 유지한다.
이 정책은 저장 파지 경로에 적용되며, 이후 기존 스캔/적재 실행기의 고장 검사는 유지한다.

``/task/lookup_grasp``는 새 LookupGrasp Action이다. request_id/target_id/실행 완료를
검사한 뒤 실제 측정 관절에서 ScanPose를 계획한다. 기존 ``/task/grid_grasp``
서버와 동시에 실행하지 않는다. ``/task/lookup_prepare``는 현재 측정 관절에서
기존 준비 경로를 재계산해 오른팔과 installation.yaml의 왼팔 기본자세를 복원한다.
양팔 준비/파지는 같은 Action 잠금을 공유한다. 준비 동작은 손에 명령하지 않는다. 왼팔은 YAML의 기존 final_tolerance(현재6°),
오른팔은 저장 궤적 시작 오차0.03rad 기준을 적용한다.

인식 제어권은 ``/task/lookup_perception_hold`` SetBool로 넘긴다. false일 때
lookup_perception_bridge가 detect/locate 및 이동 예측을 공급하며, true 응답 뒤에는
BT만 barcode 요청을 발행한다. 종료/실패/취소 시 false로 복귀한다.
기존 barcode 모드는 봉투가 ROI를 벗어나도 실제 검출된 단일 바코드의 깊이 중심을
사용한다. 이 값은 바코드 위치 증거이며 봉투 ID의 재확인이나 물리 파지 증명이 아니다.

현재 설치 설정
--------------

installation.yaml과 경로 입력 해시는 변경하지 않았다. 현재 설정은
``skip_visual_pick_verification: true``, ``skip_barcode_completion: true``다.
따라서 바코드의 실제 관측 위치로 정렬하되 판독 완료는 기다리지 않는다.
적재는 기존 임시 시연 설정과 ``skip_fixed_motion_collision_checks``를 그대로 따른다.
BT 성공은 설정된 동작 흐름 완료이며 실제 파지/바코드 판독/적재 성공을 뜻하지 않는다.
``continuous_motion: true``에서는 기존 실행기가 PlaceApproach 안에서 이동·손 열기·
후퇴를 연속 실행하므로 별도의 Release/Retreat Action을 추가하지 않는다.
다음 회차는 항상 양팔 준비 복귀부터 시작한다. 반복 모드에서는 하나의 RunTask 안에서
회차를 반복하며 새 Action 요청을 자동 생성하지 않는다.

빌드와 실행
-----------

ROS 2 Jazzy 및 기존 Thor overlay/.venv를 사용한다. 새 인터페이스만 별도 overlay에
빌드하므로 기존 메시지 필드를 변경하지 않는다::

  ./build_lookup_bt.bash
  ./run_lookup_bt.bash

runtime/GUI만 다시 빌드할 때에도 가상환경 Python으로 colcon을 실행한다::

  source setup_env.bash
  python -m colcon build --base-paths src --packages-select deux_task_runtime deux_task_gui

시스템 colcon으로 빌드하면 실행 파일의 Python이 /usr/bin/python3으로 바뀌어
가상환경에 설치된 jax를 찾지 못할 수 있다. motion_planner_node 실행 파일의
첫 줄은 이 저장소 .venv/bin/python을 가리켜야 한다.

두 번째 명령은 기본적으로 팔/손 실행이 금지된 미리보기다. 카메라·실제 보정 TF·
추적 인식·드라이버는 기존 프로세스를 사용한다. state_observation_node가 없으면
추가한다. 기존 task_node/head_task_node, motion_planner_node, execution_node,
tracking_icp_monitor, grid_grasp_trial과 중복되면 실행을 거부한다.
진행 중 Action을 먼저 확인하고, 유휴 상태의 기존 작업 노드만 종료한 뒤 시작한다.
새 launcher는 기존 프로세스를 강제 종료하거나 로봇을 자동 이동시키지 않는다.

설치 왼팔 기본자세로 컨베이어를 검사한 파일
``.setup/lookup-bt/home-left-conveyor.json``이 필요하다.
검증 파일이 없거나 라이브러리 해시가 다르면 시작하지 않는다.
``python tools/build_lookup_home_left.py``로 생성한다. 전체 lookup 경로의 컨베이어 검사 결과를 포함한다.
사용자 지시에 따라 준비 전환과 lookup 프로파일은 양팔 간/자체/기타 장면 상자 충돌 검사를 제외한다.
준비 경로는 양팔 각각을 앞쪽 실측 컨베이어 면에 대해 연속 검사하며 관절/속도/가속도 제한을 유지한다.

실제 실행을 명시적으로 켠 경우에도 Action 요청 전에는 이동하지 않는다::

  ./run_lookup_bt.bash enable_execution:=true

다른 터미널에서 동일한 ROS_DOMAIN_ID(기본12)를 사용한다::

  source setup_env.bash
  source install_lookup_bt/local_setup.bash
  export ROS_DOMAIN_ID=12
  ros2 action send_goal /task/move_to_ready deux_task_msgs/action/MoveToReady '{}' --feedback
  ros2 action send_goal /task/run deux_task_msgs/action/RunTask '{target_id: ""}' --feedback

기본 모드에서 RunTask 한 요청은 파지부터 후퇴까지 한 회차다. 반복 모드는 아래를 따른다. 준비가 완료된 뒤 ROI 상류에 물체를
투입한다. 빈 target_id는 lookup에서 선택된 실제 추적 ID를 사용한다.
``/task/status``와 RunTask feedback으로 진행을 확인한다.
``.setup/lookup-bt/``에 회차별 BT/파지 결과와 준비 경로, motion-trace를 남긴다.

검증과 한계
------------

차렷 출발 준비 경로 (Thor 적용)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

오른팔 각 관절이 차렷 기준 0rad에서 0.30rad 이내이면
``calibration/attention_ready_route.json``의 뒤로 접기 → 몸 가까이에서
올리기 → 기존 오른팔 준비자세 경로를 반드시 거친다. 왼팔은 기존 관절
직선 경로를 같은 시간표로 진행해 양팔이 동시에 출발·도착한다.
손 개폐 명령은 보내지 않는다. 다른 자세에서의 준비 복귀는
기존 경로 계산을 사용한다. 새 경로의 모델 해시/목표가 다르거나 연속
컨베이어 검사에 실패하면 넓게 우회하는 경로로 자동 대체하지 않는다.

``tools/build_compact_ready.py``는 경유점 재생성 도구이며,
``tools/render_compact_ready.py``는 스켈레톤 GIF/PNG와 비교 지표를
``artifacts/simultaneous-attention-ready/``에 왕복 동작을 저장한다. 기준 차렷에서 오른손
XY 경로 길이는 1.646m에서 0.416m, 좌우 폭은 0.608m에서 0.040m로 줄었다.
이는 전역 최적해 보장이 아닌 기존 경로 대비 오프라인 모델 비교다.
0.45rad/s·0.9rad/s² 설정에서 양팔 동시 이동은 약11.18초다.
2026-11-03 동시 준비/역순 복귀 및 스캔 단축 변경을 Thor에 배포했다.
단위 검사38개 및 격리 ROS_DOMAIN_ID의 가짜 제어기에서 준비/역순 복귀
통신 시험이 통과했다. 양방향11.178754481초이며 손 명령은 보내지 않았다.
가짜 제어기 스캔 재생도 통과했다(접촉3초, 경유점6초, 스캔7.8초;
손 닫기2.5~3초 유지). 기록은 artifacts/scan-via-preserved/validation/에 있다.
실제 하드웨어 동작 검증은 수행하지 않았다.
기존 컨베이어 전용 충돌 검사 범위는 유지하며,
자체·양팔 간 충돌을 검증했다는 의미가 아니다.

종료와 차렷 복귀
~~~~~~~~~~~~~~~~

``bash stop_lookup_bt.bash``는 반복을 해제하고 진행 중인 Action의 취소·종료를
확인한 뒤 ``/task/return_to_attention``을 한 번 요청한다. 현재 자세가
준비자세와 다르면 기존 준비 복귀 경로를 먼저 수행하고, 양팔 동시 준비
경로의 정확한 역순으로 차렷(팔14관절 0rad)에 복귀한다. 이미 차렷이면
새 궤적을 보내지 않는다. 손가락 명령은 유지한다.

복귀 전 정지, 전체 경로의 관절/속도/가속도 및 컨베이어 간격,
실행 중 피드백, 최종 관절오차0.03rad와 정지를 검사한다. 복귀가 성공한
경우에만 BT launcher를 종료하며 카메라·인식·GUI·드라이버는 유지한다.
보정 무효화, 복귀 거부/실패/시간 초과 시에는 서버를 유지하고 오류를 보고한다.
차렷 복귀 요청 후에는 새 작업/준비 이동/반복 재개를 거부한다. 복귀 재시도는
가능하며 새 작업은 BT 재시작 후 요청한다.
팔을 움직이지 않고 취소·종료만 하려면 ``bash stop_lookup_bt.bash --no-stow``를
사용한다. ``--dry-run``은 대상 확인만 수행하며 명령을 보내지 않는다.

``--return-timeout`` 기본값은120초이며 회차 취소와 프로세스 종료의
``--timeout`` 기본값은30초다. 실행 중인 BT가 이미 없으면 아무 동작도
하지 않는다. 새 동작에는 갱신된 lookup 서버/코디네이터 및 runtime 패키지가
필요하므로 Thor 연결 후 함께 배포·빌드하고 유휴 상태에서 재시작해야 한다.

기존 검증 기록
~~~~~~~~~~~~~~

* 기존 오프라인 선택/타이밍/손 스케줄 회귀34개 통과.
* 새 오프라인4개: 결과 상관관계/거부 조건, 준비 상태 검사, 적재 후 왼팔 복원, 일반/±90° 저장 경로
  종료 관절에서 ScanPose 및 준비 복귀를 실제 URDF로 확인. 고정 관절 ScanPose
  시험은 IK가 필요 없어 SciPy backend를 사용했다. 라이브 설치 backend는 변경하지 않았다.
* Thor /tmp의 별도 overlay에서 새 메시지 Jazzy 빌드 성공.
* 도메인211의 합성 BT 시험: 두 각도 분류 연속 회차, 인식 제어권, 잘못된 파지 결과,
  스캔/바코드 실패 후 적재 금지, 바코드 판독/3회 제한 재시도, 명시적 취소 전파 확인. 후속 단계는 가짜 단계다.
* 도메인212/213에서 실제 새 파지 Action + 가짜 팔/손 제어기로 일반 경로의
  인식 소실과 +90° 전용 경로의 완료/병렬 손 닫힘을 확인. 도메인214에서는
  −90° 경로 중 비상정지가 가짜 제어기까지 전달되는 것을 확인.

시험 요약과 검증한 소스 해시는 ``artifacts/lookup-bt-validation/summary.json``에 있다.
Jazzy 빌드·합성 시험은 /tmp 격리 폴더에서 수행한 뒤, 변경 파일을 백업하고
Thor 운영 폴더에 적용했다. 별도 overlay 빌드만 수행하며 기존 노드는 재시작하지 않았다.
백업: /home/xyz_thor/DEUX_ikworld_backups/20261101-132747-before-lookup-full-bt

2026-11-01 전체 BT 첫 실기를 수행했다. 일반 각도 경로에서 스캔·리더 정렬·
연속 적재/손 열기/후퇴까지 제어기 성공으로 종료됐다. 바코드 판독 완료는
기존 설정대로 생략했다. 실제 파지/적재 성공은 미확인이다.
기록: ``artifacts/thor-lookup-bt-first-run/summary.json``.
합성 단계 시험은 실제 후속 플래너/제어기 또는 물리 파지 성공의 검증을 대신하지 않는다.

연속 반복 모드
--------------

서버의 기본 모드는 1회 실행이다. 반복 서버는 다음과 같이 시작한다::

  ./run_lookup_bt.bash enable_execution:=true repeat_execution:=true

``python tools/request_lookup_repeat.py``는 반복을 활성화한 뒤 RunTask를 딱 한 번
요청한다. 하나의 Action이 계속 활성 상태를 유지하며 성공한 회차 뒤에
양팔 준비 복귀 → 새로운 대상 대기 → 파지/스캔/정렬/적재/후퇴를 반복한다.
새 Action을 자동 재요청하지 않는다. 매 회차 별도 request_id와 BT/파지 로그를 남긴다.
이미 처리 완료한 추적 ID는 파지 실행기 프로세스 내에서 다시 선택하지 않는다.
특정 target_id를 지정한 요청에서는 반복을 허용하지 않는다.

물체가 없으면 준비자세에서 명시적 취소까지 기다린다. 파지 대상 대기 제한시간은
경고 주기이며 종료 조건이 아니다. 물체는 준비 완료 후 ROI 상류에 순차 투입한다.
상태와 완료 횟수는 ``/task/repeat_status`` 및
``.setup/lookup-bt/repeat-status.json``에서 확인한다.
시험용 ``repeat_max_cycles``는 0이면 무제한, 양수이면 해당 완료 횟수에서 종료한다.

현재 회차까지 끝낸 뒤 반복 해제::

  ros2 service call /task/repeat_execution std_srvs/srv/SetBool '{data: false}'

물체 대기를 포함해 즉시 Action 취소::

  ros2 service call /task/run/_action/cancel_goal action_msgs/srv/CancelGoal '{}'

명시적 취소/비상정지는 하위 동작에 전달한다. 바코드 정렬의 계획 실패와 판독 3회 시간 초과는 실패 회차로 기록한 뒤,
기존 고정 적재 경로에서 손 열기·후퇴를 수행하고 다음 회차 준비/물체 대기로 넘어간다.
회차를 성공으로 세지 않으며 failed/recovered를 따로 기록한다.
제어기 실패·복귀 실패·기타 계획 실패는 반복을 종료하며 자동 재시도하지 않는다. 물리적 파지 확인 및 바코드 판독 생략 설정은
그대로 유지되므로 동작 완료 횟수가 실제 물체 처리 성공 개수는 아니다.


이미 실패해 정지한 파지 회차의 재개
---------------------------------

``/task/recover_before_next`` SetBool을 true로 설정하면 다음 RunTask는
기존 고정 적재 경로로 손에 남았을 수 있는 물체를 내려놓고 후퇴한 뒤 정상 회차를 시작한다.
이 설정은 유휴 상태에서만 받으며 한 번 사용하면 해제한다. 이번 정렬 IK 실패처럼
스캔자세 이동까지 완료된 상태를 확인한 뒤 사용하는 재개 절차다. 제어기 고장이나
임의 자세에서 이 서비스를 자동 설정하지 않는다::

  ros2 service call /task/recover_before_next std_srvs/srv/SetBool '{data: true}'
  python tools/request_lookup_repeat.py

회복 중에는 그립을 유지하고, 기존 적재 경로의 정지 구간에서만 손을 연다.
회복 로그는 ``recovery-*.json``, 정상/실패 회차는 ``bt-*.json``에 남긴다.

스캔 이동 속도와 일시적인 손 피드백 지연
----------------------------------------

lookup의 접촉3초/상승4.5초/정지5초 시각은 유지한다. 정지 후 실측 관절에서
ScanPose를 계획하고, 모든 경유점을 유지한 채 ``scan_speed_scale``로 시간만 바꾼다.
기본값0.5에서는 YAML의0.35보다 이동 시간이0.7배다. 최근 경로는5.48초→3.836초다.
관절 위치·속도·가속도 제한을 다시 검사한다. ``scan_speed_scale:=0.5``로 지정할 수
있으며 YAML/저장 궤적 입력 해시는 바꾸지 않는다. 준비자세를 경유하지 않고,
제어기 전송 여유0.5초는 유지한다. ``scan_timing``에 실제 계획 시간을 기록한다.

ExecutionNode의 경로 검사는 약20ms마다 실행을 양보해 피드백 콜백을 처리한다.
200ms 신선도 기준은 완화하지 않는다. 오류에는 좌/우와 샘플 나이를 기록한다.
ScanPose/AlignBarcode의 일시적인 손 피드백 부족은 ``WaitHandFeedback``으로 넘어가며,
읽기 전용 ``/task/execution_health``가 팔의 미완료 Action 없음, 팔/양손 피드백,
손 모드/매핑이 정상인 상태를1초 연속 확인하기 전에는 움직이지 않는다.
정상 복귀 후 실측 관절에서 스캔자세 → 기존 적재 경로의 손 열기·후퇴 → 준비 복귀와
다음 물체 대기를 수행한다. 실제 단절이 지속되면 움직이지 않고 기다린다.
명시적 취소/비상정지는 유지하며, 복귀·스캔 경로는 앞쪽 실측 컨베이어도 추가 검사한다.

왼팔 리더 후퇴와 오른팔 적재의 동시 진행
--------------------------------------

``parallel_reader_retreat:=true``가 lookup BT의 새 기본값이다. 기존
ClearReader → PlaceApproach 두 동작 대신, ``ParallelPlace`` 한 단계에서
왼팔 후퇴와 기존 오른팔 적재 경로를 결합한다. 실행기로 보내는 MotionPlan은
기존 PlaceApproach이며 양팔14개 관절을 하나의 JTC Action, 같은 시작 시각으로 실행한다.
서로 다른 두 Action을 병렬 발행하지 않는다.

오른팔은 기존 접근·하강·정지 손 열기·후퇴 경로를 유지한다. 왼팔은 실측 정렬 자세에서
clear_left_joints까지 시작/종료 속도와 가속도가0인5차 곡선으로 이동한다.
일반적으로 오른팔 적재 이동의65% 시점까지 왼팔 후퇴를 끝내며, 모델 제한 때문에
더 오래 필요하면 오른팔 접근을 늦춘다. 손 열기0.5초와 이후 오른팔 후퇴 시간은 유지한다.
별도 ClearReader 완료 대기와 그 뒤의 새 팔 Action 전송 여유가 사라진다.

검사는 사용자 지정대로 앞쪽 실측 컨베이어를 대상으로 수행하며, 관절 위치·속도·
가속도 한계도 검사한다. 손 열기 전에 왼팔 후퇴 완료와 양팔 정지 구간을 계획에서
검사하고, 실행 중에도 기존 Release 위치/왼팔 후퇴 검사를 유지한다.
기존 실행기는 기본적으로 초기 왼팔 후퇴 검사를 유지하며, lookup launcher가
parallel_reader_retreat를 켠 경우에만 검증된 결합 경로를 허용한다.
기능을 끄려면 동일 launcher에 ``parallel_reader_retreat:=false``를 지정한다.

기존 실패 복구의 적재·손 열기·후퇴에도 같은 결합 경로를 사용한다. 오른손은 컨베이어
위에서 미리 열지 않는다. 파지·스캔 속도와 원본 installation.yaml/lookup 파일은 변경하지 않았다.
``parallel_timing`` 및 ``parallel-place-*.json``에 결합 시간표와 경로를 기록한다.

오프라인 검증/적용 준비
~~~~~~~~~~~~~~~~~~~~~~

``tools/test_parallel_placement_offline.py``는 실제 URDF/설치 설정을 사용하는
7개 테스트다. 양팔 동시 출발, 기존 오른팔 경로 보존, 늦은 왼팔에 따른 시간 조정,
개방 전 정지/후퇴 검사, 컨베이어 검사와 실행기 승인 조건을 확인한다.
실행기 승인 조건은 ROS 의존성을 제외해 실제 validate 메서드를 실행한 테스트이며,
ROS 메시지 전송 시험은 아니다. 기존39개와 합계46개 테스트가 통과했다.

현재 노트북에는 ROS 2 Jazzy 실행 환경, Docker, tools/validate_jazzy.sh가 없어
새 기능의 Jazzy 통신 시험은 아직 수행하지 않았다. Thor 전원 OFF 상태로 적용과 실물
실행도 하지 않았다. 전원 복구 후 새 Python 모듈을 포함해 runtime 패키지를 빌드하고,
격리 도메인에서 가짜 제어기 시험을 먼저 수행한다::

  source setup_env.bash
  source install_lookup_bt/local_setup.bash
  python -m colcon build --base-paths src --packages-select deux_task_runtime --symlink-install
  ROS_DOMAIN_ID=214 python tools/test_parallel_placement_ros.py
  ROS_DOMAIN_ID=215 python tools/test_parallel_placement_ros.py --cancel

이 두 Jazzy smoke test는 가짜 팔/손만 사용하며, 아직 실행 전이다.
모델 예제와 시간표 그래프는 ``artifacts/parallel-placement-offline/``에 있다.
예제의 왼팔 시작값은 설치 기본값에 작은 관절 오프셋을 준 합성 값이며, 실측 정렬
관절값을 대신한 오프라인 검토용이다. 실물에서 자연스러운 동작 여부는 확인 전이다.

우측 경유점과 연속 스캔 연결 (오프라인 적용)
-------------------------------------------

``tools/lookup_lift_scan.py``는 기존 파지 라이브러리의 0~4초를 그대로
유지하고 그 뒤를 선택한 우측 경유점과 스캔자세까지 연결한다. 접촉은 기존과
동일한 3초이며 손 닫기 일정도 변경하지 않는다. 경유점의 실제 FK 좌표는
base_link (0.660011, -0.259993, 1.234997)m로, 스캔보다 약3.3cm 낮다.
기준 셀에서는 준비자세 출발 후6초에 경유점을 정지 없이 통과하고8초에
스캔자세에서 정지한다. 접촉 후 소요시간은5초다.

각 파지 셀의 연결은 시작 속도·가속도까지 이어지는 C2 quintic으로 생성한다.
추가 운반 구간은 속도80%, 가속도64% 상한이며 필요할 때 이 구간만 늘린다.
허용 연결 시간은4~8초다. 관절 한계와 실측 컨베이어를 검사하며 통과하지
못한 셀은 일반/90도 선택 후보에서 제외한다. 상호 팔 충돌 검사는 추가하지
않는다. 원본 라이브러리와 installation.yaml은 변경하지 않는다.

리프트·컨베이어 설치 높이가 저장된 기준과 같으면 전체 셀 재검사를 생략한다.
기존 오프라인 통과 후보를 복원해 사용하며 후보 선택 시 추가 검사도 없다.
현재 자세에서 준비 복귀 경로를 만드는 단계와 하드웨어 상태 확인은 유지한다.
새 LookupGrasp 결과의 ``lift_scan_completed`` 완료 표식이 있을 때 BT는
실측 오른팔이 스캔 관절값에 도착했는지 확인하고 중복 ScanPose 이동을 생략한다.
이 표식이 없는 이전 서버 결과와 복구 동작에는 기존 ScanPose를 유지한다.

전체 모델 검사::

  PYTHONPATH=tools:src/deux_task_core /usr/bin/python3 tools/check_lookup_lift_scan.py
  PYTHONPATH=tools:src/deux_task_core /usr/bin/python3 -m unittest tools/test_lookup_lift_scan.py

검증 결과는 ``artifacts/right-high-scan-waypoint/library-validation.json``에 저장한다.
새 연결 및 병렬 모션은 아직 Thor 배포/실기 확인 전이다. 빵 봉투의 실제 부피와
이동하는 다른 봉투와의 충돌 회피는 이 모델 검사로 확인하지 않는다.

병렬 후퇴·적재 스켈레톤은 ``tools/render_parallel_placement.py``로 생성한다.
``artifacts/parallel-placement-offline/preview.html``에서 회전·재생·시간 이동이
가능하고 ``motion.gif``에도 양팔 동시 시작, 손 열기 대기, 오른팔 후퇴가 표시된다.
왼팔의 정렬 종료 관절값은 실측값이 아닌 오프라인 예시다.

왼팔 리더 우측 방향 추가 회전 (오프라인 적용)
-------------------------------------------

BT launch의 ``barcode_reader_extra_yaw_deg`` 기본값은 ``-15.0``이다.
기존 position-only 바코드 정렬의 base +Z 기준 yaw -30도에 추가 적용하므로
리더 목표 방향은 총 -45도다. 리더 목표 위치와 기존 왼손 J6 추가30도 굽힘은
유지한다. ``0.0``으로 시작하면 기존 방향이다.

``deux_task_runtime.reader_yaw``가 AlignBarcode 계획 중에만 reader-axis IK에
전달하는 목표 회전을 조정한다. 모델/원본 Planner 및 installation.yaml을
수정하지 않으므로 저장 파지 라이브러리 입력 해시도 유지한다. 지원하지 않는
IK backend에서는 실행하지 않고 오류를 반환한다. 계획 이후의 기존 시작값·
관절·속도·가속도 검사와 추가 실측 컨베이어 검사는 유지한다.

방향 추가 회전은 리더의 목표 yaw를 바꾸는 기능이며, 더 좋은 판독이나 새로운
왼팔 IK 해의 도달성을 실기로 입증한 것은 아니다. Thor 연결이 복구되면
원격 작업 트리 백업, 배포, runtime 빌드 및 Jazzy 통신 검증이 필요하다.

모델 단위 검사::

  PYTHONPATH=src/deux_task_runtime /usr/bin/python3 -m unittest tools/test_reader_yaw.py

2026-11-01 전체 연결 검사는2,177개 중2,090개가 통과했다(일반1,510개,
90도 계열580개). 실패87개는 선택에서 제외한다. 세부 내역은
``artifacts/right-high-scan-waypoint/library-validation.json``에 있다.

기록된 리딩 자세 적용 (2026-10-29 unaligned 데이터)
--------------------------------------------------

``dataset_reading_pose_file`` BT launch 기본값은
``artifacts/dataset-barcode-pose/reading-pose.json``이다. 활성화하면
AlignBarcode는 기존 오른팔 스캔 포즈를 유지하고 JSON의 왼팔 관절값만 적용한다.
이전 추가 yaw15도 및 J6 +30도 굽힘은 중복 적용하지 않는다.
기존 접촉3초/우측 경유점/오른팔 스캔 경로는 유지한다. 동적으로 관측 바코드 위치에 맞추는 IK와는 별개인
고정 리딩 자세 모드다. 바코드 관측/대상 ID 게이트는 기존대로 유지한다.
executor는 왼팔 목표와 계획 종료값이 일치하고, 정렬 경로 전체에서 오른팔 관절값이 변하지 않는지 검사한다.
정렬/병렬 적재 경로는 coordinator에서 실측 컨베이어를 추가 검사한다.

원본은 ``woozziam/deux_breadbag_barcodescan_gr00t_261029_unaligned``의
revision ``063cda3e517a4bb73fc064bcf748318142217a0b``이며 episode 0만
다운로드했다. observation.state의 이름 기준으로 양팔 관절을 매핑했다.
원본 배열은 왼팔7/오른팔7/왼손7/오른손7 순서로, 현재 모델의
왼팔7/왼손7/오른팔7/오른손7 순서와 다르다. 손 모터 값은 URDF 손가락
관절로 직접 복사하지 않는다.

영상으로 확인한 리딩 구간7~14초에서 모든 팔 관절속도<0.1rad/s인
샘플 중 EE X합이 최대인 frame243(8.10초)를 선정했다. 전체 구간의
X 최대는 준비/복귀였으므로 제외했다. 기록된 리딩 자세 자체에 대한
바코드 판독 성공 신호는 데이터에 없다.

이전 순차 연결의 기존 스캔→리딩 계획은4.16초, 컨베이어 모델 최소 여유7.74cm였다.
현재 동시 스캔 이동은 아래 절과 같이 이 순차 이동을 겹친다.
왼팔 기록 자세 + 기존 오른팔 스캔→병렬 후퇴/적재5.96초, 최소 여유3.19cm.
물체 부피와 양팔 상호 충돌 검사는 포함하지 않는다. Thor 실기는 미확인이다.
``dataset_reading_pose_file:=''``로 실행하면 기존 바코드 IK 모드로 돌아간다.

원본/새 경로 시각화는 ``artifacts/dataset-barcode-pose/preview.html``이며
양팔 J1~J7, 원본 ego 영상, 선택 시점 왼손 영상, 원본 관절 재생 및
새 전체 BT 경로 재생을 포함한다. 모델 검사는::

  PYTHONPATH=tools:src/deux_task_core:src/deux_task_runtime /usr/bin/python3 -m unittest tools/test_dataset_reading_pose.py

파지 후 양팔 동시 스캔 이동
-------------------------

데이터셋 왼팔 자세가 활성화된 경우, lookup replay는14개 팔 관절을 하나의
FollowJointTrajectory와 공통 시간축으로 전송한다. 오른팔은 기존 저장 경로와
연속 우측 경유점/스캔 연결을 그대로 사용한다. 왼팔은0~4초 동안 설치 기본자세를
유지하고,4초부터 경유점 도착까지 zero-end-velocity/acceleration quintic으로
데이터셋 스캔자세로 이동한다. 기준 셀의 타임라인은 접촉3초, 양팔 동시 이동
시작4초, 경유점/왼팔 스캔 도착6초, 오른팔 스캔 도착8초다. 다른 셀은
오른팔 연결 시간에 맞춰 왼팔 도착 시각을 함께 조정한다.

기존3초 접촉/병렬 손 닫기는 유지한다. 오른팔만 검사한 결과를 재사용하지 않고
높이 변경 시 양팔 관절 제한·속도·가속도 및 실측 컨베이어 연속 경로를 재검사한다.
높이가 같으면 완료된 오프라인 검사 결과를 재사용한다.
검사 실패 조합은 선택 후보에서 제외한다. 양팔 상호 충돌/자가 충돌은 사용자와
합의한 검사 범위대로 추가하지 않는다. 따라서 모든 파지점의 무조건적 실행
가능성을 주장하지 않는다. 서버는 완료 전 팔 준비 요청을 대기시키고 검사
완료 후 통과한 후보만 사용한다.

20Hz replay 로그에는 왼팔 실측/계획 관절값과 오차도 기록한다. 전송 후 추적 오차는
기존대로 로그를 남기며 임의 취소하지 않는다. 명시적 취소/비상정지는 유지한다.
후속 ScanPose 완료 확인에는 양팔 도착을 확인한다. 이후 AlignBarcode는 이미
도착한 왼팔을 재검증/보정하고 기존 왼손 닫기 동작을 수행하므로, 정상 도착
시 별도4.16초 왼팔 이동이 사라진다. 현재 최소 정지 계획은0.3초이고 왼손
닫기0.5초 완료까지는 기존 executor가 기다린다. 재생 시각화의 구간 길이는
팔 계획 기준이며 인식 대기·실행 통신 지연·실제 손 동작 시간은 별도다.

시각화는 기존 ``artifacts/dataset-barcode-pose/preview.html``을 갱신했다.
전체 셀 재검사 결과는 ``artifacts/right-high-scan-waypoint/library-validation.json``이다.
Thor 배포/새 양팔 실기는 아직 확인 전이다.

2026-11-01 동시 스캔 이동 전체 재검사 완료: 총2,177개 중2,090개 통과
(일반1,510/90도580), 기존 실패87개 제외 유지. 기존 통과 후보의 추가 탈락은
없다. 변경 운반 구간의 최소 컨베이어 모델 여유는3.32cm, 출발→스캔 최대9초다.
대표·원거리·90도 및 실제 서버 gate/추적 메서드 검사 포함57개 단위 검사 통과.
이는 오프라인 모델/로직 증거이며 Jazzy 실통신 또는 실기 검증은 아니다.

Thor 배포 확인 (2026-11-01)
---------------------------

Thor ``/home/xyz_thor/DEUX_ikworld``에 동시 스캔/데이터셋 왼팔/병렬 적재를 배포했다.
배포 전 백업은
``/home/xyz_thor/DEUX_ikworld_backups/20261101-172954-parallel-scan-deploy``다.
덮어쓰는 파일 원본, 소스·설정, BT 상태를 보관했다. 변경하지 않는 모델 가중치,
가상환경, 빌드 산출물 및 대용량 ROS 로그는 소스 백업에서 제외했다.

Jazzy runtime 빌드와 모듈 import,57개 단위 검사, 격리된 ROS_DOMAIN_ID
214~217에서 다음 가짜 제어기 검사를 완료했다.

* 병렬 왼팔 후퇴/오른팔 적재의 단일 Action 및 손 열기 전 왼팔 도착
* 명시적 취소 후 손을 열지 않음
* 실제 BT coroutine/ROS child Action의 반복·피드백 복구·계획 실패·취소 경계
* 실제 LookupBtServer의14관절 replay, 왼팔0~4초 유지/4~6초 이동,
  오른팔8초 스캔 도착, 손 닫기 첫2.5014초/마지막3.0021초

마지막 검사는 시험 환경에서 실제 후보1개만 컴파일하도록 제한했다.
운영 서버는 전체 라이브러리를 검사한다. 새 시험은
``ROS_DOMAIN_ID=217 python tools/test_parallel_scan_ros.py``다.
운영 도메인0/12에서는 시험 스크립트가 실행을 거부한다.

검증 로그는 Thor ``.setup/lookup-bt/deployment-validation/``에 있으며
노트북 ``artifacts/lookup-updated-deployment/thor-validation/``에도 보관한다.
실제 하드웨어 도메인에 로봇 동작 명령을 전송하거나 운영 BT를 시작하지 않았다.

높이 동일 조건에서 재검사 생략
------------------------------

``replay-height-baseline.json``에 리프트 설치 스냅샷(어깨 지면 높이1.34m,
리프트 원점/base 상대값, 등가 prismatic 위치, 카메라 지면 높이)과 컨베이어
지면 높이0.80m/실측 상면 plane 높이를 보관했다. 현재 설정과1mm 이내로
같으면 기존 오프라인 통과2,090개/실패87개 결과를 재사용한다. 시작 시 전체
컨베이어 검사와 후보 선택 후 검사 모두 하지 않는다. 시작 시 동일한 연결
곡선 샘플을 복원하는 계산만 수행하며 노트북 측정4.4초다. 손 닫기3초 일정은
이 복원 단계와 독립적이다.

높이 값이 달라지면 경로 재검사가 필요하다. 리프트 이동으로 카메라 보정이
무효화된 상태는 이전 높이와 같다고 간주하지 않는다. 새 높이/보정을 확인해야
한다. 준비 복귀 전에도 설치 높이 변경 여부만 가볍게 비교한다.
현재에는 리프트/컨베이어의 실시간 높이 측정 토픽이 없으므로, 이 비교는
설정된 설치 스냅샷과 리프트 이동에 따른 보정 무효화 기록을 사용한다.
물리적으로 컨베이어를 옮긴 사실을 별도 센서 없이 자동 감지한다고 주장하지 않는다.

왼손 스캔/후퇴 개폐 순서
-----------------------

현재 왼손은 기본자세·파지·스캔 이동 중에는 열림0.0을 유지한다.
AlignBarcode에서 기존0.5초 스케줄로0.0→1.0 닫힘을 보낸다.
정렬 후 ClearReader/PlaceApproach(병렬 왼팔 후퇴 시작)에서 다시0.0으로 열며,
Place/Release/PlaceRetreat/Retreat/Recover에서도 열림을 유지한다.
오른손의 파지/적재 개폐 시점은 변경하지 않았다.
기본 물체대기 중이던 현재 왼손에도 idle 상태에서0.0을0.5초 전송했다.
관련 Jazzy 단위 검사4개가 통과했다. 운영 BT는 새 정책으로 재시작했다.

데이터셋 고정 스캔자세 모드는 영상 바코드 재검출을 기다리지 않는다.
고정 관절 종료값/오른팔 유지/하드웨어 시작 상태 검사는 유지한다.
동적 바코드 IK 모드만 기존 영상 위치/ID 게이트를 사용한다.
실제 바코드 판독 완료 대기 여부는 기존 skip_barcode_completion 설정에 따른다.

왼손 스캔 위치/J6 보정
---------------------

기록된 frame243 왼팔 자세를 기준으로 base_link +Y5cm, +Z10cm를 적용했다.
왼손 grasp 목표는(0.434836, -0.034465, 1.139288)m다. J6는 기존
-2.305907도에서+27.694093도로 정확히+30도 변경했다. 해당 URDF의 J6 축은
local -X이며 이 자세에서 양의30도는 리더 base yaw를 우측으로 약29.50도 돌린다.
J6를 고정한 채 나머지6개 관절 IK로 요청한 XYZ와 회전 자세를 동시에 맞췄다.
오른팔은 기존 스캔 관절값을 유지한다.

새 목표는 ``reading-pose.json``에 저장했고 원본은
``reading-pose.before-left-offset.json``에 보관했다. 기준 파일 위치는
``artifacts/dataset-barcode-pose/``다. ``tools/offset_left_scan_pose.py``는 같은
원본을 사용하므로 다시 실행해도 오프셋을 누적하지 않는다.
설치 높이 동일 시 전체/후보 궤적 재검사 생략 정책은 유지했다.
새 위치의 FK/IK 수치 일치를 확인한 것이며 새로운 실물 스캔/리딩 성공을
확인한 것은 아니다. 원본 영상은 보정 전 자세이고 현재 스켈레톤은 보정 후다.
Thor 백업은 ``20261101-183415-left-scan-offset``이다.
새 자세로 BT 노드만 다시 로딩했으며 RunTask를 제출하지 않아 실제 팔 이동은 없다.

오른손 기본 대기 개폐값
----------------------

lookup BT의 오른손 기본 대기값은 기존0.5보다 더 열린0.3으로 변경했다.
launch 인수 ``right_hand_ready_fraction:=0.3``을 lookup server에 전달하며,
원본 installation.yaml은 입력 해시 보존을 위해 변경하지 않는다.
기본자세에서 물체를 기다리기 전에 RightTrialHand가0.3을0.5초 전송한다.
이 값은 해당 준비 단계에만 적용된다. 저장 파지 궤적의 손 닫기2.5~3초 및
닫힘 목표1.0, 기존 적재 위치에서 열림0.0은 유지한다.
다른 값은 BT 시작 시 launch 인수로 지정한다.

현재 스캔 기준 J6 왼쪽30도
--------------------------

이전 적용 자세에서 J6를27.694093도→-2.305907도로30도 줄였다.
왼손 XYZ는(0.434836, -0.034465, 1.139288)m로 유지했다.
즉 데이터셋 기준 왼쪽5cm/위10cm 보정은 유지하고, 직전 우측30도 J6 증가를
되돌렸다. 현 자세에서 J6를 감소시킨 FK 회전을 목표로 하고 나머지6개 관절 IK로
XYZ를 보정했다. 원본 데이터셋의 다른 관절값을 그대로 복사한 것은 아니다.
오른팔 스캔/오른손 준비0.3/왼손 정렬 닫기와 후퇴 열기 일정은 유지한다.
변경 전 자세는 ``reading-pose.before-j6-left30.json``에 보관했다.
반복 BT는 물체 대기 중인 상태에서만 중단하고 새 자세로 재시작했다.

J6 추가 왼쪽10도
----------------

직전 스캔 정렬 자세의 왼팔 J6를-2.305907도에서-12.305907도로 추가10도 줄였다.
왼손 XYZ(0.434836, -0.034465, 1.139288)m는 유지하도록 나머지6개 관절을
IK로 보정했다. 변경 전 값은 ``reading-pose.before-j6-left10.json``에 보관했다.
오른팔/오른손 준비0.3/왼손 정렬 닫기·후퇴 열기 일정은 변경하지 않았다.
BT는 빈 물체 대기 상태에서만 중단하고 새 자세로 재시작했다.

왼쪽5cm/위3cm 추가 이동 및 승인된 리더축 회전
--------------------------------------------

직전 스캔 포즈에서 base_link +Y0.05m/+Z0.03m를 추가했다.
새 왼손 위치는(0.434836, 0.015535, 1.169288)m다.
J6=-12.305907도와 리더의 바라보는 축은 유지한다. 전체 회전까지 고정하면
J4 최대120도 한계로 목표 XYZ를 만족하지 못해, 사용자 승인에 따라 리더 축
주위 회전15.0216도를 허용했다. 선택된 J4는119도(한계까지 약1도)다.
정확한7개 관절값/승인 조건은 reading-pose.json에 보관한다.
이전 포즈는 reading-pose.before-left5-up3.json, 해는
translation-margin1-reader_axis_fixed_j6.json에 보관했다.
오른팔/양손 개폐/기존 궤적 재검사 생략 정책은 유지한다.
동작 중인 팔 궤적은 중단하지 않고 물체 대기 또는 회차 완료 후 반영했다.

왼팔 J7만 우측30도 회전
----------------------

직전 적용 스캔 자세에서 J7만-5.673139도→-35.673139도로 변경했다.
URDF의 J7 양의 축이 모델 리더의 바라보는 축과 반대이므로, 로봇 쪽에서
리더 시선 방향으로 볼 때 우측/시계방향30도는 관절값-30도에 해당한다.
J1~J6와 오른팔 관절값은 그대로이며 XYZ 보정 IK는 수행하지 않았다.
이에 따른 왼손 grasp FK 위치는(0.405130, 0.040265, 1.245162)m다.
기존 위치 대비(-2.97cm,+2.47cm,+7.59cm)는 이 J7 회전 자체의 결과다.
변경 전 자세는 reading-pose.before-j7-right30.json에 보관했다.
물체 대기 상태에서만 BT를 멈추고 새 자세로 반복 실행을 재개했다.

J7 원복 후 J6만 왼쪽10도
------------------------

직전 J7 우측30도 변경은 원복하여 J7=-5.673139도로 되돌렸다.
그 기준 자세에서 J6만-12.305907도→-22.305907도로 추가10도 줄였다.
J1~J5와 원복된 J7은 변경하지 않았으며 XYZ 보정 IK도 수행하지 않았다.
따라서 새 왼손 grasp FK는(0.457715, 0.039938, 1.164088)m다.
다른 관절/오른팔/손 개폐 일정은 그대로이며 물체 대기에서만 BT를 멈추고
새 포즈로 반복 실행을 재개했다.

J6만 우측20도
--------------

스캔 정렬 자세의 왼팔 J6만-22.305907도에서-2.305907도로20도 늘렸다.
다른6개 왼팔 관절과 오른팔은 변경하지 않았고 XYZ 보정 IK도 수행하지 않았다.
왼손 grasp FK는(0.407973, -0.004626, 1.173517)m다.
이전 자세는 reading-pose.before-j6-right20.json에 보관했다.
진행 중 회차를 완료한 뒤 새 포즈로 반복 BT를 재개했다.

스캔 bbox 존재 확인 (2026-11-01)
------------------------------
스캔 도착 후 최대1초 동안 새 RGB 검출을 확인한다. 현재 인식 요청 ID와
일치하고 스캔 도착 이후 취득한, max_object_age 이내의 영상만 사용한다.
bread_bag bbox 면적3,000px² 이상 또는 barcode bbox 면적300px² 이상이면
정렬/적재를 계속한다. 둘 다 없으면 정렬/적재를 생략하고 lookup 준비자세로
복귀한 뒤 다음 물체를 기다린다. 준비 복귀 자체가 실패하면 정지한다.
빵 검출은 현재 카메라 전체 영상의 존재 판정이며 물리적 파지 성공 증명은 아니다.
640×360 원본 RGB 기준 면적이며 해상도 변경 시 임계값도 조정해야 한다.
launch 인자 scan_bbox_wait_s, scan_bag_min_area_px2,
scan_barcode_min_area_px2로 조정한다. BT 회차 JSON의 scan_bbox_evidence에
임계값/판정/통과 bbox를 저장하고 미검출 회차는 failed에 집계한다.
다음 파지 대기 시작 시 기존 정책대로 오른손을 준비 열림값으로 보낸다.

준비 복귀 시간 조정 (2026-11-01)
-----------------------------
lookup 준비 복귀 서버의 ready_velocity_cap=0.45rad/s,
ready_acceleration_cap=0.9rad/s²를 적용했다. 로봇 관절별 제한과 이 값 중
더 작은 값을 사용하며, 기존 경로/오른팔→왼팔 순서/경유점 정지는 유지한다.
스캔 미검출 복귀와 일반 준비 복귀에 동일하게 적용한다. 현재 스캔 목표에서
준비 목표까지 오프라인 산출 궤적 시간은16.15초에서5.38초로 줄었다.
이는 명령 궤적 시간이며1초 검출 대기/계획/제어기 완료 시간은 별도다.

스캔 빵 존재 조건 완화 (2026-11-01)
--------------------------------
최신 실패 bt-9dbc1a7c1c59441ea6e8cc857aa1896e 회차의 통과 bbox는 없었으며,
당시 탈락 bbox/영상은 보관되지 않아 실제 면적은 복원할 수 없었다.
스캔 barcode 모드에서 같은 RGB 추론의 raw 빵 bbox도 존재 판정에 사용한다.
ROI 진입/추적 ID 유지 조건은 이 스캔 존재 판정에만 적용하지 않는다.
raw 빵 confidence>=0.15, 면적>=1,000px²를 허용하며, barcode>=300px²,
도착 후 새 영상/요청 ID/유효시간/최대1초 조건은 유지한다.
이전 detector 프레임의 raw bbox를 재사용하지 않는다. 빵 존재는 전체 영상
기준이므로 컨베이어의 다른 봉투도 통과 근거가 될 수 있으며 파지 증명이 아니다.
파지 후보 선택/컨베이어 추적/ICP 조건은 변경하지 않았다.
scan_bbox_evidence의 fresh_frames와 last_boxes에 미통과 최신 bbox도 남긴다.

경유점 이동 중 봉투 확인 (2026-11-03)
------------------------------------

이 절은 이전 스캔 도착 후 bbox 검사 설명을 대체한다. 파지→경유점과
경유점→스캔의 저장 궤적/속도는 변경하지 않는다. lookup 피드백의 cell_id와
검증 기록의 waypoint_at_s로 경유점 전후0.5초 창을 정하고, 해당 창에서
barcode 모드의 새 RGB 봉투/바코드 bbox를 검사한다. 실제 요청 전 영상,
다른 요청 ID, 오래된 영상, ROI 밖 검출은 통과하지 않는다.

검출되면 전송된 궤적을 그대로 끝까지 진행한다. 미검출이면 lookup Action과
실행 중 제어기 궤적의 취소/종료를 확인하고, 팔이0.3초간 정지한 것을 확인한
뒤 기존 준비 복귀를 수행한다. 종료 또는 정지가 확인되지 않으면 복귀를
시작하지 않는다. 이동 중 검사이므로 미검출 결정 전 경유점을 약0.5초
지나갈 수 있으며, 스캔 도착 후 중복 검사는 하지 않는다.

calibration/via_presence_roi.json은 현재 경유점에서 촬영된 봉투 검출15장의
중앙값 bbox에 가로/세로 길이의10%를 각각 사방으로 확장한 별도 ROI다.
640×360에서 대략(437,87)–(559,202)이며 기존 컨베이어 추적 ROI는 유지한다.
기존 중심점 포함 판정과 최소 면적(봉투1000px², 바코드300px²)을 유지한다.

검증: 단위 검사38개와 격리 Jazzy의 검출/미검출 분기 시험을 통과했다.
가짜 제어기에서 미검출 취소 시각6.51초 및 준비 복귀를 확인했다.
새 분기의 실물 동작은 실행하지 않았다. artifacts/via-presence-roi/validation/ 참조.

미검출 전용 부드러운 준비 복귀 (2026-11-03)
-------------------------------------------

ViaEmptyReturnReady만 /task/lookup_empty_return Action을 사용한다. 일반 준비,
차렷 복귀, 파지→경유점→스캔은 기존 동작을 사용한다. 공유 실행 잠금과 취소,
정지 확인, 컨베이어 경로 검사를 유지한다.

기존 복귀의 기하 경로에 5차 곡선(10u³−15u⁴+6u⁵)을 적용하고 모든 경로점에서
속도와 가속도를0으로 지정한다. 위치·속도·가속도를 함께 전송해 제어기의
splines 보간을 사용하며, 관절 속도0.30rad/s, 가속도0.25rad/s²,
jerk0.8rad/s³ 상한으로 시간을 산정한다. 가속도는 경로점에서 연속이며
jerk 자체의 연속성은 보장하지 않는다. 경유점 통과 후0.51초인 대표 자세에서
복귀 시간은 약3.98초→7.89초로 늘어난다.

이 변경은 취소 후 정지한 상태에서 시작하는 복귀 곡선에만 적용한다.
미검출 시 진행 중 궤적을 취소하는 순간의 제어기 정지 방식은 변경하지 않는다.

경유점 확인: 5프레임과 EE depth 필터 (2026-11-03)
-------------------------------------------------

기존 검사 창(경유점 전후0.5초)에서 ROI·면적·신선도 조건과 depth 조건을
동시에 만족하는 서로 다른 촬영 프레임을5개 이상 요구한다. 누적 기준이며
연속5프레임 조건은 아니다. 같은 프레임에 bbox가 여러 개 있거나 같은
메시지가 재수신돼도1회만 센다. 4프레임까지는 통과하지 않는다.

RGB와 등록 depth/CameraInfo를20ms 이내로 동기화하고, 검출 영상과 동일한
RGB 촬영 시각의 depth만 사용한다. 해당 촬영 시각 이전100ms 이내의 실제
팔 관절값으로 오른손 EE FK를 계산하고, 외부 카메라의 실제 TF로 광학 좌표계
Z 깊이를 구한다. bbox 중앙60% 영역의 유효 depth 중앙값이 EE 깊이±0.15m
이내여야 한다. 유효 픽셀은30개 이상 및 영역의30% 이상이어야 한다.
16UC1/MONO16의 mm는 m로 변환한다. TF·관절값·depth가 없거나 보정이
무효화됐으면 depth 검사를 생략하지 않고 해당 프레임을 탈락시킨다.

launch 인자는 via_presence_min_frames(기본5, 최소5),
via_depth_tolerance_m(기본0.15)다. 실측 보정으로 계산한 경유점 EE Z는
약0.374m이며 현재 배경 중앙값은 약0.964m였다. EE 기준 범위는 매 영상에서
갱신하며, 봉투 형상에 맞춘 정밀 파지 검증이 아닌 대략적인 깊이 필터다.
검사 기록에 matched_frames, matched_stamps_s, last_depth를 저장한다.

검사 프레임 저장과 손 제외 / 모델 교체 (2026-11-03)
---------------------------------------------------

tracked_perception.thor.yaml의 weights를 models/deux_1102_bread_detection.pt로
교체했다. 실제 모델은 detect이며 클래스는 b1,m1,s1,s1_1,bar다. 기존 봉투
대상 클래스 s1,s1_1 및 바코드 bar 매핑은 유지한다. 분할 마스크를 출력하지
않으므로 봉투 후보 영역은 bbox 내부90%를 사용하고, 향후 polygon_xy가
제공되면 그 다각형으로 추가 제한한다.

촬영 시각 이전100ms 이내 팔/손가락 측정값으로 URDF의 양손 및 양쪽 손목
(arm_6, arm_7) collision mesh를 투영한다. 각 링크의 보수적인 볼록 외곽을
합치고8px 확장해 제외한다. 카메라는 기존 외부 ZED 보정 TF만 사용한다.
이 마스크 밖의 후보 픽셀만 depth 중앙값 계산에 사용하며, 남은 영역은
100픽셀 이상 및 원 후보 영역의15% 이상이어야 한다. 유효 depth는 남은
영역의30% 이상/30픽셀 이상이어야 한다. 손 관절·TF·메시가 없으면 제외를
생략하지 않고 해당 검사에서 통과시키지 않는다.

검사 요청별 저장 경로::

  .setup/lookup-bt/via-frames/<request_id>/<RGB stamp ns>/

RGB 및 오버레이 이미지 저장은 비활성화했다. depth_m.npy는
정밀도를 유지한 미터 단위 depth(NaN 포함), hand_excluded.png는 무손실
제외 마스크,
decision.json은 bbox/검출기 해시/관절값/카메라 변환/깊이/탈락 이유다.
검출이 없는 신선한 검사 프레임도 저장한다. 비동기 큐는32프레임으로
제한하며 디스크 공간 확인·파일 쓰기는 백그라운드 작업자가 수행한다.
저장 실패/잔여 공간512MiB 미만/큐 포화는 별도 저장 오류로 기록하며
검출 결과에 영향을 주지 않는다. 5프레임 충족 후 추가 수집을 멈추고
파일 저장 완료를 기다리지 않고 즉시 통과한다. 기록의 archive_status에는
대기·저장 성공·실패·큐 포화로 누락된 프레임 수와 마지막 오류가 남는다.
저장 자료는 자동 삭제하지 않는다. 이 마스크는 보정된 픽셀 분할 결과가
아니라 URDF 기반의 보수적 손 영역이므로 실제 손 위치/가림에 따른 오차가
남을 수 있다. 실제 촬영 예시와 시험 기록은 artifacts/via-hand-exclusion/에 있다.

Lookup 자세 허용 오차 (2026-11-03)
----------------------------------
사용자 요청으로 Lookup BT 차렷 최종 오차와 오른팔 준비/재생 시작 오차를
0.03에서 0.05rad로 완화했다. 왼팔 준비 허용 오차는 6도에서 8도로
완화했다. 각 관절 절대 오차의 최댓값으로 판정하며, 이동 중 추종 제한·
계획 중 자세 변화 검사·충돌 검사는 유지한다. 일반 단독 grid 실행은
기존 0.03rad를 유지한다. 변경은 BT 재시작부터 적용된다.

미검출 감속 정지 / 스캔 J7 / confidence (2026-11-03)
---------------------------------------------------
경유점 봉투 미검출은 ``/task/lookup_soft_stop``으로 선택된 양팔 경로를
감속 궤적으로 교체한다. 파지·들어올리는 동안 감속 후보를 미리 검증하고,
정지 요청 시 현재 진행 시점 이후 후보를 선택한다. 경로 진행 속도는
1에서 0으로 5차 smoothstep으로 줄이며, 기본 감속 시간은 1초다.
후보별 관절·속도·가속도 한계 및 연속 컨베이어 여유를 검사한다.
정지 완료와 관절 안정화를 확인해야 기존 저가속 준비 복귀로 넘어간다.
감속 후보/제어기 실패에는 취소 후 작업을 실패 처리하며 자동 복귀하지 않는다.
사용자 취소·비상정지는 감속 요청을 기다리지 않는다.

스캔 왼팔 J7은 -5.673139도에서 +4.326861도로 10도 증가했다.
설치 URDF FK에서 EE Z가 약 2.80cm 낮아지는 방향이며 다른 관절은 유지한다.
전체 2,177개 경로 중 2,175개 통과, 기존 2개 제외를 유지했다.
이는 경로/컨베이어 모델 검증이며 봉투 부피·양팔 전체 충돌 검증은 아니다.

추적 최초 획득 confidence는 기존 0.5를 유지한다. 경유점용 raw 봉투
박스는 기존 0.15 대신 설정 confidence(0.5)를 사용한다. 별도로 BT 검사도
``via_presence_min_confidence=0.5``를 적용하여 추적 유지 중 낮은 score가
5프레임 판정을 우회하지 못하게 한다. 카메라 영상/검출 score·검사 기준은
검사 로그에 기록한다. ROI와 depth 허용 폭은 이번 분석에서 변경하지 않았다.

봉투 최소 박스 면적 (2026-11-03)
-------------------------------
사용자 지정 오검출 프레임 1793666700800442026의 원본 640×360 박스는
면적 2586.9916445px²다. 2배인 5173.9832891px²를 올림한
``scan_bag_min_area_px2=5174.0``을 기본값으로 적용한다. 확대된 시각화
이미지가 아닌 원본 검출 좌표 기준이며 봉투 박스 면적을 검사한다.
confidence 0.5, ROI, depth, 5개 고유 프레임 조건도 함께 유지한다.

미검출 감속 강화 (2026-11-03)
----------------------------
감속 후보는 0.6초를 우선한다. 관절 속도/가속도·컨베이어 여유 검증에
실패하면 0.8, 1.0, 1.2, 1.4초 순서로 늘린다. 감속 곡선과 정지 확인,
준비 복귀 속도는 유지한다. 실제 선택 시간은 lookup 결과의 soft_stop.duration_s에 기록한다.

미검출 정지 중 EE 이동량 감소 (2026-11-03)
----------------------------------------
``lookup_soft_stop.build_soft_stop``는 공통 경로 감속과 관절별 시간 재매개화
후보를 비교한다. 각 관절은 기존 궤적의 동일한 시작 위치·속도에서 출발하고
서로 다른 시간에 속도 0으로 정지한다. J7을 포함한 모든 관절의 기존 속도·
가속도·위치 제한은 유지한다. 후보 목적함수는 오른손 EE 이동거리 + 왼손
이동거리의 0.25배이며, 양손 각각의 이동거리와 최대 이탈거리 모두 기존
공통 경로 감속보다 늘어나지 않는 후보만 허용한다. 이는 제한된 후보 집합의
최소값 선택이며 전역 최적화 보장은 아니다.

최종 전송 cubic 궤적의 정확한 관절/속도/가속도 극값 및 연속 컨베이어 여유를
검증한다. 충돌 검증 범위는 기존 설치의 컨베이어/팔 capsule 모델이며,
양팔 상호·전체 주변 물체 충돌을 검증했다는 의미는 아니다. 개선되고 검증된
후보가 없으면 기존 공통 경로 감속을 사용한다. 결과 JSON에 profile,
joint_stop_durations_s, ee_travel, baseline_ee_travel, joint_dynamic_ratios를
저장한다. 전체 정지 시간은 가장 늦게 멈추는 관절에 의해 결정된다.

직전 x+019_y+000_yaw-100 경로의 경유점+0.65초 기준 오프라인 비교:
오른손 EE 이동거리 0.12153→0.09671m, 최대 이탈거리 0.11909→0.06533m.
오른 J7 감속은 1.0초, 나머지 오른 관절은 0.2~0.6초로 검증됐다.
검증 자료는 artifacts/jointwise-stop/에 저장하며 수치는 실제 손끝 측정값이
아닌 설치 URDF FK 기반이다.

스캔 정렬 왼팔 J7 위로 5도 (2026-11-03)
--------------------------------------
현재 자세를 기준으로 J7만 +4.326861도에서 -0.673139도로 변경했다.
설치 URDF FK에서 EE Z가 1.405cm 높아지는 방향이다. 이전 자세는
reading-pose.before-j7-up5.json에 보존했다. 다른 관절은 유지하며,
경유점→스캔과 AlignBarcode는 동일한 새 목표 자세를 사용한다.

스캔 정렬 끝의 base 좌측 yaw 5도 (2026-11-03)
--------------------------------------------
원래 왼팔 스캔 목표는 유지하고 AlignBarcode 마지막에 alignment_tail을
추가했다. 먼저 원래 목표에 도착해 왼손을 닫고, 최소 0.1초 여유 후
base_link +Z 축으로 +5도 회전한다(위에서 보면 반시계). 회전은 약
1.16초이며 오른팔은 측정된 시작 관절값을 유지한다. 팔 전체 IK를 사용하며
J7 단독 회전이 아니다. 현재 초기 정렬/닫힘을 포함한 모델 예시는 1.76초다.

J4 상한에 0.1도 여유를 두어 모델상 최종 yaw 4.9991도, 회전 중 EE 위치
변화 최대 0.486mm로 검증했다. 완벽한 위치 고정 대신 1mm/축외 회전
0.002rad 이내를 검사한다. 런타임은 URDF 해시와 고정 관절 설정, 원래
스캔 관절점, 최종 yaw FK를 검증하며 모델 변경 시 꼬리 경로를 재생성해야 한다.
기존 경유점→스캔 곡선의 왼팔 목표값은 변경하지 않았다.

검증: tools/test_dataset_reading_pose.py의 생산 경로·위치/회전·후퇴 검사,
격리된 ROS 도메인의 실제 ExecutionNode + 가상 팔/핸드 제어기 실행·취소.
스켈레톤은 artifacts/scan-alignment-yaw5/skeleton.gif 및 skeleton.png이며
실제 로봇 영상이 아닌 설치 URDF 기반 모델 시각화다. 검증 범위는 기존
관절/동역학 및 컨베이어 모델이며 전체 주변 물체 충돌 검증은 아니다.

JPEG 비동기 저장 회귀 수정 (2026-11-03)
-------------------------------------
32회차에서 5프레임이 유효했지만 파일 저장 대기로 미검출 처리된 문제를
수정했다. 검출 마감 시점 전에 계산된 5프레임 결과만 판정하고, 검사
코루틴이 늦게 재개돼도 이미 확보한 유효 결과를 버리지 않는다. 저장
지연·오류를 미검출로 분류하지 않는다. 기존 PNG 캡처는 그대로 보존한다.
