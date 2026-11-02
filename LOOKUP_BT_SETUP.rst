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
현재 저장된 기존 실행 경로를 사용하며, 별도 시각화한6.8초 단축 후보는 적용하지 않는다.
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
-----------

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
