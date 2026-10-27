ROS 2 MuJoCo 연결과 메시지 동기화 녹화
=====================================

2026-10-25 사용자 요청: sim도 동일한 ROS 2 인터페이스로 연결.
이후 범위 조정: 물리 태스크 성공까지 튜닝하지 않고, 실행 흐름을 녹화한 뒤
같은 시각의 노드·메시지를 시각화.

구현
----

새 패키지 deux_task_sim은 기존 DEUX_mujoco의 모델·장면·물리 엔진을 사용합니다.
기존 DEUX 실기 코드와 DEUX_mujoco 원본은 변경하지 않습니다.

* task_node / motion_planner_node / execution_node / arm_bridge_node /
  hand_bridge_node / barcode_bridge_node / state_observation_node는 실제 ROS 구현을 사용합니다.
* mujoco_driver_node는 팔 FollowJointTrajectory Action 서버 및 손 HandCtlMessage
  수신 드라이버입니다. 실제 MuJoCo 관절값을 JointState로 발행합니다.
* /clock은 MuJoCo 시간입니다. 나머지 노드는 use_sim_time=true로 동작합니다.
* 센서는 렌더 가시성으로 제한한 sim truth6D ObjectDetectionArray와 분석적 BarcodeTarget을
  제공합니다. 실제 YOLO/ICP 실행은 이 검증 범위에 포함하지 않습니다.
* 가상 리더는 기존 SimBarcodeSource의 거리·방향·250ms 유지 조건을 만족해야 문자열을
  발생시킵니다. barcode_bridge_node의 simulated=true가 유지됩니다.
* 정지 컨베이어, seed4, 고정된 테스트 설치를 사용합니다. 오른쪽 출발 트레이와 내용물을
  실행 전에 함께10cm 이동시킵니다. 실행 중 물체를 이동시키거나 weld로 붙이지 않습니다.
* MuJoCo의 알려진 액추에이터 강성 및 관측 접점 모델로 생성한 sim 전용 preload를 사용합니다.
  생성 설정은 실기 보정값이 아닙니다. 팔 gravcomp 등 원래 sim의 모델 설정은 유지합니다.
* 각 노드는 같은 검증 프로세스의 ROS executor에서 실행하며, 계획 계산은 별도 프로세스입니다.
  DDS 토픽·Action 경로를 사용하지만 별도 장치/프로세스 배포 지연 시험은 아닙니다.

연결 시험으로 수정한 공통 코드
-----------------------------

* loop_pick_scene가 URDF 로드 후 리프트를15cm 올리는 변경을 sim 계획용 URDF에도 반영.
  초기 양팔 FK가 MuJoCo 측정 프레임과1e-6 오차 이내인지 확인합니다.
* Approach는 도달 불가능한 중간 손목 방향을 피하도록 관절 경로로 계획합니다.
  Descend/Lift는 Cartesian이며 모든 경로의 기존 충돌/속도/가속도 검사는 유지합니다.
* MuJoCo soft limit의 수치 오차를 위한 robot.joint_limit_tolerance 옵션을 추가했습니다.
  기본1e-7rad, 허용 설정 범위0..0.001rad; sim은1e-4rad입니다. 측정값을 클리핑하지 않습니다.
* ROS 시간 마감은 유지하고, synthetic=true + use_sim_time=true인 경우에만
  simulation_wall_timeout_scale로 느린 소프트웨어 렌더링의 벽시계 watchdog 여유를 둡니다.
  기본 배율1, sim8. 잘못된 조합은 시작 시 거부합니다.
* 성공한 손 명령의 마지막 목표를 측정 각도로 덮어쓰지 않아 preload를 유지합니다.
  Lift/Scan/AlignBarcode/Place는 설정된 hand_closed 목표를 유지하며 측정 시작 오차는
  계속 검사합니다. 실패/취소 시에는 기존 측정 위치 hold 정책을 유지합니다.

녹화와 관측 의미
----------------

실행 중 MuJoCo qpos와 시각을 저장하고 종료 후 해당 물리 상태를 렌더링합니다.
영상 렌더링/인코딩이 제어 callback을 막지 않도록 분리했습니다.
프레임 간격은0.102초이며 영상0초가 대응하는 ROS 시각은 video-frames.json에 기록됩니다.
실행 중의 렌더 가시성 기반 센서 계산은 별도로 계속 수행합니다.

* Topic:14개 업무 토픽의 실제 DDS 구독 수신 시각과 메시지 요약.
* Action:6개 Action의 client API 전송·수락/거부·feedback·result·cancel 경계.
  기록을 위해 추가 goal이나 결과 요청을 보내지 않습니다.
* 각 이벤트는 time_s(ROS/sim 시각), wall_s(실행 경과 벽시계), 이름, 송신 역할,
  수신 대상, phase, goal_id, payload 요약을 보존합니다.
* Topic 수신 대상은 코드상 구독 관계이며 각 노드의 callback 처리 완료 증거가 아닙니다.
  Action 기록은 네트워크 패킷 캡처가 아닙니다. /clock과 /tf 내부 트래픽은 목록에서 제외합니다.
* 시나리오 클라이언트는 mujoco_driver_node 내부의 RunTask 시작 역할로 표시합니다.

HTML에는 영상, 단계 구간, 노드 연결 강조, 시점별 메시지 묶음, 메시지 상세가 있습니다.
단계/이벤트 선택은 영상을 이동시키며 노드/텍스트/고빈도 스트림 필터를 지원합니다.

실행
----

호스트 Docker와 ffmpeg 필요. 컨테이너에 장치나 host network를 연결하지 않습니다.::

  bash tools/validate_sim.sh

기본 외부 경로는 환경변수로 바꿀 수 있습니다:
DEUX_SIM_REPOSITORY, DEUX_HAND_MSGS, DEUX_SIM_RESULTS.
기존 사용자 방침대로 tools/와 validation/은 커밋 대상에서 제외할 수 있습니다.

이미 준비된 ROS 2 Jazzy 환경에서는::

  ros2 run deux_task_sim validate_transfer \
    --sim-repository /absolute/DEUX_mujoco \
    --output /absolute/recording --record --observe-flow
  ros2 run deux_task_sim render_timeline --output /absolute/recording

MuJoCo3.13.0, imageio, Pillow, OpenCV 및 소프트웨어 렌더링 라이브러리가 필요합니다.
requirements.txt와 tools/validate_sim.sh의 OSMesa 설정을 참고하십시오.
--observe-flow는 태스크 실패 자체를 검증 프로그램 오류로 처리하지 않지만 실제 실패를
result.json과 화면에 그대로 남깁니다. 기본 모드는 물리 태스크 성공을 요구하며,
현재 장면에서 그 모드가 통과한다고 주장하지 않습니다.

현재 실행 결과
--------------

Approach → Descend → Close → Lift → VerifyLift → ScanSearch → AlignBarcode →
WaitBarcode → Place 요청까지 진행했습니다. 가상 판독 문자열을 수신했고,
Place 계획에서 IK unreachable이 반환되어 Failed로 종료했습니다.
따라서 ROS 호출·피드백·오류 반환의 실행 증거이며, 전체 시나리오 정상 완료 또는
물리 태스크 성공 판정이 아닙니다. 사용자의 범위 조정에 따라 적재 성공 튜닝은 중단했습니다.
Release/Retreat/VerifyPlace는 이 최종 녹화에서 실행되지 않았습니다.

결과 위치: validation/ros2-sim/scenario-recording/

* dashboard.html, scenario.mp4: 동기화 화면과 영상.
* ros-events.json, video-frames.json, recorded-qpos.npz: 원본 시각/물리 기록.
* result.json, trace.json: 실제 결과와 상태 궤적.
* installation.yaml, hand_mapping.yaml, robot.urdf: 해당 실행의 sim 설정.
* browser-check.json: 재생·이동·필터 검증 기록.

공통 코드 회귀:52개 테스트 통과. 로봇 명령은 MuJoCo 드라이버에만 전달했습니다.
실기, 이동 컨베이어 추적, 실제 카메라/YOLO/ICP 인식 성능, 실제 바코드 판독은 미검증입니다.


실제 로봇 전환 시 교체·설정할 부분
----------------------------------

mujoco_driver_node는 실기에서 실행하지 않습니다. 이 노드에 묶여 있던 구동,
센서, 가상 리더를 아래 실제 장치 입력/출력으로 분리합니다.
다음은 코드 기준 통합 지점이며, 실기 동작 검증 결과가 아닙니다.

* 팔: 실제 FollowJointTrajectory 서버 및 /arm/joint_states 측정 입력.
  arm_bridge_node의 driver_action 기본값은
  /arm/deux_arm_controller/follow_joint_trajectory입니다.
  장치 이름이 다르면 노드 파라미터/토픽 remap을 적용합니다.
* 손: /hand/left/cmd 및 /hand/right/cmd의 HandCtlMessage를 실제 드라이버에 연결.
  hand_mapping.yaml의 motor ID, position mode, 부호/offset, 범위, 속도,
  goal_current 단위, feedback_topic/type을 실제 프로토콜에 맞춥니다.
  bridge는 측정값을 /task/hand_state로 집계합니다.
  sim의 mode7/ID0..6, preload, hand_open/hand_closed를 그대로 옮기지 않습니다.
* 인식: 외부 ZED RGB-D/CameraInfo 및 보정된 TF를 perception_node에 연결.
  공통 YOLO의 봉투/바코드 검출과 ICP/라벨 표면 worker가 sim 센서를 대체합니다.
  controlled_perception.yaml에 weights, model_path/origin, 클래스 및 frame을 설정합니다.
  /task/perception_request를 받아 /task/object_detections, /task/barcode_target을 발행하고,
  state_observation_node는 유지합니다. 현재 yaw ICP의 full_orientation=false로는
  Place 조건을 충족하지 못합니다. 파지 후6D 관측 생산자 통합은 추가 구현/통합 과제입니다.
* 바코드 문자열: 실제 USB/serial/SDK 리더의 결과를 std_msgs/String으로 내보내는
  드라이버/어댑터가 필요합니다. 이 overlay가 해당 실제 장치 드라이버를 제공하지는 않습니다.
  task.launch.py의 barcode_topic으로 입력을 지정합니다. 기본 /barcode/raw이며
  barcode_bridge_node의 simulated=false를 사용합니다.
  /task/scan_session과 /task/barcode_read의 세션 매칭은 유지합니다.
* 시간/TF: use_sim_time=false, simulation_wall_timeout_scale=1.
  실제 관절값 기반 robot_state_publisher/TF 및 장치 간 시각 동기화가 필요합니다.
  외부 ZED→base는 실측 보정하고, 기존 head camera pose를 대신 사용하지 않습니다.
  sim에서 보정한 리프트 +15cm는 실기 URDF에 복사하지 않습니다.
* 설치: src/deux_task_bringup/config/installation.yaml 템플릿을 실측값으로 작성합니다.
  실제 expanded URDF/리프트/EE 프레임, grasp/reader/label 변환, 상자/목표 위치,
  충돌 형상, 관절 제한, start/tracking 허용 오차와 손 목표값이 필요합니다.
  synthetic=false, calibration_confirmed/command_ownership_confirmed 및
  /state/status의 allowed_state_modes는 실제 확인 결과로 채웁니다.
  sim의 단순 충돌 형상과 넓힌 허용 오차를 실기 보정값으로 사용하지 않습니다.
* 시작 클라이언트: sim 내부 클라이언트를 운영 UI/ROS Action 클라이언트로 교체하여
  동일한 /task/run을 호출합니다. task.launch.py는 팔/손/카메라 드라이버를 실행하지 않습니다.
  실제 명령 전송은 hardware_enabled=true, dry_run=false 설정이며 기본값은 비활성입니다.

유지하는 코드는 task_node, motion_planner_node, execution_node, arm_bridge_node,
hand_bridge_node, barcode_bridge_node, state_observation_node 및 ROS 메시지/Action 계약입니다.
설치 YAML은 계속 각 노드가 시작 시 직접 읽습니다. scene_node나 설정 배포 토픽은 없습니다.
동일 인터페이스 사용이 실기 검증 완료를 뜻하지는 않습니다. Place IK 실패와
아직 실행하지 않은 Release/Retreat/VerifyPlace 단계도 남아 있습니다.

녹화 화면 하단의 '실제 로봇으로 전환할 때' 표에도 같은 항목을 표시합니다.

커밋 범위 및 실기 단계별 적용/검증 계획은 hardware_rollout.rst를 참고하십시오.


미사용 구독 정리 후 검증 (2026-10-25)
-----------------------------------

RuntimeNode의 공통 구독을 역할별 선택으로 변경했습니다.
StateObservation은 task/planner/execution, BarcodeTarget은 task/execution,
/state/status는 execution/arm/hand만 구독합니다. task의 별도 라벨 callback은
유지하고 부모의 중복 구독을 제거했습니다. 총8개 구독 객체가 제거되었습니다.
브리지 직접 측정 피드백, planner의 계획 후 최신 관측 비교 및 실행 감시는 유지합니다.
구조도와 새 sim trace의 수신 대상 목록도 같은 구조로 갱신했습니다.
기존 scenario-recording은 당시 실행 구조를 보존하는 과거 기록입니다.

Jazzy8개 패키지 빌드 및53개 회귀 테스트 통과. 변경 전후 PlanMotion 요청은
Approach/Descend/Close/Lift/AlignBarcode/Place로 동일하며, barcode 모드 요청과
BarcodeRead 수신을 확인했습니다. Place는 기존 IK unreachable로 종료했고,
새 실행 오류/물리 경고는 없습니다. 전체 태스크 성공 검증은 아닙니다.
짧은 VerifyLift/ScanSearch 상태는 주기적 상태 기록에서 누락될 수 있으므로
샘플링된 단계 목록의 완전 일치 대신 실제 Action goal 순서와 판독 이벤트를 비교했습니다.
증거: validation/ros2-sim-subscriptions/{build.log,regression.log,flow-comparison.json}.
새 영상/화면은 validation/ros2-sim-subscriptions/scenario-recording/에 있습니다.


최초 준비 이동과 로봇 베이스 후퇴 (2026-10-25)
--------------------------------------------

사용자 요청에 따라 URDF 관절0 자세에서 준비 자세로 이동하는 Prepare 단계를 추가했습니다.
이후 최초 이동은 반복하지 않습니다. 작업 중 파지/스캔 등 기존 동작은 그대로 수행하고,
작업 사이에 관절0 자세로 리셋하지 않습니다.

* prepare_before_first_task=true인 경우 최초 RunTask에서 Prepare를 먼저 실행합니다.
  실기 설치 템플릿과 sim 설정에서 활성화하며, 기존 설정에서 생략하면 이전 동작을 유지합니다.
* task → PlanMotion(Prepare) → ExecuteMotion → arm/hand BridgeMotion → 장치 경로를 사용합니다.
  준비 단계만 물체/라벨 관측을 요구하지 않습니다. 관절 측정, 설정 버전, 충돌/관절/속도/
  가속도, 실행 추종, 취소 및 장치 상태 검사는 유지합니다.
* home_joints는 28관절 준비 목표입니다. 순서는 왼팔/왼손/오른팔/오른손 각7개입니다.
  영상에서 사용한 정확한 rad 값은 src/deux_task_sim/deux_task_sim/ready_pose.py에 명시했습니다.
* prepare_joint_waypoints는 선택적인 28관절 중간 자세입니다. sim에서는 양쪽 어깨1을 먼저
  올린 다음 팔을 목표로 이동하고, 마지막으로 손을 맞춥니다. 실제 경로는 Hermite 시간 보간을
  사용하며 모든 샘플의 기존 계획 검사를 수행합니다.
* 성공한 측정 실행 후에만 task_node의 prepared 플래그가 설정됩니다.
  같은 프로세스의 다음 작업에는 Prepare가 없습니다. 취소/실패/dry-run은 완료로 표시하지
  않습니다. 플래그는 디스크에 저장하지 않아 task_node 재시작 후에는 다시 준비합니다.
  이미 준비된 뒤 다른 태스크 단계가 실패했다고 해서 준비 플래그를 되돌리지는 않습니다.
* 실기에서는 현재 측정값에서 준비 경로를 시작합니다. 관절0으로 강제 이동/초기화하지 않습니다.
  준비 값과 중간 자세는 실측 설치에 맞춰 별도로 정해야 합니다.

사용자가 지시한 sim 배치 변경:

* 기존 관절0 자세는 양손이 컨베이어에 약5.6cm 겹쳤습니다. 그 상태의 물리 시험은 불안정으로
  실패했고, 기록은 validation/ros2-sim-prepare/에 남겨두었습니다.
* 컨베이어/상자를 옮기지 않고 로봇 base_link만 world X 방향으로20cm 뒤로 이동합니다.
  world 위치는 [-0.848, 0, -0.75]에서 [-1.048, 0, -0.75]로 변경됩니다.
  sim 모델의 배치 변경이며 원본 DEUX/MuJoCo 파일이나 실기 URDF는 수정하지 않습니다.
* 관절0→준비 경로의151개 기하 샘플에서 로봇과 외부 물체 사이1mm 초과 침투가 없는
  배치/중간 자세를 선택했습니다. 최종 물리 실행에서도 제어 스텝마다 접촉 깊이를 기록합니다.
  이는 전체 실기 충돌 안전 검증이나 연속 시간의 완전한 충돌 증명은 아닙니다.
* 초기 관절0은 손 관절 제한의 경계입니다. MuJoCo soft limit의 작은 수치 초과를 위해
  sim의 robot.joint_limit_tolerance와 측정 손 매핑 여유를1e-3rad로 설정했습니다.
  측정값을 클리핑하지 않으며 실기 기본 허용 오차는 변경하지 않습니다.
* 기존 fixed-reader sim command가 왼손 목표를 강제로 덮어쓰던 부분은 ROS driver에서
  양손 명령값을 적용하도록 처리했습니다. 준비 중에도 실제 손 위치제어로 이동합니다.

준비 동작 단독 검증 명령::

  ros2 run deux_task_sim validate_transfer --sim-repository /absolute/DEUX_mujoco \
    --case preparation --record --output /absolute/preparation-recording
  ros2 run deux_task_sim render_timeline --output /absolute/preparation-recording

preparation 케이스는 task_node의 prepare_once를 executor에서 호출하고 그 내부 ROS
계획/실행/장치 Action 경로를 검증합니다. prepare_once를 다시 호출해 계획 요청이 한 번뿐인지
확인한 뒤3초간 측정된 준비 자세 유지를 검사합니다. RunTask 전체 이송 성공 시험은 아닙니다.
이 케이스에서 /task/run Action을 호출한 것처럼 기록하지 않습니다.

검증 결과: Jazzy8개 패키지 빌드,56개 회귀 테스트 통과.
Prepare는 ROS 시각0.102s에 시작해9.146s에 Ready로 전환했습니다.
재호출 후에도 계획1회이며3초 유지 및 후속1초 관찰에서 최대 관절 오차0.000463rad,
제어 스텝에서 관측된 로봇–환경 침투0m, 물리 경고0을 확인했습니다.
영상/메시지 화면: validation/ros2-sim-ready/preparation-recording/dashboard.html.
이전 원위치 관절0 물리 실패, 격리 장면 진단, 후퇴 거리별 기하 검사는
validation/ros2-sim-prepare/ 및 validation/ros2-sim-prepare-debug/에 보존했습니다.
최종 영상은 컨베이어를 제거하거나 접촉을 끈 장면이 아닙니다.

추가 RunTask 연결 시험에서는 Prepare1회 후 Approach/Descend/Close/Lift까지 진행했고
AlignBarcode에서 barcode moved; replan으로 중단했습니다. 이동한 배치에서 기존 바코드
판독/적재 성공을 재확인한 것은 아닙니다. 준비 경로의 침투0m 수치를 전체 작업 구간으로
확대해 해석하지 않습니다. 전체 작업의 접촉 통계는 봉투 파지 접촉도 포함합니다.
기록: validation/ros2-sim-ready/transfer-with-prepare/.


EE 위치 유지와 팔꿈치 후퇴 준비 자세 — 이전 해석 (2026-10-25)
-----------------------------------------------

사용자 요청에 따라20cm 후퇴한 로봇 배치는 유지하되 양쪽 EE의 준비 위치를 보정했습니다.
비교 기준은 로봇 후퇴 전 ros2-sim-subscriptions 녹화의 준비 자세이며 월드 좌표입니다.
후퇴 직후 EE도20cm 뒤로 이동했던 ros2-sim-ready 자세를 기준으로 한 비교가 아닙니다.

* ready_pose.py의 REFERENCE_HOME_JOINTS는 이전 관절값을 보존하고 HOME_JOINTS는 새 값입니다.
  관절 제한 안에서 EE XYZ/방향을 유지하면서 팔꿈치 X를 뒤로 보내도록 offline IK로 정했습니다.
  준비 때마다 IK를 다시 풀지 않고 검증된 목표값을 사용합니다.
* 팔꿈치는 arm_joint4 회전 중심(left_arm_4/right_arm_4)으로 비교합니다.
  EE는 기존 left_grasp/right_grasp 프레임입니다.
* 시작 경로는 어깨 올리기 → 이전 팔 준비 자세(손0) → 새 팔 준비 자세(손0) → 손 준비 자세.
  직선 관절 보간의 위험한 중간 자세를 피하기 위해 기존 경유 자세를 보존했습니다.
  기존 Hermite 시간 보간, 관절/속도/가속도·충돌 검사와 최초1회 실행 정책은 유지합니다.
* 준비 목표와 scan/retreat 설정의 EE 목표도 같은 새 FK로 맞췄습니다.
  컨베이어/상자/로봇 베이스 배치는 직전20cm 후퇴 장면과 같습니다.

검증된 준비 경로489개 샘플(9.76초)의 기하 검사에서 로봇–환경 침투가 없었습니다.
ROS/MuJoCo 실제 준비 실행, 재호출 생략 및3초 유지 결과:

* 왼쪽 EE XY 오차0.190mm, Z 변화-0.447mm, 방향 오차 약0.125도.
* 오른쪽 EE XY 오차0.234mm, Z 변화+0.444mm, 방향 오차 약0.169도.
* 팔꿈치 X 후퇴: 왼쪽36.49mm, 오른쪽20.35mm.
  팔꿈치 Y 변화는 각각+31.08mm/-9.83mm, Z 변화는-50.73mm/-66.25mm입니다.
* Prepare 계획1회, 준비 성공, 제어 스텝에서 관측된 로봇–환경 침투0m.
* preparation 검증기는 EE XY 오차5mm 이내 및 양 팔꿈치10mm 이상 후퇴도 확인합니다.
  실제 좌표와 오차는 result.json의 ready_pose_comparison에 보존됩니다.
* 영상 상단에 EE XY 오차와 팔꿈치 후퇴량을 표시합니다.
  화면: validation/ros2-sim-ee-preserved/preparation-recording/dashboard.html.

수치는 sim의 준비 자세 시험 결과이며 실기 보정이나 전체 이송 성공을 뜻하지 않습니다.

최종 검증: Jazzy8개 패키지 빌드,56개 회귀 테스트 통과.
새 영상의 재생·시간 이동·노드 필터·메시지 상세 브라우저 검사 통과.


차렷 자세 XY 기준 정정 (2026-10-25)
----------------------------------

사용자가 '차렷자세의 EE xy 좌표'를 유지하는 뜻이라고 정정했습니다.
직전 절의 준비 자세 기준과 다른 목표이며 아래 결과가 현재 구현을 설명합니다.

* 기준: URDF28관절0, 로봇이20cm 후퇴하기 전의 월드 좌표.
  ready_pose.py의 ATTENTION_JOINTS=[0.0]*28로 명시합니다.
* 기준 EE XY: left_grasp [-0.650000,+0.088500]m,
  right_grasp [-0.650002,-0.088500]m. 이전 준비 자세의 [-0.459,±0.350]m가 아닙니다.
* 로봇 base_link20cm 후퇴, 컨베이어/상자 고정은 유지합니다.
  양팔의 어깨1과 팔꿈치4만 사용해 시상면에서 굽힙니다. 다른 팔 관절은0입니다.
  손은 기존 리더/준비 손 목표를 유지합니다. 좌우 팔의 부호는 URDF 축에 따릅니다.
* HOME_JOINTS 목표: 왼쪽 어깨1=0.55019351rad, 팔꿈치4=1.63722607rad,
  오른쪽 어깨1=-0.55019912rad, 팔꿈치4=-1.63723375rad.
* XY 유지 조건에 맞춰 EE Z는 높입니다. 차렷 EE Z=-0.142187m에서 약+0.100m로 올라갑니다.
  EE 방향도 약62.3도 바뀝니다. 이전 방향을 유지하도록 제한하지 않았습니다.
* 직접 굽히면 중간 손가락이 컨베이어를 통과하므로 먼저 어깨를±0.85rad 올린 뒤
  팔꿈치를 굽히고 최종 어깨 자세로 옮깁니다. 마지막으로 손 목표를 적용합니다.
  406개 계획 샘플(8.10초)의 환경 접촉 검사에서 침투가 없었습니다.
* Prepare 최초1회, 성공 후 재호출 생략,3초 유지 검사와 기존 ROS 실행/감시는 유지합니다.

물리 sim 결과:

* 왼쪽 EE XY 오차0.196mm, 오른쪽0.133mm.
* 왼쪽 EE 높이 변화+241.87mm, 오른쪽+242.26mm.
* 팔꿈치 X 후퇴: 왼쪽336.93mm, 오른쪽336.88mm.
  팔꿈치 Y 변화는0.02mm 이내이며 Z는 약38.7mm 올라갑니다.
* Prepare 계획1회, 성공 및3초 유지. 물리 경고0, 제어 스텝의 로봇–환경 침투0m.
* 실제 좌표/방향은 result.json의 ready_pose_comparison에 저장합니다.
  화면에도 '차렷 자세(관절0) 기준'과 높이 변화를 명시합니다.
* 새 결과: validation/ros2-sim-attention-xy/preparation-recording/.
  ros2-sim-ee-preserved는 잘못 해석했던 준비 자세 기준 결과로 보존합니다.

이 검증은 요청된 준비 자세와 이동 경로에 대한 sim 결과입니다.
실기나 전체 물체 이송 성공을 검증한 것으로 해석하지 않습니다.

최종 빌드8개 패키지, 회귀56개 테스트 및 새 녹화 화면 브라우저 검사 통과.

준비 동작0~4초 동시 수행 (2026-10-25)
-----------------------------------

위 순차8.10초 경로를 중간 경유점1개의 동시 동작으로 대체했습니다.
어깨±0.85rad, 팔꿈치±1.05rad, 손 목표55%를 거쳐 같은 HOME_JOINTS로 갑니다.
기존 Hermite 속도/가속도 제한을 그대로 적용한 계획은2.34초입니다.
샘플별 환경 접촉 검사와 ROS 브리지 경유 물리 실행을 확인했습니다.

* 영상3.468초부터 모든 관절의 목표 오차가0.01rad 이하로 유지됩니다.
* Ready 전환은 영상3.638초입니다. 영상 시작의 ROS 시각은0.034초입니다.
* 영상3.978초의 EE XY 오차: 왼쪽0.207mm, 오른쪽0.143mm.
* Prepare1회, 재호출 시 추가 계획 없음, 완료 후3초 유지 성공.
* 측정된 환경 침투0m, 물리 경고0. 연속시간 전체 충돌 증명은 아닙니다.
* 첫 시도는 시작 직후 stale 관절 상태로 정지했습니다. 감시 제한 변경 없이
  재실행 및 별도 repeat-check가 모두3.468초 안정화로 통과했습니다.
  실패 기록은 attempt-stale에 보존하며 최초 수신 지연 원인은 미해결입니다.

검증 옵션: validate --case preparation --prepare-within-seconds 4 --record.
시간 판정은 실제 관절 기록을 사용하며 MP4도 물리 시각 그대로 재생합니다.
result.json의 preparation_timing에 안정화 시각과4초 부근 실측 FK를 저장합니다.
새 녹화: validation/ros2-sim-ready-4s/preparation-recording/dashboard.html.
이번 결과는 준비 동작 sim 검증이며 실기 및 물체 이송 성공 검증이 아닙니다.

Jazzy8개 패키지 빌드 및56개 회귀 테스트 통과. 브라우저에서 영상4초 Ready,
시간 연동 그래프/메시지, 노드 필터 및 이벤트 이동을 확인했습니다.
