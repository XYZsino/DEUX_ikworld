파지 · 스캔 · 수거함 적재 실행 흐름
====================================

2026-10-27 갱신: 대시보드 2절에 DEUX 실기 연결을 단계별로 추가했다.
DEUX launch/자체 노드는 보라색, overlay는 청록색, 별도 장비 드라이버는 회색이다.
팔 Action·팔 관절 피드백·TF·state/status 및 DEUX 손 메시지 정의를 코드에서 확인했다.
손 생산자는 DEUX 레포 밖, 바코드는 새 barcode_node가 시리얼로 직접 연결하며 실기 실행 검증은 하지 않았다.
state_node의 arm_topic 기본값은 명령 소유권 검사와 충돌하므로 자동 운용 시 발행 경로를 격리해야 한다.
근거: validation/pick-scan-place-flow/deux-sources.json.

아래 BT/영상 설명은 2026-10-26 적재 구현 전 스냅샷이다.
현재 적재는 ClearReader → PlaceApproach → Place → Release → PlaceRetreat → Retreat이며 VerifyPlace는 제거됐다.
최신 구현은 head_placement.rst 참조. 이 보고서의 과거 영상/로그는 변경하지 않았다.

기준: 2026-10-26 작업 트리. 사용자는 기존 수거함에 한 개씩 내려놓기를 선택했다.
이번 산출물은 코드 조사와 실행 기록 기반 설계 정리이며 런타임 적재 구현은 변경하지 않았다.
현재 head_task_node.py의 HeadRgbdPickScan은 스캔 판독 후 종료한다. 적재는 아래 추가 연결 제안이다.
기존 task_node.py Transfer의 PlaceAndVerify 구현과 현재 HeadRgbdPickScan을 혼동하지 않는다.

보고서: file:///home/xyz-ai/git/DEUX_ikworld/validation/pick-scan-place-flow/dashboard.html
Archify: file:///home/xyz-ai/git/DEUX_ikworld/validation/pick-scan-place-flow/behavior-tree.html

실행 원칙
---------

* task_node는 BT 순서·판단을 맡고, motion_planner_node는 MotionPlan, execution_node는 실행·중단 전파를 맡는다.
* execution_node 내부 ArmAdapter/HandAdapter가 드라이버 인터페이스 및 피드백 매핑을 담당한다.
* 각 동작은 PlanMotion 응답 후 ExecuteMotion으로 진행하며 execution 내부 팔·손 처리는 같은 계획의 시작 시각을 사용한다.
* 정적 설치 설정은 각 노드가 파일로 읽는다. scene_node나 반복 scene 토픽은 없다. config version 불일치는 거부한다.
* 현재 ScanRetry는 TrackBarcode/AlignBarcode + WaitBarcode를 최대 3회 시도한다. 전체 파지 재시도 루프가 아니다. Retry는 명시적 Failure만 재시도하며 계획/실행 ValueError와 취소는 즉시 전파된다.
* 실패·취소 시 하위 액션을 취소한다. 자동 손 개방, 자동 다음 물체 선택, 적재 재시도는 보장하지 않는다.
* Prepare의 prepared 상태는 프로세스 메모리다. 같은 프로세스에서는 재사용되지만 재시작·실기 위치 변경 후에는 재확인이 필요하다.

BT 단계별 흐름
---------------

Prepare — 준비 · 대상 추적
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

현재 구현

프로세스의 최초 성공한 준비 동작만 실행. 손을 반닫힘으로 준비하면서 상류 물체를 미리 선택하고 locate 요청으로 전환할 수 있다.

전이 조건: 준비 궤적 완료 → prepared=True. 이후 같은 프로세스 작업은 준비 생략. 프로세스 재기동 시 초기화.

주요 인터페이스: ``/task/plan_motion``, ``/task/execute_motion``, ``/arm/deux_arm_controller/follow_joint_trajectory``, ``/hand/{left,right}/cmd``, ``/task/observation``, ``/task/perception_request``, ``/task/detections + /task/visual_candidates``, ``/task/detection_frames``, ``/task/object_detections``

TrackAndSelect — 대상 선택
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

현재 구현

Prepare 중 대상이 확보되면 빠르게 통과할 수 있는 논리 BT 단계. 실제 로그에 독립된 지속 구간이 없을 수 있다.

전이 조건: 봉투 track 3프레임 안정, acquisition_region 내부; 요청 target_id 우선.

주요 인터페이스: ``/task/observation``, ``/task/perception_request``, ``/task/detections + /task/visual_candidates``, ``/task/state_observation``

EstimatePose — ICP · 이동 예측
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

현재 구현

선택 봉투에 ICP를 수행하고 Kalman으로 위치·속도를 추정한다. 컨베이어 8cm/s는 초기 prior이며 관측으로 보정한다.

전이 조건: 필터 관측 수·3개 속도 일관성 및 예상 도착/파지 위치가 오른손 grasp_region 안.

주요 인터페이스: ``/task/observation``, ``/task/perception_request``, ``/task/detections + /task/visual_candidates``, ``/task/detection_frames``, ``/task/object_detections``, ``/task/state_observation``

Approach — 예측 위치로 접근
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

현재 구현

계획 시 최신 pose/velocity snapshot으로 미래 접근점을 계산한다. 도착 후 약 1초의 물체 이동까지 내다보고 관절/충돌 제약을 적용한다.

전이 조건: 팔·손 action 완료. 이후 최신 관측으로 하강 시점과 TrackGrasp를 결정.

주요 인터페이스: ``/task/plan_motion``, ``/task/execute_motion``, ``/arm/deux_arm_controller/follow_joint_trajectory``, ``/hand/{left,right}/cmd``, ``/task/perception_request``, ``/task/detection_frames``, ``/task/object_detections``, ``/task/state_observation``

WaitIntercept — 진입 시점 대기
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

현재 구현

고정 0.65초 Track 대기 없음. 관측/예측한 물체와 손의 상대 위치로 하강 시작 시점을 기다린다.

전이 조건: XY ≤25mm / 높이 ≤20mm 등 rendezvous 조건. 시간 초과·불확실성 초과는 실패.

주요 인터페이스: ``/task/observation``, ``/task/detection_frames``, ``/task/object_detections``, ``/task/state_observation``

TrackGrasp — 추종 하강 → 닫힘
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

현재 구현

TrackGrasp 한 번의 계획/실행 안에 Descend → Close가 연속된다. 별도 정지 Track 길이는 0. 손가림 때 이전 추정+컨베이어 속도를 제한적으로 사용한다.

전이 조건: 관측 원시각 유지, 예측 age/공분산/작업공간 제한. 손 닫힘 동작 완료 후 바로 Lift.

주요 인터페이스: ``/task/plan_motion``, ``/task/execute_motion``, ``/arm/deux_arm_controller/follow_joint_trajectory``, ``/hand/{left,right}/cmd``, ``/task/detection_frames``, ``/task/object_detections``, ``/task/state_observation``

Lift — 바로 들어올리기
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

현재 구현

파지 성공/접촉 판정 없이 닫힘 완료 직후 상승. perception을 barcode로 바꾸어 선택 ICP 작업 제출 중단.

전이 조건: 궤적 완료와 실행 안전 조건만 확인. 파지 증거는 이후 재인식에서 확인.

주요 인터페이스: ``/task/plan_motion``, ``/task/execute_motion``, ``/arm/deux_arm_controller/follow_joint_trajectory``, ``/hand/{left,right}/cmd``, ``/task/perception_request``, ``/task/visual_evidence + /task/visual_candidates``

ScanPose — 스캔 위치로 이동
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

현재 구현

오른손 손등이 위를 향하는 높은 스캔 자세로 이동. 현재 EE 목표 약 [0.675, -0.097, 1.069]m (base_link).

전이 조건: 팔/손 궤적 완료. 현재 자세는 SIM 보정값이며 실기 캘리브레이션 필요.

주요 인터페이스: ``/task/plan_motion``, ``/task/execute_motion``, ``/arm/deux_arm_controller/follow_joint_trajectory``, ``/hand/{left,right}/cmd``, ``/task/visual_evidence + /task/visual_candidates``

ReacquireHeldTarget — 손 근처 물체 재인식
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

현재 구현

추가 팔 이동 단계가 아니다. 오른손 근처의 유일한 봉투를 영상에서 다시 찾고 ID를 연결한다.

전이 조건: 이전 표면보다 4cm 이상 상승, 손 근처 조건 및 서로 다른 3프레임. 영구 동일 물체 증명은 아님.

주요 인터페이스: ``/task/perception_request``, ``/task/observation``, ``/task/visual_evidence + /task/visual_candidates``, ``/task/state_observation``

VerifyVisualPick — 영상 기반 파지 확인
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

현재 구현

봉투·바코드가 보이고 표면 중심이 상승했으며 오른손과 가까운지 확인한다.

전이 조건: 표면 상승 ≥4cm / 오른손 거리 ≤20cm / 서로 다른 3프레임.

주요 인터페이스: ``/task/observation``, ``/task/visual_evidence + /task/visual_candidates``, ``/task/state_observation``

AlignBarcode — 왼손 리더 정렬
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

현재 구현

TrackBarcode에서 최신 바코드 표면을 기다린 후 AlignBarcode. 왼팔 5~7번은 준비 각도를 유지하고 1~4번으로 조준한다.

전이 조건: 최대 3번 ScanRetry에 포함. 이번 SIM 이동 기울기 12.77°, 판독 자세 3.46°, 손목 측정 편차 ≤0.20°.

주요 인터페이스: ``/task/plan_motion``, ``/task/execute_motion``, ``/arm/deux_arm_controller/follow_joint_trajectory``, ``/hand/{left,right}/cmd``, ``/task/barcode_target``, ``/task/barcode_target``, ``/task/visual_evidence + /task/visual_candidates``

WaitBarcode — 실제 판독 대기
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

현재 구현

세션을 열어 스캐너 응답을 기다린다. 왼손이 코드를 가릴 수 있으므로 fresh 봉투 가시성과 세션 응답을 확인한다.

전이 조건: 현재: 판독 성공이면 RunTask 성공 종료. 제안: 여기서 ClearReader로 연결. 세션은 finally에서 닫음.

주요 인터페이스: ``/task/scan_session``, ``/barcode/raw``, ``/task/barcode_read``, ``/task/visual_evidence + /task/visual_candidates``, ``/task/status``

ClearReader — ① 왼손 리더 후퇴
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

추가 제안 / head BT 미연결

추가 제안. 스캔 세션을 닫고 오른손 물체를 유지한 채 왼손 리더를 검증된 대피 자세로 이동한다.

전이 조건: 리더가 오른손 적재 경로를 벗어나고 정지했는지 측정 확인. 실패하면 적재 시작 금지.

주요 인터페이스: ``/task/plan_motion``, ``/task/execute_motion``, ``/arm/deux_arm_controller/follow_joint_trajectory``, ``/hand/{left,right}/cmd``, ``/task/scan_session``, ``/task/state_observation``

Place — ② 수거함 접근 · 하강
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

추가 제안 / head BT 미연결

기존 Place 계획기는 있지만 head BT 연결은 없음. 함 위 경유점 → 개방 높이까지 하강하는 두 구간을 추가하고 물체를 잡은 상태로 이동한다.

전이 조건: 기존 Place는 최신 held-object pose 필요. 이번 단일 함 용도에는 보정된 EE 경유/개방 pose 방식 우선 제안. 파지 전 ICP 재사용 금지.

주요 인터페이스: ``/task/plan_motion``, ``/task/execute_motion``, ``/arm/deux_arm_controller/follow_joint_trajectory``, ``/hand/{left,right}/cmd``, ``/task/state_observation``

Release — ③ 수거함 안에서 손 열기
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

추가 제안 / head BT 미연결

기존 Release 계획을 head BT에 연결. 오른손 위치와 왼손 대피 자세를 유지하며 오른손만 연다.

전이 조건: 팔이 함 내부의 승인된 개방 영역에 정지 + 손 개방 측정 완료. 명령 발행만으로 놓기 성공 처리하지 않음.

주요 인터페이스: ``/task/plan_motion``, ``/task/execute_motion``, ``/arm/deux_arm_controller/follow_joint_trajectory``, ``/hand/{left,right}/cmd``, ``/task/state_observation``

Retreat — ④ 물체에서 손 후퇴
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

추가 제안 / head BT 미연결

기존 Retreat는 home_joints 복귀이며 손 목표를 현재값으로 유지한다. 함에서 먼저 위로 빠져나오는 구간을 추가해야 한다.

전이 조건: 개방 상태로 충분히 분리 후 준비 대기 위치. 다음 물체용 반닫힘은 함/봉투와 분리된 뒤에만.

주요 인터페이스: ``/task/plan_motion``, ``/task/execute_motion``, ``/arm/deux_arm_controller/follow_joint_trajectory``, ``/hand/{left,right}/cmd``, ``/task/state_observation``

VerifyPlace — ⑤ 적재 · 손 분리 확인
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

추가 제안 / head BT 미연결

새 head BT용 검증 필요. Release 이후 새 영상에서 수거함 안의 물체와 손 분리를 확인한다. 기존 TaskNode 검증은 fresh 물체 pose에 의존한다.

전이 조건: 제안: 목적지 ROI의 새 물체 변화 + 손과 분리의 복수 프레임 증거. 가림/겹침으로 확인 불가면 성공을 선언하지 않고 확인 필요 상태로 종료.

주요 인터페이스: ``/task/perception_request``, ``/task/observation``, ``/task/visual_evidence + /task/visual_candidates``, ``/task/state_observation``, ``/task/status``

Succeeded — ⑥ 한 개 적재 완료
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

추가 제안 / head BT 미연결

제안: 바코드 판독 + 놓기/후퇴 + 적재 확인 모두 통과한 시점에 RunTask 결과를 반환한다.

전이 조건: 다음 물체는 새 RunTask. 자동 연속 루프/함 용량 관리/바코드별 분류 적재는 이번 범위 밖.

주요 인터페이스: ``/task/status``, ``/task/run``

적재 구현 시 우선 해결할 점
-------------------------

1. HeadRgbdPickScan의 성공 종료를 WaitBarcode 이후에서 VerifyPlace 이후로 옮기고, workflow/GUI 단계 목록도 함께 확장한다.
2. ClearReader를 먼저 실행한다. 기존 Place는 양팔 목표를 동시에 계산하므로 리더 대피가 선행된다고 보장하지 않는다.
3. 수거함 상부 경유와 낮춤, Release, 수직 후퇴를 별도 조건으로 분리한다. 기존 Place/Retreat 직결만으로 수거함 벽 회피를 보장할 수 없다.
4. 기존 Place는 object_from_ee = inverse(current_object_pose) @ measured_right_ee를 사용한다. 파지 전 평면 제한 ICP 또는 단순 surface_center는 이 입력을 대체할 수 없다.
   한 개씩 동일 수거함에 넣는 이번 범위는 보정된 EE 경유/개방 자세 + 보수적 물체 부피로 계획하는 방식을 우선 제안한다. 자세/높이·그립 변화의 허용 범위를 실측해야 한다.
   물체 중심 위치 정밀도가 필요해지면 손에 든 물체의 fresh 6D pose/손-물체 변환 추정과 그 신뢰도 검증이 추가로 필요하다.
5. VerifyPlace는 Release 이후의 새 영상/센서 증거를 요구한다. 목적지 ROI 내부 관측·손 분리·투입 전후 변화로 확인하고, 기존 물체/가림과 구분이 안 되면 확인 불가로 처리한다.
6. Release 전 실패는 물체를 잡은 상태로 정지하고 재개/회수 정책을 결정한다. Release 후에는 무조건 재시작하여 중복 투입하지 않도록 작업 결과를 보존하는 설계가 필요하다.
7. 실행 중인 target ID와 판독 barcode를 작업 단위로 보존한다. 재인식 ID 교체를 영구 물체 동일성 증명으로 해석하지 않는다.

현재 SIM 수거함 설정
---------------------

head_calibration.yaml의 destination_pose 위치는 base_link 기준 [0.208, -0.560, 0.655] m.
destination_bounds는 min [0.073, -0.695, 0.505], max [0.343, -0.425, 0.715] m이다.
이는 기존 물체 목표/판정 영역이며 손 개방 pose나 실측 실기 수거함 좌표가 아니다. 실기에서 그대로 사용하지 않는다.

ROS 인터페이스 계약
--------------------

* ``/task/run`` — deux_task_msgs/action/RunTask
  client → task: goal: target_id. feedback: state/attempts. result: success/reason/barcode. 취소는 ROS action cancel.

* ``/task/status`` — deux_task_msgs/msg/TaskStatus
  task → client: state, target_id, session_id, reason, workflow, active. GUI 표시; 제어 명령 아님.

* ``/task/observation`` — deux_task_msgs/msg/Observation
  driver → detect: 렌더 RGB-D / intrinsics / 취득 시각. REAL에서는 ZED 드라이버 → observation_node가 생성.

* ``/task/perception_request`` — deux_task_msgs/msg/PerceptionRequest
  task → detect: mode: detect / locate / barcode, target_id, request_id, expires_at. 동적 처리 요청; 정적 config와 별개.

* ``/task/detections + /task/visual_candidates`` — DetectionArray + VisualEvidenceArray
  detect → task: 영상 track ID, 봉투/바코드 가시성, 관측 표면 중심. 후보 3프레임 안정성·오른손 작업 영역 검사.

* ``/task/detection_frames`` — deux_task_msgs/msg/DetectionFrame
  detect → icp: locate 요청에서 선택 봉투의 mask/depth/track ID 전달. barcode 모드에서는 ICP 작업 중단.

* ``/task/object_detections`` — deux_task_msgs/msg/ObjectDetectionArray
  icp → state: ICP + Kalman pose, velocity, motion_covariance, filter_observations, 원 취득 시각. 급격한 innovation 제외.

* ``/task/state_observation`` — deux_task_msgs/msg/StateObservation
  state → task: 팔·손 측정값, TF 기반 EE, 물체 관측 및 valid flags/시각. BT 조건 판단.

* ``/task/state_observation`` — deux_task_msgs/msg/StateObservation
  state → plan: 계획 승인 시 측정 상태·관측 유효성 검사. PlanMotion goal에도 관측 snapshot 포함.

* ``/task/state_observation`` — deux_task_msgs/msg/StateObservation
  state → exec: 실행 중 현재 관절·EE·물체 상태 감시. 목표값을 측정값으로 대신하지 않음.

* ``/task/visual_evidence + /task/visual_candidates`` — VisualEvidence + VisualEvidenceArray
  detect → task: 봉투 표면, 손과 거리, 바코드 가시성. 파지 후 재인식/상승 확인. full held-object 6D pose 아님.

* ``/task/barcode_target`` — deux_task_msgs/msg/BarcodeTarget
  detect → task: 선택 물체의 바코드 표면 pose, normal, barcode_track_id, request_id, 취득 시각.

* ``/task/barcode_target`` — deux_task_msgs/msg/BarcodeTarget
  detect → exec: AlignBarcode 실행 감시용 최신 표면. planner에는 task가 action goal로 전달.

* ``/task/plan_motion`` — deux_task_msgs/action/PlanMotion
  task → plan: goal: stage/target_id/scene_version/observation/barcode_target → result: success/reason/MotionPlan.

* ``/task/execute_motion`` — deux_task_msgs/action/ExecuteMotion
  task → exec: goal: MotionPlan (trajectory, phase end times, 예측/유효기간) → feedback/progress → result.

* ``/arm/deux_arm_controller/follow_joint_trajectory`` — control_msgs/action/FollowJointTrajectory
  exec → driver: 팔 드라이버 궤적 실행. 결과·실측 허용오차 확인.

* ``/hand/{left,right}/cmd`` — deux_hand_msgs/msg/HandCtlMessage
  exec → driver: 손 매핑을 적용한 모터 명령. 손을 여는 동작은 Release에서만 명시적으로 수행.

* ``/arm/joint_states`` — sensor_msgs/msg/JointState
  driver → state: 팔 측정 관절. execution_node도 같은 topic을 구독한다(도식에는 대표 피드백 경로).

* ``/sim/hand/{left,right}/joint_states`` — sensor_msgs/msg/JointState
  driver → exec: SIM raw 손 피드백. REAL은 hand_mapping 파일의 feedback_topic / feedback_type.

* ``/task/hand_state`` — sensor_msgs/msg/JointState
  exec → state: 실제 드라이버 피드백을 공통 손 관절 이름·단위로 변환.

* ``/task/scan_session`` — deux_task_msgs/msg/ScanSession
  task → barcode: session_id, track_id, active, expires_at. 시작/종료 lease. SIM driver도 구독.

* ``/barcode/raw`` — std_msgs/msg/String
  driver → barcode: SIM: 정렬·시야 기반 가상 리더 fixture EAN. REAL: barcode_node의 직접 serial 입력. 이 String 경로는 SIM 전용.

* ``/task/barcode_read`` — deux_task_msgs/msg/BarcodeRead
  barcode → task: 유효 scan session에 귀속한 판독값; 늦은 응답/다른 세션은 채택하지 않음.

항상 사용하는 기반 신호
------------------------

* /arm/joint_states → state_observation_node, execution_node.
* raw hand feedback → execution_node → /task/hand_state → state_observation_node.
* /task/state_observation → task_node, motion_planner_node, execution_node.
* /tf, /tf_static: 좌표 변환. /state/status: execution hardware gate. 이 신호들은 이번 기록기에서 모두 기록하지 않았다.
* /task/execution_record: execution_node의 실행 감사용 String. GUI를 사용하는 배포에서는 /task/status를 GUI가 구독한다.
* SIM action 호출자는 scenario_client였다. GUI를 사용한 실제 기록이라고 해석하지 않는다.
* 도식의 화살표는 logical action/topic이며 action의 goal/result/feedback/cancel은 하나의 인터페이스로 묶었다.

실기 전환
---------

RGB-D 입력
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

SIM: mujoco_driver_node 렌더 Observation

REAL: ZED 2i driver → observation_node → /task/observation

검증: RGB/depth 동기화·단위·intrinsics·카메라 TF를 실측. 현재 45°/3mm 조건의 절대 장착 위치는 SIM 추정.

팔 제어
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

SIM: MuJoCo FollowJointTrajectory 서버

REAL: 실제 팔 controller; execution 내부 ArmAdapter 유지

검증: joint 순서/방향/단위, feedback 시간, tracking tolerance, cancel/hold 검증.

손 제어
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

SIM: SIM raw joint states + 물리 모터

REAL: 실제 hand driver + 실측 hand_mapping; execution 내부 HandAdapter 유지

검증: 부하에서 grip/개방/미끄러짐 검증. SIM Kp 4배는 실제 잡는 힘 검증이 아님.

스캐너
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

SIM: 가상 fixture EAN 응답

REAL: barcode_node → Newland serial 직접 연결 → /task/barcode_read

검증: 원문 형식·타임스탬프·지연·중복/만료 세션 검증. 현재 성공은 영상 디코딩 증거가 아님.

좌표·설비
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

SIM: SIM head_calibration / destination_bounds

REAL: 실측 base/EE/reader/수거함·컨베이어 TF + config

검증: 수거함 입구·높이·벽·여유공간 및 두 팔/리더/봉투 충돌 모델 추가.

인식·추종
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

SIM: SIM 학습 모델 + ICP/Kalman

REAL: 같은 노드, 실제 카메라 데이터로 보정

검증: 실제 봉투 외형/깊이 결손/손가림에서 8cm/s 추정, outlier·재인식 오인 검증.

시간·상태
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

SIM: /clock 및 SIM 상태

REAL: use_sim_time=false; 실제 /state/status 발행

검증: 각 노드 같은 설치 버전, 하드웨어 enable gate, 시계/센서 지연·stale 차단 확인.

적재 완료 증거
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

SIM: 현재 head BT 없음

REAL: head BT 확장 + 목적지 관측/필요시 별도 센서

검증: 한 개 적재부터 검증, 연속 투입·가림·함 포화는 별도 시험.

실행 증거와 한계
----------------

fixed-wrist-scan/run-01의 기존 ROS 2 / MuJoCo 영상·ros-events.json을 재사용했다. 새 적재 SIM을 실행한 결과가 아니다.
기록 result.json의 단계 시간: AlignBarcode 20.604s → WaitBarcode 23.392s → Succeeded 23.698s.
대시보드의 로그/영상 축은 기록의 원 time_s(sim clock)를 그대로 사용한다. result.json 단계 시간은 ROS 상태 기록과 일치하고, trace.json 물리 샘플은 약 0.034s 늦게 상태를 포착할 수 있다.
빠른 BT 단계 TrackAndSelect/TrackBarcode는 코드상 존재하나 타이머 샘플에서 별도 구간으로 나타나지 않을 수 있다.
가상 바코드 값 2087231250396, 물리 경고 0, 설비 침투 0, 물체 접촉 침투 약 23.12mm, weld 0.
따라서 실기 파지력·실제 바코드 디코딩·물리 접촉 품질·적재 성공·반복 성공률은 검증하지 않았다.
대시보드에서 주황색 적재 단계에는 영상/로그/예상 소요시간을 만들어 붙이지 않았다.
소스 목록·SHA256: validation/pick-scan-place-flow/sources.json. 보고서 재생성: python validation/pick-scan-place-flow/build_report.py.
