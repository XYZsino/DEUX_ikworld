실행 노드 통합 및 영상 Observation 정리
======================================

2026-10-27. 팔·손 브리지 통합과 영상 상태 중복 제거를 구현했다.
barcode_bridge, BT, 플래너, 검출/ICP 및 StateObservation 집계는 분리 유지한다.
DEUX 원본과 하드웨어 드라이버는 수정하지 않았다.

실행 경계
---------

기존 execution → BridgeMotion → arm/hand bridge 경로를 제거했다.
execution_node 안의 ArmAdapter와 HandAdapter는 일반 Python 객체이며 별도 ROS 노드가 아니다.

* task_node → /task/execute_motion (ExecuteMotion) → execution_node.
* execution의 ArmAdapter → /arm/deux_arm_controller/follow_joint_trajectory
  (control_msgs/action/FollowJointTrajectory) → 팔 드라이버.
* execution의 HandAdapter → /hand/left/cmd, /hand/right/cmd
  (deux_hand_msgs/msg/HandCtlMessage) → 손 드라이버.
* /arm/joint_states 및 매핑 파일의 raw 손 피드백을 execution이 직접 수신한다.
* /task/hand_state (JointState)는 execution이 발행하며 StateObservation 집계에 사용한다.
* /state/status는 execution에서 한 번 구독하고 실행 허용 조건을 검사한다.

팔·손 시작 시각/추종 오차/최종 도착을 하나의 실행 루프에서 검사한다.
추종 검사에는 원시 팔·손 측정을 사용하므로 StateObservation 왕복을 기다리지 않는다.
물체 예측/바코드/EE 등 의미적 실행 조건은 기존 StateObservation과 BarcodeTarget을 사용한다.
계획 시 한 번, 실제 실행 시 한 번 검사하는 관절·충돌 조건은 유지한다.

ArmAdapter는 팔 Action의 승인/결과/취소를 처리한다. 늦게 승인된 goal도 취소하며,
이전 승인/취소가 미완료인 동안 새 팔 궤적 발행을 허용하지 않는다.
HandAdapter는 ID·부호·offset·위치모드·모터 상태·실측 시각을 검사한다.
손 명령은 팔 승인 이후 공통 시작 시각부터 보낸다. 손 완료 후에는 기존 파지 preload를 유지한다.
진행 중 실패/취소 시 유효한 손 측정이 있으면 측정 위치 유지 명령을 보낸다.
피드백이 없으면 임의 위치 명령을 만들지 않으며 장비 watchdog이 필요하다.
팔·손의 하드웨어 차원 동시 정지 또는 Python 프로세스 장애 시의 정지를 보장한다는 뜻은 아니다.

제거한 인터페이스/패키지
-----------------------

* /task/arm_motion, /task/hand_motion 및 BridgeMotion.action.
* arm_bridge_node, hand_bridge_node 실행 파일 및 deux_task_adapters 패키지.
* 실행 파일이 독립 노드였던 두 브리지는 runtime의 arm_adapter.py, hand_adapter.py로 이동.
* launch와 SIM stack, Action 기록기가 실행 노드 직접 연결을 사용하도록 갱신됨.

execution_node 파라미터는 config_file, dry_run에 더해 hardware_enabled,
mapping_file, driver_action을 받는다. 기본 dry_run=true, hardware_enabled=false 유지.
head_rgbd.launch.py와 task.launch.py의 기존 hand_mapping/hardware_enabled 인자는
이제 execution_node에 전달된다. 설치 YAML은 task/planner/execution이 각자 읽는다.

영상과 제어 상태
----------------

Observation.msg는 RGB, depth, CameraInfo 두 개, 원 촬영 header/sequence,
valid_depth_fraction, camera_transform_valid, base_from_camera만 포함한다.
영상 observation_node의 팔/손/물체 구독·히스토리·선택적 재필터링·EE 조회를 제거했다.
촬영 시각의 카메라 TF 조회와 RGB-D 동기화/검증은 유지한다.

StateObservation.msg는 변경하지 않았다. 현재 관절·EE·ICP/Kalman 집계는
state_observation_node에 남으며 영상이 없어도 제어 측정이 발행된다.
ICP의 이동 추정 필터는 유지했고 영상 Observation의 별도 filter_objects 옵션만 제거했다.
GUI RGB 3D 범위는 카메라 TF/동기 depth/선택 검출을 계속 사용한다.
legacy JSON adapter는 StateObservation 입력을 사용한다.

설치 전환
---------

메시지 ABI가 바뀌었으므로 기존/신규 Observation 생산자·소비자를 같은 ROS 도메인에
혼용하지 않는다. 모든 overlay 패키지와 외부 소비자를 함께 재빌드하고 재시작한다.
기존 설치 폴더에는 제거된 패키지/메시지가 남을 수 있으므로 새 build/install 경로를 사용한다.
예: 기존 DEUX underlay를 source한 새 셸에서::

  colcon build --build-base build-integrated --install-base install-integrated
  source install-integrated/setup.bash

검증은 /tmp/integrated-build, /tmp/integrated-install과 별도 ROS_DOMAIN_ID=231을 사용했다.
현재 8090 GUI/SIM은 이 작업에서 재시작하지 않았다. 이전 메시지 정의의 프로세스이다.
실기 손 mapping/드라이버 확인 및 command ownership 조건은 여전히 필요하다.
DEUX state_node의 arm_topic 기본 publisher는 execution의 소유권 검사와 충돌하므로
실기 자동 운용에서는 해당 명령 경로 격리가 필요하다.

검증 산출물
-----------

validation/head-rgbd/execution-integration/:

* build.log: ROS 2 Jazzy, 신규 overlay 7패키지 빌드.
* tests.log: 전체193개 회귀 및 통합 실행 예외 사례 통과.
* scenario/result.json, ros-events.json: 첫 통합 MuJoCo 흐름.
* scenario-final/result.json, ros-events.json: 최종 변경 후 MuJoCo 흐름.
* sim.log, sim-final.log: 실행 로그.

테스트는 시작 동기화, 정상 완료, 파지 preload 유지, 시작 전/진행 중 취소,
원시 피드백 소실, 손만 피드백 소실, 팔 승인 거절/실행 실패, 늦은 승인 취소,
이전 승인/취소 중 재발행 차단을 포함한다. Observation 메시지에서 중복 필드와
세 상태 구독이 사라졌는지도 확인한다.
실기 검증, 실제 영상 바코드 디코딩, 반복 파지 성공률 또는 적재 확인을 뜻하지 않는다.

대시보드
--------

file:///home/xyz-ai/git/DEUX_ikworld/validation/pick-scan-place-flow/dashboard.html

2절의 논리 메시지 경로와 DEUX 연결은 통합 구조로 수정했다.
같은 페이지의 과거 BT/영상/실측 로그는 기존 스냅샷으로 보존한다.

최종 결과
---------

최종 SIM은41.038초에 Succeeded. 준비→파지→상승→스캔→수거함 개방·후퇴를 완료했다.
8개 런타임 노드(driver 포함),4개 Action client 계약을 실제 ROS 그래프에서 확인했다.
브리지 노드/두 내부 Action 없음. 손 명령·hand_state의 발행자는 execution_node이다.
물리 경고0, 설비 침투0, 강제 weld0. 물체 접촉 침투약23.08mm는 남아 있다.
가상 리더의 fixture EAN 판독이며 실제 디코딩/적재 확인/실기 성공을 검증한 것은 아니다.
요약 및 소스 해시: validation/head-rgbd/execution-integration/verification.json.
