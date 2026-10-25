.. note:: 최신 권장 경로는 controlled_perception.rst를 참고하십시오. 아래 분리 인식 및 고정 Scan 설명은 이전 구현 설명입니다.

설계의 나머지 ROS 2 노드 구현 — 2026-10-24
=========================================

구현 범위
---------

기존 observation/detection/ICP에 아래6개 노드를 추가했습니다. scene_node는 제거했으며 현재9개 노드이고
독립 overlay6개 패키지입니다. 검증 컨테이너는 기존 deux_hand_msgs를 읽기 전용으로
가져와 함께 빌드하므로 빌드 로그에는7개 패키지가 나옵니다.

* barcode_bridge_node (deux_task_perception): std_msgs/String 리더 입력을
  BarcodeRead로 변환. 활성 ScanSession의 ID/대상/기한을 붙이며 중복 문자열 제거.
* task_node: /task/run RunTask Action. 접근→하강→닫기→상승 검증→스캔→적재→
  놓기→후퇴→분리 검증. 파지 최대2회, 바코드 최대3회. 취소 시 하위 Action 취소.
* motion_planner_node: /task/plan_motion PlanMotion Action. 별도 프로세스의
  URDF FK/IK, 물체 기준 파지 목표, Cartesian 접근/하강/상승, 라벨 기준 스캔 목표,
  관절 Hermite 경로, 속도/가속도/충돌 여유 검사. scene/대상/시작 자세 재검증.
* execution_node: /task/execute_motion ExecuteMotion Action. 승인 궤적 재검사,
  공통 시작 시각으로 두 bridge에 전송. 만료·중복·scene 변경·대상 이동·추종 오차·
  피드백 단절 감시. dry_run 기본값true, 실제 명령 없이 검사 결과만 반환.
* arm_bridge_node (deux_task_adapters): /task/arm_motion BridgeMotion Action →
  기존 FollowJointTrajectory Action. 양팔14관절을 한 goal로 전달. 수락/결과/취소,
  URDF 관절/속도/가속도 한계와 실제 측정 상태 검사. hardware_enabled 기본false.
* hand_bridge_node: /task/hand_motion BridgeMotion Action → 기존 좌우
  HandCtlMessage. 명시된 ID·부호·영점·rad 범위·속도·전류/프로토콜 설정 사용.
  JointState 또는 HandCtlMessage 측정 피드백만 /task/hand_state로 정규화.
  실제 측정 시각을 보존하고, 모터 에러/누락/비유한 값은 거부.

observation_node에는 선택적 등속도 XY/yaw Kalman 필터를 추가했습니다.
filter_objects 기본false이며, 활성화하면 원본 촬영 시각당 한 번만 갱신합니다.
pose_filtered 표시를 통해 이중 필터링을 피하고 ROS clock 역행 시 초기화합니다.
필터링된 공분산을 ICP 기반 통계 추정치로 주장하지 않습니다.

설정 파일 직접 로드
-------------------

task_node, motion_planner_node, execution_node, arm_bridge_node, hand_bridge_node는
각자 시작 시 ``config_file`` 파라미터의 YAML을 직접 읽고 설정과 URDF를 검증합니다.
``scene_node``, ``/task/scene``, ``/task/reload_scene``, ``Scene.msg``는 제거했습니다.
설정 파일이 없거나 잘못됐으면 해당 노드는 시작을 거부합니다.

launch의 ``scene_config`` 인자는 위5개 노드의 ``config_file``에 같은 경로로 전달됩니다.
URDF의 상대 경로는 YAML 파일이 있는 디렉터리를 기준으로 해석합니다.
카메라/perception 설정과 손 mapping 파일은 기존처럼 별도 설정입니다.

설정은 시작 시 한 번 로드하며 파일 변경 감시·주기적 읽기·설정 배포 토픽은 없습니다.
YAML 수정 후에는 작업을 종료하고 관련5개 노드를 함께 재시작해야 합니다.
``config_file`` 파라미터도 실행 중에는 변경할 수 없습니다.
계획기가 URDF를 다시 읽을 때는 시작 시의 해시와 비교하며, 변경됐으면 재시작을 요구합니다.

각 노드는 YAML 설정과 URDF 내용으로 동일한 버전 해시를 계산합니다. PlanMotion,
MotionPlan 및 BridgeMotion에 실린 ``scene_version``을 자신의 버전과 비교하므로,
서로 다른 설정으로 시작한 노드의 계획/명령은 거부됩니다. BridgeMotion 인터페이스가
변경되었으므로 모든 관련 패키지를 함께 재빌드해야 합니다.
여러 컴퓨터에 배치할 때는 설정과 URDF 파일을 각 노드가 읽을 수 있게 배포해야 하며,
현재 버전 계산에는 해석된 파일 경로도 포함되므로 동일한 절대 경로를 사용하십시오.

구체화한 설계와 한계
-------------------

이 구현은 이전 MuJoCo controller.tick을 실기에 그대로 실행하지 않습니다.
실기 expanded URDF의 관절/보정 grasp link를 사용하는 scipy 기반 수치 IK와
별도 Action 상태 머신입니다. sim의 monotone cubic Hermite 경유 방식과 단계 순서를
참고했으며, 기존 sim 궤적/물리 동작과 동일하다는 검증은 하지 않았습니다.

충돌 모델은 설치자가 제공한 로봇·리더의 보수적 구형 envelope, 작업공간 AABB,
오른손 grasp 원점 중심의 봉투 envelope입니다. 접촉역학/미끄럼 모델은 아닙니다.
모든 궤적 표본을 검사하지만 연속시간 swept-volume 증명은 아니므로 설치 envelope와
여유가 표본 사이 운동까지 포함해야 합니다. 현재 대상 외 물체는 설치 obstacles에
포함해야 합니다. 환경 전체 형상을 봉투 pose 하나에서 추론하지 않습니다.

접근/하강/상승은 Cartesian waypoint IK를 거치며 경로가 막히면 실패합니다.
Scan/Place 등 관절 경로는 직접 경로가 불가할 때 명시된 safe_joint_waypoint 하나를
Hermite로 연속 통과합니다. 자동 전역 경로 탐색기는 아닙니다.

파지 목표는 model 원점에 대한 보정 grasp transform입니다. 실기 접점/손가락
변형을 자동 최적화하는 기능은 포함하지 않습니다. 스캔은 보정된 앞/뒷면 라벨과
왼손 리더 transform으로 양면 후보를 계산합니다. 도달 가능한 후보를 고르지만
실제 새 판독 문자열이 없으면 스캔 성공으로 처리하지 않습니다.

Scan/Place는 들어 올린 물체의 full_orientation=true 관측을 요구합니다.
현재 yaw-only ICP만으로는 이 단계가 실행되지 않습니다. 파지 후6D 인식 또는
독립 라벨/봉투 자세 측정 생산자가 추가로 필요합니다. 손 FK로 만든 봉투 예측을
파지/적재 성공의 독립 관측으로 사용하지 않습니다.

상승 성공은 원래 높이보다 오른 물체의 서로 다른 촬영 관측2개로 확인합니다.
재파지는 신선한 관측에서 물체가 원래 지지 높이에 남은 경우만 허용합니다.
적재 성공은 목적 영역 안의 물체와 후퇴한 손의 분리 관측2개를 요구합니다.
마지막 관측만 반복해서 붙은 경우에는 새 증거로 세지 않습니다.

리더 입력이 String이면 header.stamp는 촬영/장치 판독 시각이 아닌 ROS 수신 시각입니다.
장치에서 오래 보관한 문자열까지 식별하려면 상류에서 버퍼를 비우거나 별도
장치 시각을 갖는 생산자가 필요합니다. simulated 판독은 실제 scene에서 거부합니다.

손의 실패/취소 시 신선한 측정값으로 위치 hold를 보냅니다. 측정이 끊겼다면 위치를
추측하거나 torque-off/놓기를 보내지 않고 중단합니다. 이 경우 드라이버의 watchdog과
loaded-hand hold 동작은 현장 확인 항목입니다. 손 hardware_error_status는 피드백
형식이 HandCtlMessage일 때만 직접 검사할 수 있습니다.

팔/손 Action은 공통 미래 시작 시각을 받지만 네트워크에 걸친 원자적 실행 보장은
없습니다. 수락 기한을 시작 시각보다 앞에 두고 실패 시 이미 수락된 goal과 늦게
수락된 goal도 취소합니다. 취소 확인 실패는 오류 로그로 표시합니다.

/state/status의 teleop='off'와 승인 mode, 직접 팔 궤적 publisher 유무를 검사합니다.
기존 state_node는 값이 바뀔 때만 status를 발행하므로 주기적 timestamp로 간주하지
않습니다. 이 검사는 ROS Action의 전역 소유권 lock이 아닙니다. 기존 teleop/state
명령을 launch/remapping으로 분리하고 command_ownership_confirmed를 실제 확인한
설정에서만 켜야 합니다. /state/set_mode는 호출하지 않습니다.

설정과 실행
-----------

필수 설치 템플릿:

* src/deux_task_bringup/config/installation.yaml
* src/deux_task_bringup/config/hand_mapping.yaml

빈 모델/frame/관절/hand mapping을 실제 값으로 채우기 전에는 설정/명령을 거부합니다.
템플릿의 거리·시간·오차 제한은 출발점이며 설치 검증값이 아닙니다.
URDF의 mimic joint는 현재 지원하지 않으므로 확장해 실제 측정/고정 관절로 표현해야
합니다. 알려지지 않은 movable joint를0으로 채우지 않습니다.

ROS 2 Jazzy 환경에서 기존 DEUX underlay를 source한 뒤::

  colcon build --symlink-install
  source install/setup.bash
  ros2 launch deux_task_perception observation.launch.py
  # detection/ICP는 별도 perception 설정으로 실행합니다.
  ros2 launch deux_task_bringup task.launch.py \
    scene_config:=/absolute/path/installation.yaml \
    hand_mapping:=/absolute/path/hand_mapping.yaml \
    barcode_topic:=/actual/reader/string/topic

위 기본 launch는 hardware_enabled=false, dry_run=true입니다. 드라이버, teleop,
카메라 또는 리프트를 시작하지 않습니다. 실제 실행 설정은 보정/소유권 확인과
손 프로토콜 검증 후 별도로 지정해야 합니다.

태스크 시작 API::

  ros2 action send_goal /task/run deux_task_msgs/action/RunTask "{target_id: 'actual-track-id'}" --feedback

빈 target_id는 현재 관측에서 첫 유효 대상을 선택합니다. dry_run에서는 첫 단계
계획 검증 후 "dry-run plan checked; no motion or task success claimed"로 종료합니다.
가짜 관절 피드백을 만들어 전체 물리 작업이 성공한 것처럼 진행하지 않습니다.
관측 재생 시 모든 관련 노드의 use_sim_time을 동일하게 설정하십시오.

상태/진단:

* /task/status: 현재 단계·대상·세션·재시도·IK 횟수·실패 원인.
* 각 소비 노드의 시작 로그: 로드한 설정 버전 및 설정 오류.
* /task/execution_record: dry-run 검사 기록.
* 각 Action result: 거부/만료/추종/취소/실패 원인.

legacy sim JSON adapter는 runtime/observation_adapter.py에 있습니다. v1에는 개별
물체 시각과 관측축 표현이 없어, 시간 불일치/yaw-only/이미 필터링된 물체는 거부합니다.
새 runtime은 이 제한된 adapter 대신 원래 ROS 메시지를 사용합니다.

검증과 남은 현장 작업
--------------------

최종 검증: ROS 2 Jazzy/Python3.12.3에서7개 패키지 빌드, 전체41개 테스트 통과.
검증 명령은 bash tools/validate_jazzy.sh이며 로그는 validation/jazzy-container.log입니다.
컨테이너는 소스를 읽기 전용으로 가져오며 host network나 장치를 연결하지 않습니다.
기존 DEUX의 deux_hand_msgs만 빌드 입력으로 복사하고 원본 소스는 수정하지 않습니다.

코어 테스트: URDF FK/IK·한계·충돌, Hermite 속도/가속도, scene 필수값,
XY/yaw 중복 제거·각도 래핑, 라벨 후보 선택.
ROS 테스트: 설정 직접 로드·시작 실패·설정 토픽 부재·버전 불일치 거부,
barcode 세션·만료 경계, 별도 프로세스 planner,
dry-run 결과 및 만료/중복/scene 거부, 손 정규화, 태스크 성공/판독 실패/상승 실패,
실제 bridge 코드와 합성 드라이버의 정상 실행·취소·피드백 단절, legacy adapter 거부.

합성 태스크 테스트는 물체/리더 관측을 시험 코드가 제공합니다. MuJoCo 접촉/봉투
미끄럼이나 실제 로봇 성공률을 검증한 것이 아닙니다. 실제 expanded URDF·충돌 envelope,
설치 기하·grasp/reader frame, 손 프로토콜/피드백, 파지 후6D 인식이 확정된 뒤
저장 관측 통합 드라이런과 현장 단계별 검증이 필요합니다.
