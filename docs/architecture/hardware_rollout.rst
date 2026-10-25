ROS 2 sim 검증 이후 커밋 범위와 실기 적용 계획
=============================================

작성 기준: 2026-10-25, HEAD ``2c89b29`` 및 현재 미커밋 작업 트리.
이 문서는 커밋 준비 시점의 변경 범위와 실기 적용 계획입니다. 이후 사용자 요청으로
아래 범위의 소스·설정·문서를 커밋합니다. 푸시, 장치 설정 변경, 하드웨어 명령 전송은
수행하지 않습니다. 기존 DEUX 원본은 수정하지 않습니다.

현재 판단
---------

현재 코드는 ROS 제어 흐름을 실제 장치 인터페이스에 연결하기 위한 기반입니다.
실기 자동 이송 전체가 검증된 상태는 아닙니다. sim에서는 접근·파지·상승·스캔·바코드
수신 후 Place 계획 요청까지 진행했고, IK unreachable로 종료했습니다.
Release/Retreat/VerifyPlace는 최종 sim 녹화에서 실행되지 않았습니다.
실기 연결 초기에는 정지한 장면에서 관측, 단일 장치, 단일 단계 순서로 검증하고,
전체 자동 실행은 아래 선행 조건을 충족한 다음 진행합니다.

확인된 근거와 한계
------------------

* ROS 2 Jazzy: 8개 패키지 빌드 완료, 회귀 테스트 52개 통과.
  이는 테스트용 deux_hand_msgs를 포함한 빌드이며, 실기 시험 52건이라는 의미가 아닙니다.
* MuJoCo: 영상 24.276초/238프레임, 관측 이벤트 5,946건.
  14개 업무 토픽과 6개 Action의 호출/수락/피드백/결과를 계측했습니다.
* 측정된 봉투 상승량 약 0.133m, 가상 리더 문자열 수신, Place 계획 실패 결과의
  상위 전달을 확인했습니다. 물리 태스크 성공을 확인한 것은 아닙니다.
* 센서는 렌더 가시성 기반 truth 6D와 분석적 라벨 표면입니다.
  실제 ZED, YOLO/ICP, 실제 바코드 리더는 이번 실행에서 검증하지 않았습니다.
* 정지 컨베이어와 이동시킨 출발 트레이, sim 전용 preload, 단순화한 충돌 형상,
  sim 전용 허용 오차를 사용했습니다. 여러 ROS 노드는 같은 executor에서 실행했습니다.
  별도 장치/프로세스 배포 지연, 이동 물체 추적, 실물 변형·가림은 확인하지 않았습니다.
* Topic 기록은 관측 구독자의 수신 증거이며 대상 노드별 처리 완료 증거가 아닙니다.
  Action 기록은 client API 경계이며 패킷 캡처가 아닙니다.
* 영상 시간 이동·노드 필터·메시지 상세 보기 브라우저 검사도 통과했습니다.

로컬 증거 파일:

* ``validation/ros2-sim/build-final.log``
* ``validation/ros2-sim/regression-current.log``
* ``validation/ros2-sim/scenario-recording/{result,ros-events,browser-check}.json``
* ``validation/ros2-sim/scenario-recording/{dashboard.html,scenario.mp4}``

화면 주소::

  file:///home/xyz-ai/git/DEUX_ikworld/validation/ros2-sim/scenario-recording/dashboard.html

커밋해야 할 현재 변경
--------------------

기존 작업과 이번 sim 변경을 포함한 범위입니다. ``git diff --stat``에는 신규 미추적
패키지가 나타나지 않으므로 ``git status --short``도 함께 확인해야 합니다.
인터페이스가 함께 바뀌었으므로 다음 묶음은 동일 배포 버전으로 빌드합니다.

1. 메시지·인식·제어 기반

   * ``src/deux_task_msgs/``: 신규 Action 및 MotionPlan/TaskStatus/PerceptionRequest/
     ScanSession/BarcodeTarget/BarcodeRead/DetectionArray, 객체 메시지 확장과 빌드 등록.
   * ``src/deux_task_perception/`` 변경: 공통 검출과 독립 ICP/라벨 worker,
     request_id/lease 처리, 경량 관측, barcode bridge, 설정/launch와 테스트.
   * ``src/deux_task_core/``: 설정 검증, URDF 모델/FK/IK/계획, 필터와 테스트.
   * ``src/deux_task_runtime/``: BT, task/planner/execution, 관측 어댑터,
     경로 검사와 테스트. task는 단계·재시도·결과를 관리하고 planner는 경로를 계산합니다.
   * ``src/deux_task_adapters/``: 팔 Action 및 손 명령/측정 피드백 브리지.
   * ``src/deux_task_bringup/``: launch, 실측 입력을 요구하는 설치/손 매핑 템플릿.
     기본 hardware_enabled=false, dry_run=true를 유지합니다.

2. sim 연결과 공통 실행 수정

   * ``src/deux_task_sim/`` 전체: MuJoCo ROS 드라이버, 실행기, 상태 녹화,
     토픽/Action 계측, 동기화 HTML 생성기, 패키지 및 의존성 정보.
   * 공통 코드의 Approach 관절 경로, 손 성공 후 목표 유지, loaded-motion 손 목표 유지,
     제한된 관절 수치 오차, sim 전용 wall watchdog 배율도 포함합니다.
     이 변경들은 1번의 core/runtime/adapters에 섞여 있으므로 누락하지 않습니다.
   * 손 preload 유지 동작은 실기 힘/전류 검증이 필요합니다. sim에서 통과했다는 이유로
     실기 hand_closed 값을 확정하지 않습니다.

3. 실행·설계·인계 문서

   * ``README.rst``, ``docs/HANDOFF.rst``.
   * ``docs/architecture/{runtime,controlled_perception,simulation,hardware_rollout}.rst``.
   * ``docs/architecture/{design.html,perception.rst}``의 기존 문서 수정.
   * 구조도 본문/원본: ``nodes.html``, ``nodes.dot``, ``nodes.graph.json``,
     ``scenario.html``, ``scenario-final.workflow.json``.
   * ``scenario.delivery.json``, ``scenario.review.json``, ``scenario.visual-check.*``는
     생성·검토 증거입니다. 실행 코드 의존성은 없으며 기본 커밋 범위에서는 제외할 것을 권장합니다.

현재 소스는 서로 의존하므로 첫 커밋은 위 기반 코드와 sim 코드를 함께 넣는 방식이
간단합니다. 이력 분리가 필요하면 실제 diff hunk와 패키지 의존성에 맞춰 분리하고
각 커밋에서 빌드해야 합니다. 디렉터리만 순서대로 나누면 중간 커밋이 깨질 수 있습니다.
문서·구조도는 별도 커밋으로 분리할 수 있습니다.

커밋 제외 및 별도 보존
---------------------

* 이전 사용자 방침대로 ``tools/``와 ``validation/``는 커밋하지 않습니다.
  ``tools/replay_icp.py``, ``tools/validate_jazzy.sh``, ``tools/validate_sim.sh``도 해당합니다.
* build/install/log, Python cache, 생성 URDF/실행별 YAML, 영상/이미지/원시 trace를
  소스 패키지에 복사하지 않습니다. 현재 .gitignore는 tools/validation 전체를 제외하지
  않으므로 ``git add .``로 일괄 추가하지 않습니다.
* 실행 증거는 Git 외부 검증 묶음으로 별도 보존합니다. 배포 커밋 SHA, 외부 DEUX 및
  MuJoCo 소스 버전/해시, 모델/설치 설정 해시와 함께 남겨 재현 기준을 고정합니다.
* tools를 제외하면 새 checkout에는 위 셸 검증 명령이 없습니다. sim 패키지의
  ``validate_transfer``/``render_timeline`` 실행 방법과 의존성은 simulation.rst에 있습니다.
  컨테이너 자동화까지 공유하려면 별도 배포해야 하며, 저장소만으로 같은 환경이
  자동 재현된다고 표시하지 않습니다.
* ``models/deux_1021_A_v2.0only_adamw.pt``는 이미 추적 중입니다.
  이번 작업에 신규/변경된 pt 파일은 없습니다. 실제 봉투/바코드 클래스 의미는
  별도 확인해야 하며 모델 포함 여부와 클래스 검증 완료를 구분합니다.

커밋 직전 확인: stage한 파일 목록에 위 제외 항목이 없는지 확인하고,
``git diff --cached --check`` 및 ``git diff --cached --stat``를 검토합니다.
메시지 변경을 사용하는 외부 클라이언트도 같은 인터페이스 버전으로 재빌드합니다.

실기 적용에 추가할 구현·설정
---------------------------

아래 항목은 완료 선언이 아니라 남은 작업입니다.

A. 인식과 물체 자세 — 전체 시나리오의 선행 조건

   * 외부 ZED의 등록 RGB/depth/CameraInfo, 실제 base↔camera TF를 연결합니다.
     기존 head camera pose를 외부 카메라 보정값으로 사용하지 않습니다.
   * controlled_perception.yaml에 실제 모델 경로, model origin, 봉투/바코드 클래스,
     카메라/EE frame을 채웁니다. YOLO의 b1 등을 근거 없이 바코드로 간주하지 않습니다.
   * 현재 yaw ICP는 full_orientation=false입니다. Place에서 요구하는 파지 후6D 자세를
     제공하는 생산자를 구현/통합해야 합니다. full_orientation만 true로 바꾸면 안 됩니다.
     추정 결과는 실제 회전·위치, 원본 시각, 추적 ID/request_id, 품질 정보를 보존해야 합니다.
   * 겹침·가림·봉투 변형·ID 교차, 손에 들린 상태의 가시성을 확인합니다.
     현재 ScanSearch는 정지 상태 대기이며 뒤쪽 라벨을 찾는 능동 회전 탐색은 없습니다.
     해당 물체 배치가 운용 범위에 포함되면 탐색 동작을 추가해야 합니다.

B. 실제 구동·리더 연결

   * MuJoCo 팔 Action 서버를 실제 FollowJointTrajectory 서버로 교체하고,
     /arm/joint_states에 실제 관절 측정값을 연결합니다. 서버 이름이 다르면
     arm_bridge_node의 driver_action과 필요한 토픽 remap을 배포 설정에 반영합니다.
   * /hand/left/cmd 및 /hand/right/cmd를 실제 손 드라이버에 연결합니다.
     hand_mapping.yaml에 ID/position mode/부호/offset/한계/속도/전류 단위와
     feedback_topic/type을 확정합니다. 명령값으로 측정 피드백을 대체하지 않습니다.
   * USB/serial/SDK 바코드 리더 결과를 std_msgs/String으로 발행할 어댑터를 연결합니다.
     이 overlay에는 실제 리더 드라이버가 없습니다. barcode_topic 기본값은 /barcode/raw,
     barcode_bridge_node의 simulated는 false입니다.
   * sim 시작 클라이언트를 운영 UI/클라이언트로 교체하여 동일한 /task/run을 호출합니다.
     task.launch.py는 실제 팔/손/카메라 드라이버를 시작하지 않으므로 배포 진입점에
     드라이버 실행, 설정 경로, 노드 시작 순서와 종료 절차를 추가해야 합니다.

C. 실측 설치와 운용 설정

   * installation.yaml: 실제 expanded URDF/리프트/EE, grasp/reader/label 변환,
     작업영역, 상자와 목표 위치, 로봇·리더·봉투 충돌 형상, 속도/가속도와 허용 오차.
   * 손 open/closed와 preload는 실제 전류·접촉·파지 유지 결과로 결정합니다.
     sim ID/mode, 트레이 이동, 리프트 +15cm, 단순 손목 충돌 형상과 넓힌 오차를 복사하지 않습니다.
   * use_sim_time=false, synthetic=false, simulation_wall_timeout_scale=1.
     실제 관절값 기반 TF와 여러 장치 간 시각 동기화를 구성합니다.
   * 실제 확인 후 calibration_confirmed/command_ownership_confirmed를 설정하고
     /state/status의 teleop=off 및 allowed_state_modes를 현재 운용 모드와 연결합니다.
     현재 gate는 직접 팔 명령 publisher와 상태를 검사하지만 모든 장치의 독점 제어를
     보장하는 범용 중재기는 아닙니다. 다른 명령 경로와 드라이버 watchdog도 확인해야 합니다.
   * 현재 /state/status gate는 수신 JSON과 publisher 존재를 확인합니다.
     publisher가 살아 있으나 상태가 갱신되지 않는 경우를 구분할 freshness/heartbeat 기준은
     실기 통합에서 정의하고 필요한 감시를 추가해야 합니다.
   * 설치 YAML은 시작 시 각 노드가 직접 읽습니다. 변경 후 관련 노드를 재시작하며,
     설정 버전 일치 여부를 확인합니다. scene_node나 배포 토픽을 추가할 필요는 없습니다.

단계별 검증과 통과 조건
-----------------------

각 단계의 기준은 실측 설치 설정에 먼저 기록하고 시험 결과와 비교합니다.
허용 위치/각도 오차, 감시 시간, 전류/속도를 이 문서에서 임의로 확정하지 않습니다.

0. 동일 소스 재빌드 및 배포 점검 — 모터 명령 없음

   * Jazzy에서 메시지를 포함해 깨끗이 빌드하고 기존 52개 회귀 테스트를 실행합니다.
   * 실기용 YAML/URDF 경로, 모델·클래스·frame, 메시지 버전, QoS/네임스페이스를 확인합니다.
   * 기본 hardware_enabled=false, dry_run=true와 실기 시간 설정으로 시작합니다.
   * 통과: 필수 설정이 누락되면 시작/실행이 거부되고, 준비된 설정은 일관되게 로드되며
     장치 명령이 발생하지 않습니다. 이 단계는 동작 검증이 아닙니다.

1. 실센서와 측정 피드백 — 모터 명령 없음

   * ZED 입력/등록 depth, TF 방향·단위, 28관절 이름·부호·시각을 실측과 비교합니다.
   * 봉투/라벨 검출, 바코드 문자열, 동일 물체 ID 유지, request_id 교체/lease 만료를 검사합니다.
   * 통과: 오차와 데이터 나이가 설정 기준 이내이며, 잘못된 TF·stale 데이터·다른 목표의
     라벨/판독이 제어 입력으로 사용되지 않습니다. synthetic 판독은 실기에서 거부됩니다.

2. 계획과 명령 차단 — 실제 관측을 사용하되 모터 명령 없음

   * 단계별 PlanMotion의 IK, 충돌, 관절/속도/가속도 제한과 시작 자세를 확인합니다.
     Place는 유효한6D 관측으로 별도 검증하고 현재 sim IK 실패 조건과 구분합니다.
   * dry_run 실행 결과는 dry_run_only이며 task는 전체 성공으로 처리하지 않습니다.
     dry_run만으로 BT 전체 정상 종료를 기대하지 않습니다.
   * 통과: 도달 불가 목표/설정 불일치/관측 누락이 계획 또는 실행 전에 거부되고,
     유효 경로의 끝 자세·여유 거리가 실측 설치 기준에 맞습니다.

3. 팔·손 단독 동작 — 제한된 범위에서 실제 명령

   * 장치별 준비 완료 후 hardware_enabled=true, dry_run=false로 전환합니다.
     낮은 속도의 단일 팔 경로 및 손 개폐를 각각 검증하고 측정값과 목표를 비교합니다.
   * Action accept/result/cancel, 추종 오차, 손 피드백 단절, 명령 중복을 확인합니다.
     성공 후 손 목표 유지에 따른 전류, 실패/취소 후 hold 및 장치 watchdog을 관찰합니다.
   * 통과: 실측 추종/전류가 승인 범위 안이며, 취소·오류 시 정의된 정지/유지 동작이
     실제 장치에서 확인됩니다. Action 성공 응답만으로 물리 결과를 판정하지 않습니다.

4. 단계 조합 — 처음에는 정지 물체

   * Approach/Descend → Close/Lift → ScanSearch/AlignBarcode/WaitBarcode 순으로 진행합니다.
   * 새 관측에 의한 상승 확인, 오른팔 파지 유지, 왼팔 리더 정렬과 실제 문자열 수신을 확인합니다.
   * 통과: 단계 전환 조건이 실제 관측과 일치하고, 잘못된 session/target/만료 판독이 거부됩니다.

5. 적재 및 오류 경로 — 아직 sim 최종 녹화에 없는 구간 포함

   * Place/Release/Retreat/VerifyPlace를 각각 검증한 뒤 전체 /task/run을 실행합니다.
   * 새6D 관측, 손 해제, EE와 물체 분리, 실제 적재 영역 조건을 확인합니다.
   * 목표 가림, 라벨 미검출, 판독 시간 초과, IK 실패, Action 취소, 피드백/TF 단절,
     상태 모드 변경, 노드 종료 상황에서 재시도 또는 실패 종료와 명령 중단을 확인합니다.
   * 통과: 정상 경로와 각 오류 경로의 결과가 실제 상태와 일치하고 남은 명령이 없습니다.
     이동 컨베이어는 이 정지 장면 검증 후 별도 지연·추적·드리프트 시험으로 다룹니다.

검증 기록과 완료 판단
---------------------

매 실행마다 다음을 함께 남깁니다:

* 실행 ID, 일시, 코드/인터페이스 버전, 모델/설치 YAML/URDF 해시, 장치·펌웨어 버전.
* 실행 단계, 실제 enable/dry_run/synthetic/time 설정, 적용한 제한값.
* 원본 관측/관절/TF/상태/바코드와 Action goal/result/cancel 기록, 동기화된 영상.
* 기대 결과, 실제 결과, 통과/실패/미실행, 오차·지연·전류 수치와 실패 사유.

현재 TraceRecorder는 sim 실행기에 연결되어 있습니다. 실기 녹화에는 카메라 영상과
장치 timestamp에 맞춘 기록 진입점 및 rosbag2/Action 계측을 별도로 연결해야 합니다.
단순히 sim 대시보드의 노드 이름을 바꾸어 실기 증거로 사용하지 않습니다.

당장 커밋 가능한 것은 구현 코드·템플릿·문서입니다. 전체 실기 자동 이송 준비 완료는
6D 관측/실제 리더/실측 설정 및 위 단계별 하드웨어 검증을 마친 뒤에 판단합니다.
