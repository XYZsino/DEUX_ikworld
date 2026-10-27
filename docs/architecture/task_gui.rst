브라우저 작업 제어 GUI
=====================

구성
----

``deux_task_gui/task_gui``는 로봇 측에서 실행하는 작은 HTTP/ROS 2 게이트웨이입니다.
브라우저에는 ROS 설치가 필요하지 않습니다. 외부 CDN이나 rosbridge도 사용하지 않습니다.

* 구독: ``/task/status`` (TaskStatus, reliable/transient-local).
* 제어: ``/task/run`` (RunTask), ``/task/move_to_ready`` (MoveToReady) Action.
  관절/손 명령을 직접 보내지 않습니다.
* 표시: 현재 BT 단계, 대상 ID, 파지/스캔 시도 횟수, 최근 단계 변경 및 작업 결과.
* 시작: 대상 ID를 비우면 task_node의 자동 선택을 사용합니다.
* 중단: 이 GUI 서버에서 소유한 Action goal을 취소하고 terminal result를 기다립니다.
  취소 수락 응답만으로 중단 완료로 표시하지 않습니다.
* 준비자세로 이동: 현재 goal 종료 및 새로운 inactive TaskStatus를 확인한 뒤
  MoveToReady를 보냅니다. 첫 준비는 기존 Prepare 경유 동작을 사용하고, 이후는
  ReadyPose로 양팔을 home_joints에 복귀시킵니다. 손 개방/폐쇄 상태는 유지합니다.
  복귀 완료 후 새 작업을 자동 시작하지 않습니다. 재시작 버튼은 제거했습니다.
* 다른 Action client가 시작한 작업은 표시만 합니다. 해당 client에서 중단하십시오.
* 브라우저 새로고침/재접속은 실행 중인 작업에 영향을 주지 않습니다.
  GUI 서버 프로세스를 재시작하면 이전 goal의 제어 소유권은 복구하지 않습니다.

연결 확인은 실제 상태 수신의 wall-clock 경과 시간으로 판정합니다.
2초 동안 상태가 없거나 Action 서버가 없으면 시작/준비자세 이동을 차단합니다.
연결이 끊기거나 종료 확인이15초를 넘기면 준비자세 예약을 해제합니다.
중단은 연결된 기존 goal에 계속 요청할 수 있으며, 연결 복구만으로 새 작업을 시작하지 않습니다.
HTTP 요청은 인증 후 큐에 넣고 단일 ROS executor에서 순차 처리합니다.

실행
----

ROS 2 Jazzy overlay를 빌드하고 기존 구동 스택과 같은 ROS_DOMAIN_ID/네트워크에서 실행합니다::

  colcon build
  source install/setup.bash
  ros2 run deux_task_gui task_gui --ros-args -p deployment:=real

시뮬레이션에서는 ``deployment:=sim``으로 실행합니다. 이 값은 화면 표시용이며,
ROS 연결 대상을 전환하거나 실기 구동을 활성화하지 않습니다.
GUI는 드라이버, perception, task/planner/execution 노드를 자동 시작하지 않습니다.
실기에서는 기존 task launch의 config, hardware_enabled, dry_run 설정이 그대로 적용됩니다.

기본 주소는 ``127.0.0.1:8080``입니다. 실행 로그의 토큰 포함 URL로 접속합니다.
토큰은 URL fragment로 전달한 뒤 주소에서 제거하고 해당 탭의 sessionStorage에 보관합니다.
API는 Bearer 토큰을 요구합니다. ``DEUX_GUI_TOKEN`` 환경변수로 고정 토큰을 설정할 수도 있습니다.

원격 PC에서 SSH 터널을 엽니다::

  ssh -N -L 8080:127.0.0.1:8080 USER@ROBOT_HOST

그 후 원격 PC 브라우저에서 ``http://127.0.0.1:8080/``를 열고 접속 설정에
로봇 측 실행 로그의 토큰을 입력합니다. 동일 포트로 전달했다면 로그의 URL을 그대로 사용할 수 있습니다.
ROS DDS 포트를 원격 브라우저에 노출할 필요가 없습니다.

설정 가능한 ROS 파라미터: ``host``, ``port``, ``deployment``, ``action_name``, ``ready_action_name``, ``status_topic``.
기본 localhost 바인딩과 SSH 터널을 권장합니다. 다른 host로 바인딩하는 경우
HTTPS 프록시 또는 신뢰하는 VPN을 통해 사용하십시오. 자체 HTTP 서버에는 TLS가 없습니다.

인터랙티브 MuJoCo
------------------

sim 의존성(MuJoCo/EGL, YOLO 및 외부 DEUX_mujoco)을 준비한 환경에서::

  ros2 run deux_task_sim serve_sim \
    --sim-repository /home/xyz-ai/git/DEUX_mujoco \
    --weights /home/xyz-ai/git/DEUX_ikworld/models/bread_barcode_yolo11n_seg_sim.pt \
    --output /tmp/deux-interactive-sim

다른 터미널에서 같은 overlay/ROS_DOMAIN_ID로 실행합니다::

  ros2 run deux_task_gui task_gui --ros-args -p deployment:=sim

sim은 초기 관절0에서 작업 요청을 기다립니다. 기본 head-rgbd 경로에서는 GUI 대상 ID를
비워 자동 선택하거나 현재 관측 중인 ``visual-N`` ID를 지정합니다.
시작하면 Prepare 및 head RGB-D PickScan BT를 실행합니다.
기존 truth 이송 시험은 ``--perception legacy-truth``로 명시하며 그때만 ``bread_3`` ID를 사용합니다. 재시작해도 sim 물체/관절을 초기화하지 않습니다.
기록 버퍼는 제한하며, 이 명령은 영상 녹화가 아닌 지속 실행용입니다.
``validate_transfer`` 자동 시험과 동시에 실행하지 마십시오(같은 Action/토픽 이름 사용).

검증 범위와 운용 조건
--------------------

중단은 ROS Action을 통한 정상 취소이며 장비의 비상정지 대체가 아닙니다.
브라우저를 닫거나 GUI 연결이 끊겨도 작업을 자동 중단하지 않습니다.
실기에서는 기존 하드웨어 인터록/비상정지 및 드라이버의 중단 처리가 필요합니다.

단계는 task_node가 실제 발행한 값만 강조합니다. 표시 순서만 보고 앞선 단계들을
성공으로 추정하지 않으며, 재시도·Recover·Prepare 생략도 그대로 표시합니다.
실제 하드웨어 연결 시험 및 전체 물체 이송 성공은 이 GUI 구현의 검증 범위에 포함하지 않습니다.

이번 검증 결과 (2026-10-25)
--------------------------

* Jazzy9개 패키지 빌드 통과.
* 기존56개 회귀 테스트와 최초 GUI2개 통합 테스트를 함께 실행해58개 통과.
  연결 손실 시 재시작 예약 해제 테스트를 추가한 뒤 GUI3개를 다시 실행해 모두 통과.
* 실제 DDS/Action + HTTP 시험: 인증, 잘못된 요청/Origin 차단, 중복 시작 차단,
  취소 ACK 이후 terminal result 대기, 시작 응답 전 중단, 서버 거부,
  stale 상태, 다른 client의 작업 보호, 연결 복구 시 자동 재시작 방지.
* 실제 MuJoCo + GUI HTTP/Action: Prepare → Canceled → Prepare → Approach → Canceled.
  재시작 이후 준비 동작을 마치고 Approach에 진입한 뒤 중단 완료를 확인했습니다.
* PC1400px/모바일390px 화면, 현재 단계 강조 및 조작 확인. JavaScript 오류0.
  이 환경의 headless Chromium은 로컬 HTTP 접속을 거부해 Playwright에서 요청을
  Node HTTP로 중계했습니다. 응답은 실제 GUI/ROS/sim 서버의 응답이며 mock이 아닙니다.
  사용자 기본 브라우저(Firefox)에는 실제 localhost URL을 열었습니다.
* 증거: validation/ros2-gui/의 tests.log, gui-tests-final.log, browser-check.json,
  gui-samples.json, sim.log 및 gui-*.png. 전체 이송/실기 성공 검증은 아닙니다.

적재 flow 관제 및 SSH 운용 (2026-10-26)
----------------------------------------

단계 지정 시작은 사용자 요청으로 추가하지 않았다. RunTask 계약은 target_id만 받는다.
화면에 단계 선택 제어가 없으며, HTTP에 start_stage 등 지원하지 않는 필드를 보내면 거부한다.
GUI는 현재 BT 순서와 실패/취소 직전 마지막 단계를 표시하며, 재시작은 항상 새 RunTask다.

placement.enabled인 head BT는 head_rgbd_pick_scan_place workflow를 발행한다.
기존 스캔 단계 이후 ClearReader → PlaceApproach → Place → Release → PlaceRetreat →
Retreat가 화면에 표시된다. 사용자 요청으로 head BT의 VerifyPlace는 제외했다.
flow 완료는 개방/복귀 동작 완료이며 물체 적재 확인을 뜻하지 않는다.
완료를 추정해서 앞 단계들을 성공으로 칠하지 않는다. 준비자세 이동은 물체·물리 상태를 되돌리지 않는다.

SIM과 GUI를 같은 프로세스에서 함께 실행할 수 있다::

  source install/setup.bash
  export ROS_DOMAIN_ID=42
  ros2 run deux_task_sim serve_sim \
    --sim-repository /home/xyz-ai/git/DEUX_mujoco \
    --weights /home/xyz-ai/git/DEUX_ikworld/models/bread_barcode_yolo11n_seg_sim.pt \
    --output /tmp/deux-sim-control --gui --gui-port 8080

실제 로봇에서는 보정된 구동 스택이 실행 중인 호스트에서 같은 overlay와 ROS_DOMAIN_ID로
GUI만 실행한다. GUI 실행 자체는 드라이버나 모션을 활성화하지 않는다::

  source install/setup.bash
  export ROS_DOMAIN_ID=42
  ros2 launch deux_task_gui control.launch.py deployment:=real port:=8080

원격 PC에서는 포트 충돌을 피할 수 있도록 로컬18080을 로봇8080으로 연결한다::

  ssh -N -L 18080:127.0.0.1:8080 USER@ROBOT_HOST

브라우저 주소는 http://127.0.0.1:18080/ 이다. 로봇 실행 로그의 접속 토큰을 입력하거나
로그 URL의 포트만18080으로 바꾸어 연다. 토큰은 탭에만 저장한다.
로봇 GUI와 ROS 스택을 tmux에서 실행하면 SSH 셸을 닫아도 프로세스를 유지할 수 있다.
브라우저/터널 단절은 작업 취소가 아니며, 재접속만으로 새 작업을 보내지 않는다.
동일 로봇에는 하나의 GUI 서버를 지속해서 사용해야 goal 소유권이 유지된다.

실시간 SIM 미리보기
--------------------

serve_sim --gui 사용 시 같은 페이지에 실제 MuJoCo 장면을 약3fps로 표시한다.
/api/frame은 API와 같은 Bearer 인증을 요구한다. 렌더는 SIM executor에서 수행하며
HTTP에서는 JPEG 사본만 읽는다. REAL 모드에서 SIM 이미지를 만들어 표시하지 않는다.
영상 수신 지연은 LIVE 표시를 해제한다. 영상은 측정 피드백이나 비상정지 통로가 아니다.

이 워크스페이스의 현재 미리보기는 http://127.0.0.1:8090/ 이다.
Docker 내부8090을 호스트 loopback8090으로 전달한다. 기존8080 인스턴스는 변경하지 않았다.
토큰 없는 새 탭에는 접속 설정에서 실행 중인 서버 토큰을 입력한다.
토큰 포함 URL은 http://127.0.0.1:8090/#token=실행로그의토큰 형식이다.

ZED RGB와 선택 대상3D 박스
--------------------------

SIM/REAL 모두 GUI가 /task/observation(Observation)과 /task/detections(DetectionArray)를
구독한다. 실기는 ZED ROS 드라이버의 RGB/registered depth/CameraInfo를 기존
observation_node에서 Observation으로 묶고, tracked_detection_node를 함께 실행한다.
GUI는 구독만 하며 카메라/드라이버를 자동 시작하거나 실기 카메라 좌표를 추정하지 않는다.
토픽이 다르면 control.launch.py의 observation_topic, detections_topic,
camera_base_frame 인자를 사용한다. 기본 base frame은 base_link이다.

같은 sequence, 취득시각, optical frame의 RGB/깊이/검출을 결합한다.
TaskStatus.target_id와 일치하는 bread_bag의 instance mask 안에서 깊이 점을 추출하고,
깊이 이상치를 제거한 뒤 해당 영상 시각의 base_from_camera로 변환한다.
각 축2~98백분위 범위의8개 모서리를 원본 CameraInfo.P로 투영해12개 선을 그린다.
TF·RGB-D 동기·마스크·선택 대상이 없으면 박스를 만들지 않는다.
최대8프레임을 보관하며, 영상 갱신이0.8초 이상 끊기면 RGB 표시를 숨긴다.

박스는 가시 표면의 관측3D 범위이다. 가려진 뒷면까지 포함한 물체 전체 크기,
ICP full pose, 시뮬레이터 정답 pose 또는 파지/적재 성공 판정으로 사용하지 않는다.
다른 영상 시점의 오래된 ICP 박스를 최신 RGB에 붙이지 않는다.
선택 대상이 없는 Idle 상태에서는 RGB만 보이며 BT에서 대상이 선택되면 박스가 표시된다.

상단은 현재 단계와 제어, 그 아래 왼쪽은 실시간 MuJoCo, 오른쪽은 ZED RGB,
맨 아래는 전체 폭의 작업 흐름이다. 좁은 화면에서는 영상이 세로로 배치된다. REAL에는 MuJoCo 패널이 숨겨지고 ZED RGB는 유지된다.
SIM에서 첫 작업 시작 전에는 컨베이어를 대기시켜 인증/접속 중 물체가 지나가지 않게 했다.
첫 RunTask 승인 후에는8cm/s로 계속 이동하며 재시작이 장면을 초기화하지 않는다.

현재 검증 범위
--------------

통합 SIM GUI에서 실제 RGB 수신 및 MuJoCo/RGB/BT 배치를 브라우저로 확인했다.
GUI·3D 투영14개 테스트 통과. 선택 대상 변경, 시각/sequence/frame 불일치,
TF 부재, mask 부재, 깊이 결손, mm/m 단위, 깊이 이상치를 검사했다.
실기 카메라 연결 시험은 하지 않았다. 별도 DDS reader 시험에서는 작은 DetectionArray는
수신했으나 대용량 Observation은 수신하지 못했다. 따라서 실기에서 GUI를 별도 프로세스로
실행할 때 Observation 전달(QoS/공유메모리/수신 버퍼/네트워크)을 먼저 확인해야 한다.
같은 프로세스로 묶은 현재 SIM GUI의 수신 성공을 실기 DDS 전송 성공으로 해석하지 않는다.


준비자세 전용 Action
---------------------

``/task/move_to_ready``는 빈 goal과 success/reason 결과를 사용한다.
RunTask와 공통 admission lock을 사용해 수락 시점부터 병렬 실행을 막는다.
측정 관절의 최신성, planner/executor, 기존 충돌·관절 한계 검사를 그대로 적용한다.
이미 준비를 완료했다면 물체 ICP를 요구하지 않고 양팔을 설정된 home_joints로 복귀시킨다.
손은 현재 자세를 유지하며, 닫힌 손은 설정된 닫힘 힘의 목표를 유지한다.
준비자세 이동 중에도 중단 버튼으로 취소할 수 있다.
통신 소실 또는 중단 종료 대기15초 초과 시 예약된 복귀를 해제한다.
임의 BT 단계에서 시작하는 인터페이스는 제공하지 않는다.
SIM의 최초 준비자세 이동만으로 컨베이어는 출발하지 않는다.

검증:8개 패키지 빌드 및153개 테스트 통과. 실제 ROS 2/MuJoCo에서 HTTP 준비 명령을
두 번 실행해 최초 Prepare→Ready, 이후 ReadyPose→Ready와 새 작업 미실행을 확인했다.
측정 팔 관절의 최대 목표오차는0.00078rad(약0.045도)였다.
작업 중 준비 예약의 취소 완료 대기, 연결 끊김 시 예약 해제, 준비 동작 자체 취소는
합성 DDS/Action 테스트로 확인했다. 실기 이동 검증은 아직 하지 않았다.
