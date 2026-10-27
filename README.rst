최신 — 실기 코드 기본 IK를 PyRoKi로 통일 (2026-10-27)
==================================================

사용자 요청으로 ik_backend 생략 시 코드 기본값도 pyroki로 변경했다.
실기/일반 launch 설정 로더가 기본값을 명시적으로 채워 scene_version에 포함한다.
명시한 scipy/curobo는 유지하며, 합성 테스트 fixture는 scipy를 명시한다.
task.launch.py도 head_rgbd와 동일하게 planner_python(Python3.12) 인자를 지원한다.
선택 의존성이 없으면 시작 실패, SciPy 자동 대체 없음. 실제 로봇에는 연결하지 않았다.
7패키지 /tmp/real-pyroki-install 빌드. ROS_DOMAIN_ID215에서266테스트 통과,
task.launch --show-args에서 planner_python 확인. 최초 기본 도메인 실행은 GUI 통신 테스트
1개가5초 timeout(265개 통과); 코드를 바꾸지 않고 독립 도메인 전체 재실행 통과.
증거 validation/head-rgbd/real-pyroki-default/. 현재 관제 SIM은 그대로 유지했다.
상세 docs/architecture/planner_backends.rst. 실기 Python3.12 환경에 PyRoKi 의존성 설치 필요.

최신 — PyRoKi 기본값 및 관제 SIM 실행 (2026-10-27)
=================================================

사용자 요청으로 head_calibration.yaml과 bringup installation.yaml의 ik_backend를 pyroki로 변경.
키가 없는 legacy 설정의 scipy 호환 동작은 유지한다. SIM/bringup 2패키지 재빌드.
restart_live.py는 /tmp/backends-install, /opt/deux-planner-venv/bin/python 사용으로 갱신했고
이전/현재 Python 경로의 같은 serve 프로세스를 확인하는 패턴으로 변경했다.
현재 두 팔 PyRoKi 초기 준비 완료, ROS_DOMAIN_ID230, 8090 GUI Idle 확인.
MuJoCo/ZED RGB 두 영상·준비자세 버튼·흐름 배치 및 모바일 검사 통과.
기존 토큰 유지하여 브라우저 열기 성공. 자동 작업은 시작하지 않았다.
이전 SIM 종료 요청은 이번 명시적 시작 요청으로 해제됨. 전체 PyRoKi 시나리오 검증은 아직 아님.
증거 validation/head-rgbd/pyroki-live/{build.log,live-check.json,browser-check.json}.

최신 — SciPy / PyRoKi / cuRobo IK 선택 및 비교 (2026-10-27)
=========================================================

사용자가 두 백엔드 구현·선택·비교 보고서를 요청했다. 도중 실행 중 SIM 종료 요청으로
8090 serve 프로세스를 INT 종료하고 프로세스 없음 확인. 이후 SIM을 다시 시작하지 않았다.
설치 YAML ik_backend: scipy|pyroki|curobo, 기본scipy. SIM CLI --ik-backend 제공.
공통 컨베이어 예측/후보 평가/retiming/전체28관절 충돌 검사는 유지, IK만 교체했다.
cuRobo native 모션/궤적 최적화는 사용하지 않는다. 다른 엔진으로 자동 fallback 없음.
MotionPlan.ik_backend ABI 추가, 실행기는 설치와 다른 엔진 계획 거부. 전체 재빌드 필요.
플래너 worker에서 양팔 초기 준비 및 버전별 모델/컴파일 캐시. 작업 접수 전 준비 완료.
head_rgbd.launch.py planner_python으로 플래너만 Python3.12 venv 지정 가능.

7패키지 빌드/264테스트 통과. 세 실제 라이브러리의 오프라인 URDF 검사와 ROS PlanMotion
Lift 응답 검증 완료(ROS_DOMAIN_ID214, 실행노드/팔명령/MuJoCo 없음).
36개 도달목표×3=108/108 통과, 도달불가4×3=12/12 거부, 세 방식 동일.
IK 중앙값 SciPy5.33/PyRoKi CPU1.87/cuRobo GPU8.74ms. 전체5후보 검사 중앙값
2.475/1.304/2.705초. 위치3개×2회 경로4/6 통과; X65cm는 TrackGrasp에서 모두 거부.
같은 입력·허용 기준이나 내부 비용/seed 전략은 다름. CUDA native 배치 계획 비교 아님.
전체 SIM/실기 미검증. 이번 입력에서는 다음 SIM 비교 후보로PyRoKi 권장, 기본SciPy 유지.

증거 validation/head-rgbd/planner-backends/dashboard.html, benchmark-*.json, ros-*.json.
보고서 차트·필터·모바일 검사 완료, 브라우저 열기 성공. 상세 docs/architecture/planner_backends.rst.
컨테이너deux-head-rgbd 내 /tmp/backends-install 신규overlay,
/opt/deux-planner-venv/bin/python(Python3.12) 선택환경,
/opt/deux-curobo-source 로컬소스사본. cuRobo/DEUX 원본레포 수정 없음.
PyRoKi CPU/JAX0.6.2, cuRoboV2 core78fd485f/Torch2.10cu130/Warp1.18.
원래host cuRobo .venv3.11/pyroki conda3.10은 Jazzy노드에 바로 사용하면 안 됨.
기존 restart_live.py는 아직 /tmp/candidates-install을 사용하므로 다음 실행 시 새overlay/
원하는 백엔드/해당 Python을 명시해야 한다. 사용자 요청 없이 SIM 재시작하지 말 것.

최신 — SIM 파지 후보 비교 및 Lift 사전 검증 (2026-10-27)
======================================================

기준/yaw±8도/접근X±1.5cm 다섯 후보를 접근→하강·닫힘→Lift까지 검사한다.
이동량·최소 충돌 여유로 선택 후 최신 관측으로 전체 재검증, 접근 후에도 선택 자세의
TrackGrasp/Lift를 재검증한다. 실행 감시는 유지. 첫 비교 궤적은 실행하지 않는다.
SIM candidate_search.enabled=true, start_lead=.8초. 고정 XYZ gate는 계속 ik 정책.
PlanMotion/MotionPlan grasp_candidate_id ABI 추가, /task/grasp_candidates JSON 진단.
7패키지/257테스트 통과. 최종 SIM 43.350초에 스캔·개방·후퇴 완료. 실기 미검증.
선택 approach_x_plus_15mm. 물리경고0/설비침투0/weld0, 물체침투23.14mm는 남음.
초기 두 시도는 예약 시작 시각 누락으로 접근 전에 거절. 최종 최신 관측 재검증 적용.
증거 validation/head-rgbd/grasp-candidates/scenario-03/dashboard.html.
상세 docs/architecture/grasp_candidates.rst. overlay /tmp/candidates-install.
cuRobo 비교 질문에는 V2 배치 경로계획 우선 검토를 답했고 실제 교체는 하지 않았다.
기존8090 SIM은 장시간 물리 경고로 종료된 상태였고, 활성 프로세스 없음 확인 후 새 빌드로 실행.
기존 토큰 유지, 관제 Idle·두 영상·모바일 배치 확인. 영상 대시보드/후보 행 선택/단계 이동 검증 후 브라우저 열기 성공.

최신 — 고정 XYZ 작업 범위 제거, IK 경로 허용 (2026-10-27)
==========================================================

사용자 요청: IK로 갈 수 없는 지점 외에는 파지 시도. 기존 범위는 IK 산출값이 아닌 SIM 후보 박스였다.
head_calibration.conveyor_tracking.workspace_policy=ik. 접근/파지/합류/실행/가림예측의 XYZ 박스 gate 제거.
관절·속도·가속도·충돌·원시 피드백·관측 만료·예측 편차·불확실성 검사는 유지한다.
이전 box 모드는 호환용(default). 실기 설치에도 ik를 명시해야 한다.
RGB ROI/깊이박스 선택과 가까운 컨베이어 세로 범위 유지. 기존 박스는 ROI/순위에만 사용.
7패키지/252테스트 통과. 실제 URDF 오프라인 X61cm·65cm 접근/하강/닫힘 경로 통과(구 상한60cm).
MuJoCo+YOLO+ICP 스캔·개방·후퇴40.936초 완료. 적재 확인/실기 검증 아님.
증거 validation/head-rgbd/ik-workspace/. 상세 docs/architecture/ik_workspace.rst.
신규 /tmp/ik-workspace-install, 관제8090 비활성 확인 후 재시작/기존토큰 유지/브라우저 열기.
restart_live.py는 신규 설치 사용. 확인 중 관제 작업이 Release로 진행되어 유지했다.

최신 — 가까운 컨베이어만 RGB ROI 세로 확장 (2026-10-27)
========================================================

사용자 정정: 먼 쪽을 제외한 가까운 컨베이어만 포함.
selection_roi_vertical_fraction=[0.55,1.0]. 640×480 기준 Y264~480, X117.68~640은 기존 투영 유지.
전체 높이 [0,1] 시도는 폐기했다. 7패키지 재빌드, 비활성 확인 후8090 재시작·인증 브라우저 열기.
실제 RGB metadata 및 이미지로 새 ROI 확인. 증거 roi-selection/near-conveyor-live-check.json,
rgb-roi-near-conveyor.jpg. 움직임/성공률 추가 시험 없음. 계획기의 실제3D 영역 검사는 유지.

최신 — RGB ROI + 3D 박스 단일 프레임 선택 (2026-10-27)
=======================================================

사용자가 기존 오른손 선택 영역의 RGB 투영을 ROI로 선택했다.
head_calibration: target_selection_policy=rgb_roi_3d_bbox.
검출 박스 중심이 ROI 안이고 같은 프레임의 depth/mask 3D 박스가 유효하면 즉시 선택.
BT 추가 .5 신뢰도/3회 연속 관측/XYZ 선택 gate 제거. ICP 유효 속도 최소3회는 유지하되
추가6회·속도 안정성창·BT 사전 예측 gate 생략. 플래너/실행기 영역·충돌·만료 검사는 유지.
여러 후보는 오른손 영역을 통과하는 컨베이어 열 우선, 그 안에서 상류 순위. 제외 조건 아님.
GUI에 노란 Pick ROI 상시 표시. GUI/선택 증거는 같은 visible_geometry 표면 박스 계산 사용.
DetectionArray/VisualEvidence ABI 변경, 새 전체 빌드 /tmp/roi-install.
7패키지/242테스트 통과, 순위 보완 관련26개 통과.
SIM 대상 선택→접근→닫힘→Lift→ScanPose 확인, ReacquireHeldTarget 시간 초과27.676초.
전체 시나리오 성공 미확인. 첫 실행은 옆 열 후보로 접근 계획 거부되어 순위를 보완했다.
증거 validation/head-rgbd/roi-selection/. 상세 docs/architecture/rgb_roi_selection.rst.
8090 관제는 비활성 확인 후 새 설치로 재시작, 기존 토큰 유지, RGB ROI/두 영상 확인 후 열었다.
restart_live.py는 /tmp/roi-install 사용. 로컬 전달 프로세스는 detached 실행.

최신 — 완료 이벤트 / 검출·ICP 통합 / 관절 중계 제거 (2026-10-27)
===============================================================

head-rgbd 경로는 perception_node 하나에서 Detection/ICP 두 worker를 실행한다.
각 worker는 실행 1개 + 최신 대기 1개, 완료 GuardCondition으로 20ms 폴링 제거.
/task/detection_frames 제거. 요청 세대·촬영 시각·lease·clock 검사로 이전 결과 차단.
StateObservation의 joints/arm_state/hand_state 제거, 유효성·측정 시각·사유만 유지.
플래너는 /arm/joint_states와 /task/hand_state 직접 수신. 실행기는 원본 피드백 유지.
메시지 ABI 변경: 전체 재빌드와 생산자/소비자 동시 재시작 필요.
7패키지/227테스트 통과, 강화한 동시성 부분집합6개 통과.
동일 합성 640×480/65표본 ROS 비교 평균58.50→16.66ms, p95 67.31→17.74ms.
이는 통신·스케줄링 비교이며 실제 모델/네트워크 지연 개선 수치가 아니다.
실제 MuJoCo+YOLO+ICP: 스캔·개방·후퇴40.630초 완료. 적재 확인 없음.
물리 경고0/설비 침투0/weld0, 물체 접촉 침투22.82mm는 남음. 실기 미검증.
별도 /tmp/realtime-install, ROS_DOMAIN_ID212 사용. 기존8090 관제 SIM 유지.
상세: docs/architecture/realtime_perception.rst. 증거: validation/head-rgbd/realtime-integration/.

최신 — Newland 시리얼 barcode_node (2026-10-27)
==============================================

Downloads/barcode의 시작/중지 명령·동글 식별 기준으로 barcode_node 구현.
기존 barcode_bridge_node 대체, REAL은 시리얼 직접 연결. /barcode/raw는 SIM에서만 구독.
BT ScanSession/BarcodeRead 계약 유지. 판독 후 중지, 취소·만료·시계 역행·단절 처리.
단절 후 새 세션 필요, 시작 전 quiet drain, ACK/제어 응답 제외. 기본숫자8·12~14자리.
포트 기본비활성, barcode_enabled/barcode_port/barcode_serial_number launch 인자.
7패키지 빌드/222테스트 통과(실제 pyserial+Linux PTY+ROS 포함). 실제 스캐너 미연결.
최종 빌드 /tmp/barcode-final-install. ROS_DOMAIN_ID232 SIM 두 실행은 스캔 전 conveyor forecast expired로 중단.
전체 SIM 완료 미확인. 기존8090 SIM 유지.
설정/한계/실행: docs/architecture/barcode_scanner.rst. 증거: validation/head-rgbd/barcode-integration/.

최신 — 실행 통합 / 영상 Observation 경량화 (2026-10-27)
=====================================================

arm/hand bridge를 execution 내부 ArmAdapter/HandAdapter로 통합했다.
BridgeMotion 두 Action과 deux_task_adapters 패키지 제거, 실행·피드백·취소 감시 단일화.
Observation에서 관절/물체/EE 필드와 구독을 제거하고 RGB-D·촬영 시각 카메라 TF만 유지.
StateObservation/BT/planner/검출/ICP/barcode_bridge 구조는 유지했다.
메시지 ABI 변경: overlay 전체 신규 빌드 및 생산자/소비자 동시 재시작 필요.
검증은 /tmp/integrated-install 및 ROS_DOMAIN_ID231. 현재8090 SIM은 재시작하지 않았다.
상세: docs/architecture/execution_integration.rst.
7패키지 빌드/193테스트 통과. 최종 SIM41.038초에 스캔·개방·후퇴 완료, 적재 확인 없음.
실제 그래프에서 브리지 제거·execution 직접 명령 확인. 실기 미검증.
증거: validation/head-rgbd/execution-integration/. 대시보드 2절도 현재 경로로 갱신.

실시간 관제와 수거함 적재
========================

헤드 RGB-D BT에 리더 후퇴·수거함 접근·개방·후퇴를 연결했습니다.
사용자 요청으로 적재 확인과 단계 지정 시작은 제공하지 않습니다.
브라우저 GUI에서 현재 단계와 시작/정지/준비자세 이동, 실제 MuJoCo 및 ZED RGB를 볼 수 있습니다.
두 영상은 좌우, 작업 흐름은 아래에 표시합니다.
선택 대상의 동기화된 mask/depth/TF에서 구한 가시 표면3D 범위를 RGB에 투영합니다.
SIM/REAL은 같은 ROS 입력을 사용합니다. 실기 보정·하드웨어 검증 완료를 뜻하지 않습니다.
실행·SSH: ``docs/architecture/task_gui.rst``, 적재: ``docs/architecture/head_placement.rst``.

파지 · 스캔 · 수거함 적재 흐름 (이전 설계 정리)
==============================

현재 헤드 BT의 스캔까지 구현된 흐름과, 기존 수거함에 한 개씩 내려놓는
적재 연결 제안을 구분해 정리했습니다. 단계별 노드/ROS 메시지, 기존 SIM 영상과
로그, 실기 교체·검증 항목은 ``validation/pick-scan-place-flow/dashboard.html``에서
볼 수 있습니다. 상세 문서는 ``docs/architecture/pick_scan_place_flow.rst``입니다.
이번 작업은 문서·시각화이며 헤드 BT 적재 기능을 구현한 결과는 아닙니다.

준비자세 손목 각도를 유지하는 스캔
=================================

왼팔5~7번 관절을 준비 각도로 고정하고1~4번으로 바코드를 조준합니다.
사용자가 허용한 이동 중약13도 기울임 범위로 SIM 스캔/가상 판독까지 진행했습니다.
측정 최대 기울기12.8도, 스캔 자세3.5도, 손목 관절 최대 편차0.20도입니다.
설정/검증 범위: ``docs/architecture/fixed_wrist_scan.rst``.

ScanPose 높이15cm 추가
======================

직전 ScanPose에서 Z축으로15cm 더 높였습니다(최초 대비총약25cm).
XY와 손등 위쪽 방향을 유지하며, 새 높이에서 SIM 왼손 리더 정렬과 가상 판독까지
진행했습니다. 상세: ``docs/architecture/scan_plus_15cm.rst``.

왼손 스캔 정렬까지 실행한 SIM
============================

빠른 Lift와 손등 위쪽 ScanPose 이후 왼손 리더 정렬 및 가상 판독까지 진행했습니다.
스캔 자세로 회전할 때 봉투가 빠지던 문제에 SIM 오른손 유지강도4배를 적용했습니다.
마찰/토크상한/손가락 자세는 유지하며 물체 강제 부착은 사용하지 않습니다.
단일 SIM 결과이고 실기·실영상 바코드 디코딩·반복 성공률 검증은 아닙니다.
상세: ``docs/architecture/scan_attempt_validation.rst``.

빠른 상승과 높인 ScanPose
========================

손 닫힘 후 파지 성공 판정을 기다리지 않고 Lift로 진행합니다.
동일 시작 자세의 상승 계획을4.68→2.18초로 단축했으며 관절 제한을 유지합니다.
ScanPose는 기존보다10cm 높이고 손등이 위를 향하도록 변경했습니다.
SIM에서 해당 동작까지 확인했으나 이후 봉투 재인식은 시간 초과로 종료했습니다.
자세/검증 범위: ``docs/architecture/quick_lift_upright_scan.rst``.

반닫힘 준비와 직접 추종 하강
===========================

준비 완료 시 엄지·검지를 반쯤 닫고, 별도 Track 구간 없이 예측 접근 후
하강하며 컨베이어 이동을 따라가도록 변경했습니다. 손가림 등 관측 소실 시
제한된 예측을 사용합니다. ROS2/MuJoCo에서 하강·닫힘·Lift·ScanPose까지 진행했고
이후 봉투 재인식 시간 초과로 종료했습니다. 전체 파지/스캔 성공은 미검증입니다.
설정과 검증 범위: ``docs/architecture/pregrasp_direct_descent.rst``.

낮춘 준비자세와 예측 접근
========================

영상 기반 sim의 오른손 준비 EE를약7.3cm 낮추고 손을 미리 열도록 변경했습니다.
준비 중 상류 물체를 추적하고, 예측 접근 뒤 새 ICP/Kalman 관측으로
Track→Descend→Close 경로를 다시 계획합니다. GUI에Track 단계를 표시합니다.
설정과 검증 범위: ``docs/architecture/low_ready_intercept.rst``.

진행 중 — ZED 2i 및 이동 컨베이어
==================================

ZED 2i URDF 조립체와 sim RGB-D 입력을 추가했습니다. 하향45°, 카메라 바닥과
눈 디스플레이 상단의 수직 간격3mm이며, 절대 장착 좌표는 sim 추정값입니다.
설정/생성/실기 전환 조건: ``docs/architecture/zed2i_mount.rst``.
장착 미리보기: ``validation/head-rgbd/zed-mount-check/index.html``.
8cm/s 컨베이어 추종은 별도 검증 진행 중이며 아래 scenario-15 성공은 기존
카메라/정지 컨베이어 결과입니다. 새 구성을 실기 검증 완료로 해석하지 마십시오.

최신 헤드 RGB-D PickScan 검증
============================

물체 정답 pose 상시 입력 대신 sim 학습 YOLO11n-seg + ByteTrack, 선택 대상
ICP, 파지 후 영상 확인과 바코드 표면 기반 리더 정렬을 구현했습니다.
scenario-15는34.340초에 세션 판독까지 완료됐습니다.
ICP2회는 집기 전에만 발행됐으며,82개 회귀 테스트와8개 overlay 빌드가 통과했습니다.
실기/실영상 디코딩 검증은 아니며 가상 광학 리더가 fixture EAN을 반환합니다.
현재 동작·학습·증거·실기 전환: docs/architecture/head_rgbd_validation.rst.
영상/메시지: validation/head-rgbd/scenario-15/dashboard-rgbd.html.
MP4: 같은 폴더의 scenario-rgbd.mp4(35.394초).
구조도: docs/architecture/head_rgbd_nodes.html, head_rgbd_bt.html.
아래 legacy truth/이송 검증 내용은 기존 경로와 과거 결과입니다.

브라우저 작업 제어 GUI
=====================

sim/real 공통 ``/task/status`` 및 ``/task/run``에 연결하는 GUI를 추가했습니다.
현재 BT 단계 확인, 작업 시작/중단/재시작만 제공합니다.
실행: ``ros2 run deux_task_gui task_gui --ros-args -p deployment:=sim``.
원격에서는 SSH 터널로 접속합니다. 실행 방법과 제어 범위는
``docs/architecture/task_gui.rst``를 참고하십시오.
기본 head-rgbd 경로의 인터랙티브 sim은 ``ros2 run deux_task_sim serve_sim --sim-repository PATH --output PATH --weights MODEL.pt``입니다.

ROS 2 시뮬레이션 녹화와 메시지 타임라인
======================================

최신 준비 자세 기준은 **후퇴 전 차렷 자세(URDF 관절0)의 EE XY**입니다.
로봇20cm 후퇴 상태에서 팔꿈치를 굽혀 해당 XY를 유지하며, 손끝 높이는 약24.2cm 상승합니다.
어깨·팔꿈치·손을 함께 움직여 영상3.47초에 안정화,3.64초에 Ready가 됩니다.
4초 부근 sim 측정 EE XY 오차 왼쪽0.207mm/오른쪽0.143mm.
새 영상: ``validation/ros2-sim-ready-4s/preparation-recording/dashboard.html``.
이전 ee-preserved 영상은 준비 자세를 기준으로 했던 과거 결과입니다.

최초 준비 이동 추가: 관절0 → 준비 자세, 같은 task_node 프로세스에서 성공 후 재실행 생략.
sim 로봇을 뒤로20cm 이동하고 컨베이어/상자는 유지했습니다.
새 준비 영상: ``validation/ros2-sim-ready/preparation-recording/dashboard.html``.
Jazzy8개 패키지 빌드,56개 테스트 및 준비 이동·유지 시험 통과.
세부 설정과 실기 적용 조건은 ``docs/architecture/simulation.rst`` 마지막 절을 참고하십시오.

미사용 구독 정리 후 재검증: Jazzy8개 패키지 빌드,53개 테스트 통과.
새 sim 기록은 ``validation/ros2-sim-subscriptions/scenario-recording/dashboard.html``입니다.
바코드 수신과 Place 요청까지 동일하게 진행했고 기존 IK 실패가 유지됩니다.
아래 ros2-sim 기록은 변경 전 실행을 보존한 자료입니다.

실기 적용 준비 문서: ``docs/architecture/hardware_rollout.rst``.
현재 커밋 범위/제외 항목, 추가 구현·설정, 단계별 검증 및 통과 조건을 정리했습니다.

MuJoCo 드라이버를 실제 ROS 2 task/planner/execution/bridge에 연결했습니다.
재현: ``bash tools/validate_sim.sh``. 상세 설정과 검증 범위는
``docs/architecture/simulation.rst``를 참고하십시오.
결과 화면: ``validation/ros2-sim/scenario-recording/dashboard.html``.
영상 시각과 노드 연결, 토픽/Action 이벤트, BT 단계를 함께 확인할 수 있습니다.

Jazzy8개 패키지 빌드,52개 회귀 테스트 통과. 기록은 바코드 수신 후 Place 계획의
IK 실패까지이며, 전체 태스크 정상 완료를 확인한 것은 아닙니다.
사용자 요청에 따라 성공 튜닝은 중단했습니다. sim 관측은 가시성 기반6D/라벨 센서이고,
실제 YOLO/ICP 및 하드웨어는 이 실행에서 검증하지 않았습니다.

현재 제어 경로 — 단계별 인식 / Behavior Tree
==========================================

공통 YOLO → 독립 ICP·바코드 표면, 요청 ID/lease 기반 모드 전환,
ScanSearch → AlignBarcode → WaitBarcode, 이미지 없는 StateObservation을 구현했습니다.
설치 YAML은 각 노드에서 직접 읽습니다. 최신 실행 안내와 제약은
``docs/architecture/controlled_perception.rst``를 참고하십시오.
Jazzy7개 패키지 빌드 및49개 합성 테스트 통과. 실제 하드웨어 미검증.
새 구조도: ``docs/architecture/nodes.html``, ``docs/architecture/scenario.html``.

아래 설명 중 기존 observation/detection/ICP 분리 경로는 호환 실행 옵션입니다.

DEUX external-camera observation overlay
========================================

세션 이어가기
-------------

현재 작업 인계와 다음 detection/ICP 구현 자료는 ``docs/HANDOFF.rst``를 먼저 읽으세요.
구조도는 ``docs/architecture/architecture.html``, 상세 설계는 ``docs/architecture/design.html``입니다.

현재 구현 범위
--------------

이 폴더는 DEUX/zed-ros2-wrapper 원본을 수정하지 않는 독립 ROS 2 overlay입니다.
관측/검출/ICP와 설계의 나머지6개 노드를 포함한9개 노드를 구현했습니다.
현재 구현·실행 설정·검증 한계는 ``docs/architecture/runtime.rst``가 우선합니다.
팔/손 명령 브리지가 포함되지만 기본 launch는 명령 전송을 비활성화합니다.
기존 bringup·카메라 드라이버·teleop·리프트를 시작하지 않습니다.

* RGB, 등록된 depth, 각각의 CameraInfo를 ApproximateTimeSynchronizer로 동기화.
  기본 크기 640x480, 시각 차이 최대20ms. best-effort sensor-data QoS.
* 입력 RGB: rgb8/bgr8/rgba8/bgra8. 출력 rgb8.
* 입력 depth: 32FC1=미터, ZED OpenNI mono16/16UC1=밀리미터.
  출력 32FC1 미터, 0/음수/NaN/Inf는 NaN. 유효 depth 비율을 함께 전달.
* row padding/endianness 처리. 해상도·frame·투영행렬·CameraInfo 검증.
  같은 해상도/optical frame/등록 투영이 아닌 영상은 거부합니다.
  확대/축소/크롭은 하지 않습니다. 위쪽 별도 카메라의 실제 frame_id 유지.
* 원래 영상/관절/ICP 촬영시각을 보존. 오래된·미래·중복 RGB를 거부하고
  관절/pose는 해당 RGB 시각 이하의 최근 측정만 선택합니다.
* TF가 없어도 영상 발행. camera_transform_valid=false이면 base_from_camera
  내용은 사용 금지. identity를 보정 결과로 꾸미거나 head camera를 사용하지 않음.
* 선택 입력: /arm/joint_states + /task/hand_state → 이름 기준 28관절 정렬.
  한쪽이 없거나 오래됐으면 joints_valid=false. 명령값으로 측정을 대체하지 않음.
  원본 arm_state/hand_state 별도 stamp 보존. 합친 joints stamp는 가장 오래된 값.
* 선택 입력: /task/object_detections (신규 ObjectDetectionArray).
  이미 base_frame으로 변환된 ICP pose, track_id, 원래 촬영 stamp/sequence,
  model_id, covariance, full_orientation을 보존. source/header는 덮어쓰지 않음.
* 선택 EE: left_ee_frame/right_ee_frame을 실제 보정된 grasp frame으로 지정하면
  이미지 시각의 TF를 전달. 비어 있으면 각 ee_valid=false. 손목→접점 보정은 별도.
* /task/observation/diagnostics는 입력 대기·누락·거부 원인을 전달.
  이 진단의 OK는 RGB-D 수신 상태일 뿐 로봇 동작 가능/성공을 뜻하지 않습니다.

observation_node의 객체 입력은 검증 후 원본 전달입니다. 별도 detection_node와
icp_node가 검출·yaw ICP·거리 기반 ID 연결을 수행합니다. 선택적 XY/yaw Kalman 필터와 별도 바코드 브리지도 구현했습니다.
Kalman 필터는 설정 파일에서 ``filter_objects: true``일 때만 적용합니다. 기존 sim tracker를 연결하는 경우
동일 pose sequence/header를 재갱신하지 않아야 합니다. 본 Observation은 RGB-D 및
선택 측정 묶음이며 기존 JSON ObservationFrame과 직접 호환되는 메시지는 아닙니다.
신규 runtime은 ROS 메시지를 직접 사용합니다. legacy JSON adapter는 시각/관측축을
잃지 않는 입력만 명시적으로 허용합니다.

Detection / ICP 추가 (2026-10-24)
--------------------------------

실행 설정, 현재 3노드 데이터 흐름, 좌표·모델 원점 및 검증 범위는
``docs/architecture/perception.rst``를 참고하십시오. 기존 HTML 구조도는 이전 설계
스냅샷이며 현재 구현 구조는 이 문서가 우선합니다.

``config/perception.yaml``의 weights, model_path, model_origin, reference_frame을
명시적으로 채운 뒤 별도 launch로 실행합니다. 보정 TF가 없으면 base pose를 발행하지
않습니다. YOLO 설치는 선택 의존성이며 ROS 실행 Python 환경에 별도로 필요합니다.
기존 observation launch만 실행할 때는 YOLO가 필요하지 않습니다.

사용한 YOLO 체크포인트는 ``models/deux_1021_A_v2.0only_adamw.pt``에 포함되어
있습니다. 설치별 weights 설정에 해당 파일의 절대 경로를 지정하십시오.
모델 SHA256과 클래스 매핑은 ``models/README.rst``에 기록했습니다.

ICP 검증에 사용한 빵 봉투 v5 OBJ는
``models/breadbag_model_v5/mesh/opaque_envelope.obj``에 포함되어 있습니다.
참조 재질과 텍스처도 포함하며, model_path와 model_origin 설정은
``models/README.rst``를 참고하십시오.

촬영 프레임의 depth를 DetectionFrame에 함께 전달하며, 추론/ICP worker는 실행 중
1프레임과 최신 대기 1프레임으로 제한합니다. yaw-only 결과는 full_orientation=false,
설정 공분산은 covariance_estimated=false입니다. ObjectDetection의 ICP 품질 필드
추가로 해당 메시지를 사용하는 외부 패키지도 재빌드해야 합니다.


Runtime 추가 (2026-10-24)
------------------------

``scene_node`` 대신 task/planner/execution/arm/hand 노드가 각자 ``config_file``의
설치 YAML을 시작 시 직접 읽습니다. 설정 배포 토픽과 reload 서비스는 없습니다.
launch의 ``scene_config`` 인자로 동일 파일을 지정하며, 수정 후 관련 노드를 함께
재시작합니다. 설정·URDF 버전 비교는 Action에서 유지합니다.

``deux_task_core``, ``deux_task_runtime``, ``deux_task_adapters``, ``deux_task_bringup``을
추가했습니다. 설치 YAML, 손 매핑 및 실행 명령은 ``docs/architecture/runtime.rst``를
참고하십시오. 실제 파지 후6D 관측과 설치 보정값은 아직 공급되지 않았으며,
yaw-only ICP를 완전한 자세로 간주하여 Scan/Place를 실행하지 않습니다.

이전 MuJoCo 성공 궤적과의 동등성/실기 성공률은 미검증입니다. 새 계획기는 실기
expanded URDF 기반 IK와 명시된 collision envelope를 사용하며, 새 Action 및
합성 드라이버로 통신·실패 처리를 검증합니다.

ZED 레포 기준 입력
------------------

검사한 소스:
/home/xyz-ai/git/zed-ros2-wrapper/zed_components/src/zed_camera/src/zed_camera_component_video_depth.cpp
initVideoDepthPublishers의 sens_rgb="rgb/", color="color/", rectified="rect/",
type_image="image" 및 depth/depth_registered를 기준으로 기본값을 정했습니다.
이 checkout의 RGB 토픽은 구형 rgb/image_rect_color가 아닙니다.

기본 camera_name=zed, node_name=zed_node, namespace 미지정 기준::

  /zed/zed_node/rgb/color/rect/image
  /zed/zed_node/rgb/color/rect/camera_info
  /zed/zed_node/depth/depth_registered
  /zed/zed_node/depth/camera_info

namespace를 직접 지정하면 ZED launch가 node_name을 camera_name으로 변경하는
분기가 있습니다. 실제 ``ros2 topic list``를 확인하고 config 경로를 교체하십시오.
카메라 모델/실행 중 namespace는 이 환경에서 확인되지 않았습니다.

640x480은 상류에서 등록된 RGB/depth와 보정 CameraInfo가 함께 제공된다는 계약입니다.
ZED의 pub_downscale_factor 하나로 모든 모델에서 정확히640x480을 만들 수 있다는
의미가 아닙니다. 다른 종횡비 입력을 단순히 640x480로 늘이지 않습니다.
리사이즈가 필요하면 상류에서 두 영상과 K/P를 동일한 규칙으로 변환해야 합니다.
기본 CameraInfo는 각 프레임과 같은 시각에 발행돼야 합니다. latched/static
CameraInfo만 있는 다른 생산자라면 별도의 명시적 동기화 처리가 필요합니다.

실행 (ROS 2 Jazzy 호스트)
------------------------

기존 DEUX와 ZED 드라이버는 별도로 실행돼 있다고 가정합니다.
이 명령은 기존 소스를 수정하지 않고 이 overlay의 build/install/log만 생성합니다::

  source /opt/ros/jazzy/setup.bash
  source /home/xyz-ai/git/DEUX/install/setup.bash
  cd /home/xyz-ai/git/DEUX_ikworld
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --symlink-install
  source install/setup.bash
  ros2 launch deux_task_perception observation.launch.py

별도 설치 설정 사용::

  ros2 launch deux_task_perception observation.launch.py config:=/absolute/path/observation.yaml

rosbag 재생 시 ROS clock 사용::

  ros2 launch deux_task_perception observation.launch.py use_sim_time:=true
  ros2 bag play /path/to/bag --clock

전체 이미지가 포함된 /task/observation은 단일 로컬 소비자를 우선 대상으로 합니다.
640x480 rgb8+float depth는 약2.15MB/frame(30Hz 약64.5MB/s, DDS 오버헤드 별도)입니다.
불필요한 네트워크 복제/전체 메시지 echo를 피하십시오. 실제 측정 프레임율에 맞춰
queue/age 설정을 조정하며, 제어 실행기는 별도로 최신성을 판단해야 합니다.

입력 물체 메시지 규약
----------------------

deux_task_msgs/msg/ObjectDetectionArray:

* header.frame_id: base_frame(기본 base_link). 카메라 좌표 입력은 거부.
* header.stamp: 원본 촬영 시각, 결과 도착 시각 아님.
* sequence: 생산자가 부여한 관측 sequence. source/model_id: 추정기와 봉투 원점 버전.
* detections: 고유 track_id, pose(PoseWithCovariance), confidence [0,1], full_orientation.
* ICP가 yaw만 추정하면 full_orientation=false. 정답처럼 roll/pitch를 표시하지 않음.
* covariance는 유한한 대칭 양의 준정부호. 전부0을 추정 정확도 보장으로 해석하지 않음.
* objects_valid=true는 시각/형식 유효성만 뜻함. 비어 있는 detections는 관측 결과 없음.
* 동일 관측이 여러 카메라 frame에 붙을 수 있으므로 downstream은 원본 sequence/stamp로 중복 제거.

손 데이터는 새 /task/hand_state에 실제 측정 JointState로 정규화해야 합니다.
원격 손 드라이버의 실제 피드백 토픽·모터ID 매핑은 아직 미확정입니다.

검증
----

2026-10-24 runtime 추가 검증: overlay6개 패키지와 기존 deux_hand_msgs를 포함한
7개 패키지 빌드, 전체41개 테스트 통과. 실제 bridge와 합성 드라이버의 정상 실행·
취소·피드백 단절 및 태스크 실패/재시도 경로를 포함합니다.

이전 perception 단계 검증: ROS 2 Jazzy에서 두 패키지 빌드, 24개 테스트 통과.
신규 테스트에는 합성 검출기→실제 yaw ICP의 ROS 통신과 TF 누락·만료 거부가
포함됩니다. 저장 데이터 ICP 비교와 실제 YOLO 단일 이미지 추론 결과는
``validation/icp-replay.json``, ``validation/yolo-replay.json``에 있습니다.
YOLO의 Jazzy 환경 통합 실행 및 실제 ZED/DEUX 하드웨어 동작은 미검증입니다.

ROS 환경에서 빌드 후::

  source install/setup.bash
  python3 -m pytest src/deux_task_perception/test src/deux_task_core/test src/deux_task_runtime/test -q

ROS 통신 테스트(test_ros_smoke.py)는 640x480 BGRA와 OpenNI depth, CameraInfo를
합성 발행해서 Observation 수신/변환 및 URDF/관절 없이 수신 가능함을 검증합니다.
로봇 명령을 발행하지 않습니다. 실제 드라이버와 테스트는 별도 ROS_DOMAIN_ID에서
실행하십시오. 예: ROS_DOMAIN_ID=91 python3 -m pytest src/deux_task_perception/test src/deux_task_core/test src/deux_task_runtime/test -q

2026-10-23 검증: 호스트에는 ROS 2가 없지만 ros:jazzy-ros-base 컨테이너에서
Python 3.12.3 / ROS_DISTRO=jazzy로 colcon build 2개 패키지 성공,
테스트17개 모두 통과(ROS RGB-D 실제 pub/sub 합성 통신 테스트 포함).
실제 ZED 카메라·DEUX 하드웨어 연결은 아직 미검증입니다.

재현: bash tools/validate_jazzy.sh
로그: validation/jazzy-container.log
컨테이너: ros:jazzy-ros-base
검증 당시 image digest:
sha256:103b6c417973394659af6804d4917cd489b91e5ed22eb7bef263025a3bed2410
소스는 컨테이너에 read-only로 마운트하고 내부 임시 workspace에서 빌드합니다.
장치와 host network를 연결하지 않으며 종료 후 컨테이너는 제거합니다.
