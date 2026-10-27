최신 — 누적 기능 커밋 및 실물 검출 모델 포함 (2026-10-27)
=======================================================

실행 브리지 통합, 인식 통합·최신 큐, 관절 중계 제거, 이동 파지·후보 검증,
준비/스캔/적재 BT, 선택형 IK(기본 PyRoKi), 시리얼 바코드, 웹 관제,
ZED2i/SIM 실행 코드와 테스트·모델·설계 문서를 함께 커밋한다.
tools/validation 및 구조도 검수 산출물은 이번 커밋에서 제외한다.
실물 검출 모델 deux_1026_finetuned_from1021.pt 추가: detect, b1/m1/s1/s1_1/bar.
로딩·빈 영상 CPU 추론 확인. SIM segmentation 모델과 클래스/mask 계약이 다르므로
실기 head RGB-D 설정을 자동 변경하지 않는다. 상세 models/README.rst.
원격 main의 홈 자세·팔 점검 도구 변경을 보존하여 통합 후 검증한다.

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
보고서 차트·필터·모바일 검사 완료, 브라우저 열기 성공. 상세 architecture/planner_backends.rst.
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
상세 architecture/grasp_candidates.rst. overlay /tmp/candidates-install.
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
증거 validation/head-rgbd/ik-workspace/. 상세 architecture/ik_workspace.rst.
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
증거 validation/head-rgbd/roi-selection/. 상세 architecture/rgb_roi_selection.rst.
8090 관제는 비활성 확인 후 새 설치로 재시작, 기존 토큰 유지, RGB ROI/두 영상 확인 후 열었다.
restart_live.py는 /tmp/roi-install 사용. 로컬 전달 프로세스는 detached 실행.

최신 — 통합 코드 관제 SIM 재빌드·재시작 (2026-10-27)
====================================================

사용자 요청으로 7패키지를 /tmp/realtime-live-install에 재빌드하고 기존 작업 비활성 확인 후
8090 SIM을 재시작했다. ROS_DOMAIN_ID230, 기존 접속 토큰 유지(placement/live.log).
브라우저를 토큰 포함 링크로 열었고 Idle·MuJoCo/ZED RGB 최신 영상·GUI 배치 검사 통과.
실제 그래프: perception_node 통합, detection_frames 발행자 없음, 플래너 관절 직접 구독,
StateObservation 관절 배열 제거 확인. 자동 작업은 시작하지 않았다.
로컬 TCP 전달 local_forward.py도 복구했다. 재시작 도구 restart_live.py는 새 설치 사용.
증거: validation/head-rgbd/realtime-integration/live-build.log,
validation/head-rgbd/placement/live-browser-check.json. 기존8080 유지.

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
상세: architecture/realtime_perception.rst. 증거: validation/head-rgbd/realtime-integration/.

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
설정/한계/실행: architecture/barcode_scanner.rst. 증거: validation/head-rgbd/barcode-integration/.

최신 — 실행 통합 / 영상 Observation 경량화 (2026-10-27)
=====================================================

arm/hand bridge를 execution 내부 ArmAdapter/HandAdapter로 통합했다.
BridgeMotion 두 Action과 deux_task_adapters 패키지 제거, 실행·피드백·취소 감시 단일화.
Observation에서 관절/물체/EE 필드와 구독을 제거하고 RGB-D·촬영 시각 카메라 TF만 유지.
StateObservation/BT/planner/검출/ICP/barcode_bridge 구조는 유지했다.
메시지 ABI 변경: overlay 전체 신규 빌드 및 생산자/소비자 동시 재시작 필요.
검증은 /tmp/integrated-install 및 ROS_DOMAIN_ID231. 현재8090 SIM은 재시작하지 않았다.
상세: architecture/execution_integration.rst.
7패키지 빌드/193테스트 통과. 최종 SIM41.038초에 스캔·개방·후퇴 완료, 적재 확인 없음.
실제 그래프에서 브리지 제거·execution 직접 명령 확인. 실기 미검증.
증거: validation/head-rgbd/execution-integration/. 대시보드 2절도 현재 경로로 갱신.

최신 — DEUX 실기 연결 도식 보강 (2026-10-27)
===========================================

validation/pick-scan-place-flow/dashboard.html 2절에 단계별 DEUX 노드·메시지를 추가했다.
DEUX launch/자체 노드 보라색, overlay 청록색, 외부 장비 회색. 컨트롤러·관절 상태·TF·상태 gate와
DEUX HandCtlMessage/MotorData 정의를 원본 소스로 확인. 손/스캐너 생산자는 DEUX 레포 밖이다.
state_node 기본 arm publisher는 hardware_gate와 충돌하므로 실기 명령 경로 격리 필요를 표시했다.
과거 BT·SIM 영상은 그대로이며 적재 구현 전 스냅샷임을 표시하고 현재 head_placement.rst로 연결.
생성: build_report.py + deux_connections.py. 근거: deux-sources.json. 브라우저: check_deux.cjs.
DEUX 소스 수정·실기 실행·현재 SIM 재시작 없이 정적 문서만 변경했다.

최신 — TrackAndSelect 상세 사유 (2026-10-27)
===========================================

select_ready로 기존 선택 판정을 분리하고 모든 대기 분기에 TaskStatus.reason을 추가했다.
검출/시각/봉투/신뢰도/ID/3D 후보/요청 일치/깊이/영역/관측 횟수 구분.
혼합 후보의 거절 사유별 개수, 영역 밖 예시 XYZ·허용 범위, 관측 n/3회를 표시한다.
선택 성공 시 사유 초기화,8초 timeout에 마지막 사유 보존. 선택 순위와 지정 ID 정책 유지.
8패키지 빌드,186테스트 및 GUI 상태 재생으로 대기/실패 사유 표시·중복 억제 확인.
실기 및 전체 시나리오 재검증은 하지 않았다. tests-selection-reason.log 참조.

최신 — 접근 예측 시간 / 불충족 사유 (2026-10-27)
================================================

EstimatePose entry_lookahead=7초 제거. 두 Action 시작 여유 + Approach + 합류 대기 +
Track/Descend/Close 합산(현 SIM5.1초)에 관측 경과 시간을 추가해 완료 위치 검사.
플래너와 같은 conveyor_velocity 사용. 실제 재시간화 이후 영역/시간 검사는 유지.
이는 계획 전 예상 시간이며 실제 IK 경로시간은 이후 플래너에서 재검사한다.
TaskStatus.reason에 ICP/필터/횟수/속도/예측 좌표 등 불충족 사유를 출력하고 timeout에도 보존.
GUI 현재 단계 아래 표시, 중복 종료 메시지 제거. 회귀167개 통과.
상세: docs/architecture/conveyor_tracking.rst. 자동 재선택 정책은 변경하지 않음.

최신 — 적재 실행 / 실시간 관제 / ZED RGB 3D 박스 (2026-10-26)
=============================================================

사용자 최신 지시: 적재 구현, 단계 지정 시작 제외, 적재 확인 제외.
GUI는 상단 현재 단계/제어, 가운데 왼쪽 MuJoCo·오른쪽 ZED RGB, 맨 아래 작업 흐름이다.
재시작 버튼 제거, 준비자세로 이동 버튼 추가. /task/move_to_ready(MoveToReady) 전용 Action.
RunTask와 공유 admission lock으로 동시 실행 차단. 기존 작업 terminal + 새 inactive 상태 후 복귀.
최초 Prepare, 이후 손 상태를 유지하는 ReadyPose. 자동 작업 시작/단계 선택 없음.
RunTask 인터페이스는 target_id만 유지. 단계 지정 시작 시도 변경은 전부 되돌렸다.

HeadTaskNode placement.enabled 시 스캔 → ClearReader → PlaceApproach → Place →
Release → PlaceRetreat → Retreat → Succeeded. VerifyPlace는 제거했다.
성공은 판독 및 개방/복귀 동작 완료이며 적재 확인 증거를 의미하지 않는다.
고정 EE 경유/개방 pose 사용, stale 파지 전 ICP 재사용 없음.
planner/executor에 리더 후퇴·개방 위치·손 개방 후 후퇴 조건 검사.
SIM hover=[.208,-.560,.865], release=[.208,-.560,.785]m. 수거함 벽/바닥 장애물 추가.
실기는 placement.calibrated=true 및 실측 설정 필요. 하드웨어 검증하지 않음.

placement/run-03: ClearReader23.596 → PlaceApproach26.350 → Place29.206 →
Release33.184 → PlaceRetreat35.938 → Retreat40.018초. 영상상 봉투 수거함에 놓임.
이 기록에는 당시 VerifyPlace가 있어 관측 소실로 종료. 이후 사용자 요청으로 해당 단계 제거.
run-01은 낮은 개방 자세에서 엄지 feedback limit, run-02는 기존 forecast expiry 중단.
run-03 물리 경고0/설비 침투0/weld0, 물체 접촉 침투23.07mm. 반복 성공률 미검증.
영상: validation/head-rgbd/placement/run-03/dashboard-rgbd.html.
최신8패키지 빌드/153테스트 통과. SIM 준비 버튼: Prepare→Ready, ReadyPose→Ready 확인.
측정 팔 관절 목표 최대오차0.00078rad, 새 파지 작업 자동 실행 없음.
브라우저 데스크톱/모바일 배치 및 두 영상 확인. ready-sim-check.json, live-browser-check.json 참조.
이전 장시간 SIM은 t=681.620초 QACC 물리 경고로 종료, 종료 확인 후 재구동했다.
새 준비 동작 시험에는 경고 없음. 장시간 물리 안정성은 별도 미해결 사항이다.

실행 중인 GUI: http://127.0.0.1:8090/ (기존8080 유지), ROS_DOMAIN_ID230.
deux-head-rgbd 컨테이너 serve --gui의8090을 호스트loopback8090으로 전달.
live.log에 토큰이 있으며 재시작 시 같은 토큰을 보존했다. 토큰을 문서에 복사하지 않는다.
SIM은 첫 RunTask 승인 전 컨베이어 정지, 첫 시작 후8cm/s로 계속 움직인다.
작업 중 무단 초기화 금지. restart_live.py는 active/owned면 프로세스를 보존한다.

GUI /api/frame은 실제 MuJoCo JPEG, /api/rgb는 ROS /task/observation의 RGB이다.
RGB는 SIM/REAL 같은 입력 계약. 선택 target_id의 같은 sequence/stamp 검출 mask와
registered depth + 해당 시각 TF로 base-frame 가시 표면3D 범위를 만들어 RGB에 투영한다.
ICP full object box/시뮬 정답 pose가 아니며 제어 입력으로 사용하지 않는다.
깊이/TF/동기/선택 대상 부재 시 박스 숨김, 오래된 RGB도 숨김. 버퍼8프레임 제한.
실기 ZED 연결·SSH 하드웨어 시험은 미실시. 상세: head_placement.rst, task_gui.rst.
새 커밋/푸시 없음. 많은 기존 작업 트리 변경은 보존했다.

최신 — 수거함 적재 BT / 메시지 흐름 정리 (2026-10-26)
=====================================================

사용자는 기존 수거함에 한 개씩 내려놓기를 선택했다. 우선 요청한 흐름 정리 완료.
런타임 코드는 변경하지 않았다. 현재 HeadRgbdPickScan은 판독 후 종료하며,
ClearReader → Place → Release → Retreat → VerifyPlace 연결은 제안이다.
기존 TaskNode의 PlaceAndVerify와 현재 head BT의 구현 범위를 구분했다.
기존 Place는 fresh held-object pose가 필요하므로 파지 전 ICP 재사용 불가.
단일 함에는 보정된 EE 경유/개방 pose와 목적지 관측 검증을 우선 제안했다.

문서: docs/architecture/pick_scan_place_flow.rst.
대시보드: validation/pick-scan-place-flow/dashboard.html.
Archify: 같은 폴더 behavior-tree.html. 단계 선택, 메시지 강조, 기존 SIM 영상/로그 동기화.
스캔 증거는 fixed-wrist-scan/run-01 재사용. 적재 구간 영상/시간은 생성하지 않았다.
실기 교체·검증 항목 포함. 새 SIM/실기 실행, 커밋/푸시 없음.

최신 — 준비 손목 고정 스캔 (2026-10-26)
======================================

사용자: 준비자세 손목 각도 유지, 이동 중약13도 기울임 허용 후 우선 영상 확인.
barcode_fixed_wrist 설정으로 왼팔5~7번 목표 고정,1~4번으로 관측 바코드 조준.
손목 명령 변화0도, 측정 최대 편차0.20도. 이동 기울기12.77도, 스캔3.46도.
reader_alignment.py 추가, planner/executor에 경로/측정 손목·기울기 검사.
도달 불가 시 손목 자유 IK로 자동 전환하지 않는다. 오른손 높은 ScanPose 유지.
8패키지 빌드/125테스트 통과. fixed-wrist-scan/run-01:
AlignBarcode20.604→WaitBarcode23.392→Succeeded23.698초, 가상 판독2087231250396.
물리 경고0, 설비 침투0, 물체 침투23.12mm, weld0. 실기/반복 성공률 미검증.
상세: docs/architecture/fixed_wrist_scan.rst.
영상: validation/head-rgbd/fixed-wrist-scan/run-01/dashboard-rgbd.html.
측면 자세: 같은 폴더scan-closeup-90.png. 커밋/푸시 없음.

최신 — ScanPose Z축15cm 추가 상승 (2026-10-26)
=============================================

직전 ScanPose에서Z+15cm(최초 대비총약25cm), XY/손등 위쪽/손가락 목표 유지.
head_calibration scan_right_joints 갱신. 새 EE 목표=[.674984,-.096723,1.068591]m.
경로/관절 제한/충돌 검사 및 sim 패키지 빌드 통과.
scan-plus-15cm/run-01: AlignBarcode20.808→WaitBarcode23.834→Succeeded24.140초,
가상 리더2087231250396 응답. 측정 높이+14.97cm, XY0.18mm, 손등 오차0.08도.
물리 경고0, 설비 침투0, 물체 접촉 침투23.40mm. 실기/반복 성공률 검증 아님.
상세: docs/architecture/scan_plus_15cm.rst.
영상: validation/head-rgbd/scan-plus-15cm/run-01/dashboard-rgbd.html.
커밋/푸시 없음. installed overlay 갱신, 기존 GUI preview는 재시작하지 않음.

최신 — 왼손 스캔 정렬/가상 판독 완료 (2026-10-26)
=================================================

scan-attempt/run-01: Lift12.240→ScanPose15.232→재인식20.196→
AlignBarcode20.706→WaitBarcode23.868→Succeeded24.174초.
왼손 리더 정렬 완료, 가상 리더2087231250396 응답. 실제 이미지 디코딩/실기 아님.
기존 스캔 회전 중 봉투 이탈은 SIM 오른손 유지강도4배(Kp24/Kd1.2)로 개선.
마찰3/토크상한5N·m/손가락 목표/빠른 Lift/손등 위쪽+10cm ScanPose 유지.
8패키지 빌드/123테스트 통과. 물리 경고0, 설비 침투0, 물체 접촉 침투22.79mm,
활성 weld0. 접촉 관통 및 반복 성공률은 미검증. 물체 강제 부착 사용하지 않음.
상세: docs/architecture/scan_attempt_validation.rst.
영상: validation/head-rgbd/scan-attempt/run-01/dashboard-rgbd.html.
스캔 자세: 같은 폴더13-WaitBarcode.png. 커밋/푸시 없음.
현재 installed overlay 갱신. 기존 GUI preview는 재시작하지 않음.

최신 — 빠른 Lift / 손등 위쪽 ScanPose +10cm (2026-10-26)
======================================================

Lift 전 파지 성공 판정 없이 손 닫힘 완료 후 바로 상승한다.
Lift 높이13cm 유지, IK 간격4cm/최소구간0.06초/예약시작0.15초로 지연 축소.
동일 입력의 계획4.68→2.18초. 관절 속도/가속도 및 충돌 검사 유지.
ScanPose는 기존 EE XY 유지, Z+10cm, 손등(right_hand_base_link -Y)을 위로 정렬.
고정 scan_right_joints 갱신. 새 목표 EE=[.674890,-.096768,.918668]m.

8패키지 빌드/119테스트 및 실제 URDF 경로 검사 통과.
SIM run-02: Close10.676→Lift12.376→ScanPose15.334→ReacquireHeldTarget20.230초.
실행 Lift2.958초, 측정 ScanPose 높이+9.98cm/손등 수직 오차0.11도.
이후 재인식 시간 초과28.254초. 전체 성공/실기 검증 아님.
물리 경고0, 설비 침투0, 물체 접촉 침투7.25mm. 파지 안정성은 미검증.
상세: docs/architecture/quick_lift_upright_scan.rst.
영상: validation/head-rgbd/quick-lift-restored/run-02/dashboard-rgbd.html.
run-01은 Lift 이전 기존 conveyor forecast expired로 실패. 감시 완화하지 않음.
커밋/푸시 없음. overlay 갱신했으나 기존 GUI preview는 재시작하지 않음.

최신 — 반닫힘 준비 / Track 없는 예측 하강 (2026-10-26)
====================================================

이 절이 아래 과거 기록보다 우선합니다. 커밋/푸시 요청 없음.
복원한 엄지·검지 파지 유지, third 관절 고정. 준비 완료 시 반닫힘.
기존 준비 경유점은 유지. Track=0, 직접 추종 하강→닫힘 연속 궤적 적용.
Approach는 도착 이후1초 미래 지점 예측, 실제 하강은 물체 근접 조건으로 시작.
접근 관측 캐시는 최대1초, 손근처 하강/닫힘은 최대3초. 원 시각/불확실성 검사 유지.

8패키지 빌드,114테스트 통과. 단독 닫힘1.74→1.24초.
SIM run-05: Approach8.296→Descend10.846→Close12.036→Lift13.736→ScanPose19.618초.
Track 동작 없음. ReacquireHeldTarget25.364→관측 시간 초과33.388초.
파지 성공/전체 스캔 성공/실기 검증 아님. 물체 접촉 침투23.35mm가 남음.
상세/설정/검증 한계: docs/architecture/pregrasp_direct_descent.rst.
영상: validation/head-rgbd/pregrasp-direct/run-05/dashboard-rgbd.html.
실행 overlay /tmp/moving-install 갱신. 기존 GUI preview는 재시작하지 않음.

최신 — 낮춘 준비자세 / 예측 접근·추종 (2026-10-26)
=================================================

이 절이 아래 과거 기록보다 우선합니다. 커밋/푸시 요청 없음.
사용자 요청: 봉투에 닿지 않게 준비 손을 낮추고 이동을 예측해 접근→추종→파지.
상세 설정/증거/제한은 docs/architecture/low_ready_intercept.rst 참고.

최종 head_calibration의 오른손 EE 변화는 X+74.1mm,Y+149.8mm,Z-73.6mm,
손목 방향 변화0.30°. 준비 중 손을 미리 열고 봉투 최고점 대비 수직 여유34.9mm.
기존 HOME_JOINTS는 보존. 준비 중 상류 물체를 선택해 ICP/Kalman 속도 추정,
Approach는 미래 도착점으로 이동하고 도착 후 새 관측으로 TrackGrasp 재계획.
Track→Descend→Close 연속 경로, Jacobian 속도 계산과 단계별 시간 조정 적용.
8cm/s는 필터 초기 prior이며 실제 관측이 이후 속도를 보정함.

ObjectDetection에 Kalman 8×8 motion_covariance 추가(메시지 재빌드 필요).
열린 손 Track/예약 시작 대기만 최대0.8초 bounded prediction 허용.
원 취득시각 보존, 위치편차+2σ≤25mm 및 영역/손 열린 상태를 검사.
하강/닫힘에는 예외 없음. state_observation은 새 관측 즉시 발행.

최종 validation/head-rgbd/low-intercept-17:
Prepare0.102 → EstimatePose6.188 → Approach6.392 → TrackGrasp8.942 →
Track9.146 → Failed9.894초. 손가림 예측 불확실성이 제한을 넘어 중단.
하강/파지까지 정상 동작한 결과가 아님. 부분가림 ICP 연속성/편향이 남은 문제.
허용오차 확대나 원 취득시각 변경으로 통과시키지 말 것.

실제 접근 EE 경로19.98cm, 이전 zed-moving-07의37.76cm 대비약47% 감소.
서로 다른 도착점/시간의 시나리오 비교이며 접근시간은2.516초 대2.380초.
준비 후 대기/접근/Track 최소 mesh 거리24.93/3.28/3.96cm(저장샘플 검사).
설비/물체 침투0, physics warning0,weld0. 실기 검증 아님.
녹화 scenario-rgbd.mp4는10.914초, dashboard-rgbd.html에 메시지/전후 비교.
8패키지 빌드 build-low-intercept-final17.log,108테스트 regression-low-intercept-final17.log 통과.
기존 GUI preview는 legacy이며 이번 구성으로 재시작하지 않았음.

최신 — ZED 2i 적용 완료, 이동 파지 검증 미완료 (2026-10-26)
=========================================================

이 절이 아래 과거 진행 기록보다 우선합니다. 커밋/푸시 요청 없음.
실행 중인 sim/렌더링 없음. 기존 GUI preview는 legacy 스택이며 새 구성으로
교체했다고 주장하지 말 것. 새 요청 범위: 기존 준비자세 복원,8cm/s 컨베이어,
tracking→ICP→Kalman/급변 거부/오른손 진입 영역, 이후 ZED 2i 카메라 적용.

사용자 확인: ZED 2i, 아래45° pitch, 카메라 바닥↔눈 디스플레이3mm.
장착 기준점의 절대 좌표와 전후 위치는 미제공. config/zed2i_mount.yaml에
sim 추정값과 calibration_confirmed:false 명시. 실제 calibration이 아님.
카메라 조립체는 core/camera_mount.py로 원본과 다른 URDF에 추가.
bringup/meshes의 공식 Stereolabs STL/라이선스 포함. world.load_sim은 외부
write_fixed_urdf/add_robot_extras를 생성 중에만 wrapper하고 원복함.
기존 head_camera_pose와 별도의 ZED TF 및 zed2i_left 렌더 카메라 사용.
RGB/depth/CameraInfo/Observation은 zed_left_camera_frame_optical.
sim RGB-D는3tick마다 약9.8Hz; 실제 stereo depth 노이즈는 재현하지 않음.

검증: URDF/MuJoCo 광학 transform 오차3.33e-16, baseline0.12m,
mesh 최저점의 display 상단 수직 간격0.003m, 광축 [0.7071,0,-0.7071].
새 시야에 기존 sim YOLO11n-seg가 봉투4/바코드3 검출(한 장의 확인이며
정확도 검증 아님). ByteTrack/선택ICP/필터/연속경로까지 ROS sim 실행.
8개 빌드 build-zed-integrated.log,88개 테스트 regression-zed-integrated.log.

최신 실행 validation/head-rgbd/zed-moving-07:
Prepare0.102, EstimatePose5.746, TrackGrasp7.038, Approach7.208,
Descend9.622, Close10.846, Failed11.220. 예측/관측 위치편차25mm 초과로 중단.
정지조건/관절한계/관측유효시간을 늘리지 않았음. 카메라 적용은 검증됐으나
이동 파지 전체 정상 동작은 아직 아님. 설비/물체침투0, physics warning0,weld0.
준비 완료 팔 관절 오차 최대0.0006494rad. integration-audit.json 참고.

오프라인 qpos 대조(pose-audit.json, 온라인 입력 아님): 손이 접근해 가리는
구간에서 filtered ICP가 실제 bread_1보다 Y로최대약19.4mm 뒤처짐.
물체는 계속8cm/s로 움직임. 급격한 spike가 아니라 누적된 부분가림 편향이므로
단순 Kalman/급변 게이트로 해결되지 않았음. 허용오차 확대해서 통과시키지 말 것.
다음 핵심 작업: 부분가림에서 ICP의 temporal prior/가시성/비등방 공분산 검토,
관측 모호성 처리와 제한된 재계획. 현재 자동재계획/매프레임 visual servo 없음.

이전시험05는Lift까지 갔으나 엄지첫관절이 상한+.001rad를0.000166rad 초과해
hand bridge가 정지. thumb_3/bag 침투도약22.7mm가 있어 정상 파지 증거 아님.
sim hand_open/closed의 오른손 thumb1 목표를0→-.02rad로 변경해 상한 여유 확보.
HOME 준비자세/bridge limits는 유지.06은 자연발생ICP reject1 후 stale stop.
07에서 RGB-D를기존4.9Hz→9.8Hz로 올렸지만 위부분가림편향으로 중단.

현재 Kalman acceleration_std=.015m/s²(ICP param으로 노출), yaw process std=.15.
최소6관측+최근3속도 차이.004m/s 이하 확인. 추적영역상류Ymax.40,
파지영역[.45,-.48,.68]~[.60,.12,.82], 미래닫힘위치여유15mm.
진입 현재위치는 영역경계 허용; 미래닫힘에는 여유필수. 초기상류상자배치+.25m.
모든 영역/파지 설정은 sim용, 실제 작업공간 검증 아님.

결과/문서:
- docs/architecture/zed2i_mount.rst
- docs/architecture/conveyor_tracking.rst
- validation/head-rgbd/zed-mount-check/index.html (브라우저 열기 완료)
- validation/head-rgbd/zed-moving-07/dashboard-rgbd.html 및 scenario-rgbd.mp4

컨테이너 deux-head-rgbd, source /tmp/moving-install/setup.bash.
소스 /input 패키지별 PYTHONPATH 우선, MUJOCO_GL=egl/PYOPENGL_PLATFORM=egl.
시나리오 Python /home/xyz-ai/.cache/deux-vision-venv/bin/python,
pytest는 /usr/bin/python3 (vision venv에pytest 없음).
마지막sim ROS_DOMAIN_ID186, 회귀90. /ws/build 오래된소스캐시 사용금지.
외부 DEUX/DEUX_mujoco 수정 없음. 정지 scenario-15 성공은 과거별도결과.

진행 중 — 기존 준비자세 복원 및8cm/s 컨베이어 추종 (2026-10-26)
=============================================================

사용자 신규 요청: 준비 완료 시 원래 준비자세 복원, 이동 컨베이어0.08m/s,
tracking→ICP→Kalman 포즈/속도 사용, 급변값 거부, 오른손 작업 영역 안에서
추종/파지. 아직 구현/물리 검증 완료 아님. 이전 scenario-15는 정지 소스 검증.

ready_pose.HOME_JOINTS는 ros2-sim-subscriptions의 원래28관절로 복원.
차렷EE XY 맞춤 팔꿈치 굽힘은 ATTENTION_CLEARANCE_JOINTS 경유점으로 유지.
ObserveScene도 원래 오른팔값 유지. moving-02 준비 오차 최대0.000777rad.
검증파일 validation/conveyor-tracking/ready-audit.json.

motion_filter.py: XYZ/yaw+속도 Kalman, innovation/위치·회전 jump/속도 게이트,
거부값은 취득시각 갱신 금지. ObjectDetection에velocity/valid/observations 추가,
icp_node에서선택 ID별필터. moving02 관측속도≈0.082m/s.
conveyor.py: 파지 영역/미래 영역 이탈 거부, 관측속도로 접근/하강/닫힘 경로.
MotionPlan에 follows_conveyor/target_velocity, scheduled trajectory start 추가.
Planner/Executor에원취득시각 예측 및 최신포즈 편차 감시. moving모드는15초
frozen pose 비활성. head_task는지속 locate 요청하며현재filtered pose 사용.
현재영역은 초기 후보[.35,-.65,.55]~[.85,.05,1.05]m, IK 검증/조정 필요.

확인: 전체84개 테스트 통과(regression-moving-clean.log).
moving-02/03은 준비→추적→필터→Approach에서 IK unreachable로 종료.
03에서접근을Cartesian보간대신관절보간으로변경했으나endpoint IK도실패.
moving-04 현재실행중, target/object/velocity/horizon을오류에기록해오프라인
작업영역/도달성검토예정. docker top/log/result를확인하고중복실행금지.

컨테이너 deux-head-rgbd의 /ws/build는옛복사소스를캐시하므로 사용하지말것.
최신메시지: source /tmp/moving-install/setup.bash (build /tmp/moving-build).
Python 소스는 /input 패키지들PYTHONPATH 우선. GPU NVML오류는컨테이너restart로
복구됨. ROS 도메인 sim180, 테스트90(일부 Harness101~대역) 분리.
다음: 실제reachable영역/8cm/s경로해결,드리프트·가림 재계획 검증,최종녹화/문서갱신.
커밋/푸시 요청 없음. 외부 DEUX/DEUX_mujoco 원본변경 없음.

완료 — 헤드 RGB-D PickScan 및 영상/구조도 (2026-10-26)
===================================================

현재 결과는 docs/architecture/head_rgbd_validation.rst가 우선입니다.
scenario-15는34.340초에 가상 리더의 세션 판독까지 성공했습니다.
시나리오 MP435.394초 및 dashboard-rgbd.html 생성, 실제 브라우저 재생/
단계 이동/판독 메시지 필터 확인 완료. 두 구조도와 dashboard 브라우저 열기 완료.
전체 요구사항 증거: validation/head-rgbd/completion-audit.json.
82개 테스트,8개 overlay 빌드, 제어/ICP/세션 감사 통과.
실기 미검증. 가상 리더는 fixture EAN이며 이미지 디코딩 검증이 아닙니다.
기존 GUI preview는 legacy 경로로 유지. 커밋/푸시하지 않았습니다.
아래 실행 중/남은 작업 표현은 과거 이력입니다. 최종 실행은 모두 종료됐습니다.

최신 검증 — scenario-15 성공, 영상 렌더링 중 (2026-10-26)
======================================================

scenario-15는34.340초 Succeeded, raw/BarcodeRead 세션 대조 통과.
신선한 선택 봉투+판독 응답으로 완료하며 정렬 중 바코드 추적 검사는 유지합니다.
control-audit.json 통과. ICP2회(최종6.562초), 유일pose publisher icp_node,
설비 침투0/physics warning0/active weld0. fixture EAN이며 이미지 디코딩 아님.
82개 회귀(regression-final-15.log),8개 overlay 빌드(build-final-15.log) 통과.
BT 구조도 최종수정/9개검사/브라우저/밝음·어두움 시각검사 통과.
상세 최종 문서 docs/architecture/head_rgbd_validation.rst.

남은 작업: scenario-15 프레임347개 렌더링 종료 확인(세션49118),
자동 MP4/dashboard 합성 세션54929 확인. HTML 브라우저 재생/시각검사,
전체 요구사항 완료감사, 사용자에게전체 file:/// URL 전달 및 열기.
실패14는 종료 후576개 프레임 렌더링 중일 수 있습니다.
기존 GUI preview는 legacy 경로이며 변경하지 않았음. 커밋/푸시 요청 없음.

최신 실행 — scenario-15 (2026-10-26)
===================================

14는 실제 raw/BarcodeRead33.932초 수신했으나 리더가 카메라의 코드 영역을
가려 barcode_visible=False로 완료되지 않았습니다. 봉투는 계속 검출됐습니다.
WaitBarcode 완료 조건을 fresh bag + 세션 reader 응답으로 변경.
AlignBarcode 중 추적/신선도/이동 검사는 유지합니다.
scenario-15 ROS_DOMAIN_ID=180 녹화 실행 중, docker top/log/result 확인.
14는 종료 결과 저장 후 실패 영상 렌더링 중일 수 있음. 중복 재시작 금지.
15 결과 확인/최종 영상 생성/브라우저 검사/회귀 및 변경 BT 설명 갱신 필요.

현재 이어갈 작업 — scenario-14 통신 격리 (2026-10-26)
===================================================

scenario-13은 동시 회귀 테스트의 synthetic_optical/시스템 시각 메시지가
유입되어 Descend 관측 시각 오류로 종료. 전체 회귀82개는 통과했습니다.
scenario-14를 ROS_DOMAIN_ID=129로 격리하여 녹화 실행 중입니다.
docker top deux-head-rgbd와 validation/head-rgbd/scenario-14.log/result.json
으로 실제 상태 확인. 실행 세션46804; 프로세스 확인 없이 재시작하지 마세요.
새 가상 리더는 실제 인쇄 UV 중심/광선 가림/기존 각도 및 거리 조건 사용.
fixture EAN 발행이며 이미지 디코딩이 아닙니다. 외부 sim 원본 변경 없음.
14 결과 확인, 최종 영상/dashboard, 최신 패키지 빌드 및 전체 목표 감사 필요.

현재 이어갈 작업 — scenario-12 (2026-10-26)
=========================================

scenario-12가 deux-head-rgbd에서 실행 중입니다. docker top 및
validation/head-rgbd/scenario-12.log/result.json을 확인하고 이어가세요.
scenario-11은 AlignBarcode IK 후보 전체 탐색 때문에 기준 영상0.6s freshness를
초과했습니다. 후보를 설정 우선순위로 첫 유효 해까지만 탐색하도록 수정했습니다.
기준 신선도/실행/관절 감시는 유지합니다. 현재 ScanPose는0.75/-0.25/-0.4rad,
시야 후보 우선[-45,-45]도/roll-90도. 전체80개 테스트 후 최종 core13개 재검증.
시각적 재선택/봉투+바코드 확인은10/11에서 통과했으나 reader 판독은 미검증.
12 성공 시 최종 --record 재실행, MP4/dashboard 생성 및 최신 전체 회귀 확인 필요.
구조도2개는 완료/검증/브라우저 열기 완료이며 최신 viewer는scenario-10 실패 기록.

최신 진행 — scenario-11: 관측 바코드 시야각 후보 검증 (2026-10-26)
============================================================

두 구조도와 RGB-D 녹화 viewer 작성 완료. scenario-10은 fixture 침투0,
ScanPose→ReacquireHeldTarget→VerifyVisualPick 통과 후 AlignBarcode 정면 IK 실패.
같은 관측 plane을 향하는 허용 시야각/roll 후보를 Planner에 추가했습니다.
오프라인 계획은1.32초/모델clearance0.1988m. core13개 테스트 통과.
현재 scenario-11 실제 ROS/MuJoCo 실행 중(녹화 꺼짐).
확인: docker top deux-head-rgbd, validation/head-rgbd/scenario-11.log/result.json.
진행 세부 및 남은 작업은 docs/architecture/head_rgbd_progress.rst 마지막 절 우선.
전체 목표 미완료: reader 판독까지의 물리 검증과 최종 MP4, 최신 전체 회귀 필요.

최신 진행 — scenario-10 및 구조도 전달 (2026-10-26)
=================================================

현재 검증: deux-head-rgbd 컨테이너 scenario-10 실행/녹화 중.
반드시 docker top 및 validation/head-rgbd/scenario-10.log/result.json 확인 후 이어가세요.
scan=[0.75,0,0,-0.25,-0.4,0,0], 속도0.35, 손 끝값 여유0.02rad입니다.
scenario-07/08은 낮은 scan 자세에서 손 한계,09는 소스 상자벽 접촉으로 팔deadline 실패.
실행 servo 감시는 완화하지 않았고 검토한 적분 servo는 실행 코드에 넣지 않았습니다.
전체78개 테스트 통과. 구조도2개 및 실패한05의 RGB-D 영상/메시지 화면을 작성하고
브라우저로 열었습니다. 최신 상세/재현/제한은 head_rgbd_progress.rst 마지막 절 참조.
전체 사용자 목표는 아직 미완료(물리 재선택→바코드 정렬/판독 검증 및 최종녹화 필요).

진행 중 — 헤드 RGB-D 학습·추적·ICP PickScan (2026-10-26)
========================================================

사용자 목표는 진실값 pose 상시 공급 제거, sim 학습 YOLO/ByteTrack 기반 선택,
선택 대상 ICP 집기, 오른팔 스캔 자세 후 시각 확인, 바코드 추적/판독,
시나리오 MP4 및 노드/BT 구조도입니다. 전체 목표는 아직 미완료입니다.
최신 상세: docs/architecture/head_rgbd_progress.rst.
현재 scenario-07 실행 중(컨테이너 deux-head-rgbd, 로그 validation/head-rgbd/scenario-07.log).
scenario-06은 ScanPose 중 접힌 손가락 목표의 한계 근접으로 감시 정지했습니다.
목표만 최소값에서0.02rad 안쪽으로 수정했고 감시 기준은 유지했습니다.
scan 자세는 [0.55,0,0,-0.25,-1,0,0]rad. 재선택 단계 아직 물리 통과 미확인.
head_stack.py를 공유하여 serve_sim 기본도 head-rgbd(--weights 필수)로 바꿨습니다.
기존 실행 중 GUI preview 프로세스는 그대로 legacy입니다.
접촉 지표는 파지 봉투를 포함했던 기존값에 더해 설비/물체를 분리 기록합니다.
scenario-05 저장359프레임 감사에서는 손가락–봉투 접촉만 검출됐습니다.

최종 YOLO11n-seg 학습 완료(모델/provenance models/에 포함), 실제 ROS scenario-05는
ScanPose까지 진행하고 VerifyVisualPick timeout으로 실패했습니다.
ICP publisher만 object pose를 공급하고 최초2회 추정 후 bounded snapshot을 사용합니다.
가림 후 새 이미지 ID를 위한 ReacquireHeldTarget 및 VisualEvidenceArray 경로를
추가했습니다. 이후 전체77개 테스트 통과, 수정 후 물리 검증은 아직 남았습니다.
스캔 시 바코드 노출과 접근 경로 환경 접촉(최대2.49cm)도 해결해야 합니다.
실기 검증은 하지 않았습니다. 아직 커밋/푸시하지 않았습니다.

새 검증 컨테이너 deux-head-rgbd, 이미지 deux-ros-vision-base:local.
/input은 repo src 읽기 전용, /results는 validation/head-rgbd입니다.
실행 시 PYOPENGL_PLATFORM=egl 필수. 학습 Python은
/home/xyz-ai/.cache/deux-vision-venv (sam3의 torch2.10 CUDA13 상속).
실행법/실험 기록은 validate_head.py 및 validation/head-rgbd/ 로그를 참고하십시오.
scenario-05/scenario.mp4는 실패 시도 보존 영상이며 최종 성공 영상이 아닙니다.
scan-view-study/scan-roll-study는 recorded-qpos 기반 오프라인 시야 비교이며,
봉투-손 상대변환을 고정한 후보 평가로 물리 파지 성공 증거가 아닙니다.

기존 GUI preview는 legacy truth sim입니다. 사용자에게 새 RGB-D 경로로 바꿨다고
안내하지 마십시오. serve_sim 실동작 확인/최종 시나리오/구조도 전달이 남았습니다.

최신 변경 — sim/real 공통 원격 작업 GUI (2026-10-25)
==================================================

새 패키지 deux_task_gui: 표준 라이브러리 HTTP + ROS Action 게이트웨이.
/task/status로 실제 BT 단계 표시, /task/run으로 시작/중단/재시작합니다.
중단은 소유한 goal만 취소하며 terminal result를 기다립니다.
재시작은 종료 및 이후 inactive 상태 확인 후 새 goal을 보냅니다.
연결이 끊기거나15초 종료 확인 지연 시 재시작 예약을 해제합니다.
브라우저 종료는 작업 취소가 아니며 다른 client의 goal은 제어하지 않습니다.

기본 localhost:8080 + 토큰 인증, 원격은 SSH 터널. deployment sim/real은 표시값입니다.
구동 환경/하드웨어 활성화는 기존 ROS 스택 설정을 따릅니다.
새 serve_sim 명령은 자동 goal 없이 실제 MuJoCo/ROS 스택을 지속 실행합니다.
TaskNode의 SelectTarget/VerifyLift/ScanSearch/WaitBarcode/VerifyPlace 상태를 즉시 발행합니다.
원본 DEUX 및 외부 sim 코드 변경 없음. 세부 실행: docs/architecture/task_gui.rst.

Jazzy9패키지 빌드, 기존56+GUI2 통합 테스트58개 통과 후 추가 GUI 테스트 포함3개 재검증 통과.
실제 sim GUI 버튼: Prepare→Canceled→Prepare→Approach→Canceled 확인.
PC/모바일 화면 검사 통과. Chromium 로컬 HTTP 제한은 실제 HTTP 중계로 검사했고
사용자 Firefox에는 localhost URL을 열었습니다. 실기 및 전체 이송 성공은 미검증.
증거 validation/ros2-gui/. 커밋/푸시하지 않았습니다.
이번 GUI 외 perception/icp 관련 작업 파일 변경은 다른 작업에서 발생했으며 수정하지 않았습니다.

현재 미리보기: docker 컨테이너 deux-task-gui-preview, 내부 sim/GUI 실행 중.
호스트 /tmp/deux-gui-forward.py가127.0.0.1:8080을 컨테이너8080으로 전달합니다.
GUI는 중단된 작업 상태로 유지됩니다. 임시 환경 종료 시 컨테이너와 전달 프로세스를 종료하십시오.

최신 변경 — 준비 동작을 영상0~4초에 함께 수행 (2026-10-25)
=========================================================

차렷 자세 기준 EE XY 및 팔꿈치 약93.8도 목표는 유지합니다.
어깨·팔꿈치·손을 동시에 움직이는 중간 경유점1개로 변경했습니다.
경유점: 어깨±0.85rad, 팔꿈치±1.05rad, 손은 최종값의55%.
기존 속도/가속도 제한을 유지한 계획 길이는2.34초입니다.

ROS 2/MuJoCo 녹화 실측: 영상3.468초에 모든 관절 오차0.01rad 이내로 안정화,
영상3.638초에 Ready. 영상3.978초 표본의 EE XY 오차 왼쪽0.207mm/오른쪽0.143mm.
Prepare 계획1회, 재호출 생략 및3초 유지 성공, 관측 환경 침투0m, 물리 경고0.
첫 시도는 관절 상태가 stale로 판정되어 시작 전에 정지했습니다.
실패 기록은 validation/ros2-sim-ready-4s/attempt-stale/에 보존했고,
감시 제한을 완화하지 않은 재실행과 별도 반복 시험(repeat-check)이 모두 통과했습니다.
두 실행 모두 안정화3.468초였습니다. 최초 수신 지연의 원인은 미해결입니다.

결과: validation/ros2-sim-ready-4s/preparation-recording/dashboard.html.
validate의 --case preparation --prepare-within-seconds 4로 실측 도달 시간을 검사합니다.
영상은 기록된 물리 시각 그대로이며 배속하지 않았습니다. 실기 검증은 아닙니다.
Jazzy8개 패키지 빌드,56개 회귀 테스트, 영상4초 Ready 및 브라우저 상호작용 검사 통과.
이 절 아래의 순차8.1초 경로는 이전 결과이며 새 동시 경로로 대체되었습니다.

최신 변경 — 차렷 자세 EE XY 기준으로 수정 (2026-10-25)
=====================================================

사용자 정정: 유지할 XY는 이전 준비 자세가 아니라 후퇴 전 차렷 자세(URDF 관절0)의 EE 위치.
이 기준으로 ready_pose.py를 다시 구성했습니다. ATTENTION_JOINTS는28개0,
HOME_JOINTS는 어깨/팔꿈치만 굽힌 새 팔 목표와 기존 손 목표입니다.
로봇20cm 후퇴 및 컨베이어/상자 고정 배치는 유지합니다.

목표 월드 XY: 왼쪽[-0.650000,+0.088500], 오른쪽[-0.650002,-0.088500]m.
어깨 약31.5도, 팔꿈치 약93.8도 굽힘. 손끝 높이는 충돌 여유를 위해 약24.2cm 상승.
EE 방향은 차렷 자세와 달라집니다. 방향 유지 조건은 이번 요청에 포함되지 않았습니다.
경로: 어깨를 먼저 뒤로 올림 → 팔꿈치 굽힘 → 최종 팔 자세 → 손 준비 자세.
최초1회 Prepare 및 재호출 생략은 유지합니다.

물리 시험: EE XY 오차 왼쪽0.196mm/오른쪽0.133mm, 팔꿈치 X 후퇴 약336.9mm.
Prepare1회,3초 유지, 물리 경고0, 제어 스텝에서 관측된 로봇–환경 침투0m.
Jazzy8개 패키지 빌드,56개 회귀 테스트 및 새 화면 브라우저 검사 통과.
새 영상: validation/ros2-sim-attention-xy/preparation-recording/dashboard.html.
이전 ros2-sim-ee-preserved 영상은 잘못 이해한 '준비 자세' 기준의 과거 결과입니다.
상세 조건: docs/architecture/simulation.rst 마지막 절. 실기/전체 이송 성공 검증은 아닙니다.
이번 준비 동작 관련 변경은 아직 커밋/푸시하지 않았습니다.

최신 변경 — 최초 Prepare 이동 및 로봇20cm 후퇴 (2026-10-25)
=========================================================

사용자 요청: URDF 관절0에서 준비 자세로 최초1회 이동하고 반복 초기화하지 않음.
관절0에서 기존 컨베이어와 겹치므로, 추가 지시에 따라 컨베이어/상자는 유지하고
로봇 base_link만 world X -0.20m 이동했습니다. 외부 원본 및 실기 URDF는 변경하지 않습니다.
Prepare는 기존 PlanMotion/ExecuteMotion/팔·손 브리지를 사용합니다.
어깨를 먼저 올리는 중간 자세를 거쳐 정확한 기존 준비 관절값으로 이동합니다.
값은 src/deux_task_sim/deux_task_sim/ready_pose.py에 포함했습니다.

설정: prepare_before_first_task=true, home_joints, 선택 prepare_joint_waypoints.
성공 후 같은 task_node 프로세스에서는 Prepare 생략. 실패/취소/dry-run에는 재시도.
노드 재시작 시에는 다시 준비하며, 작업 중 기존 파지/스캔 동작은 유지합니다.
실기에서는 현재 측정값에서 시작하고 관절0으로 강제 리셋하지 않습니다.

Jazzy8개 패키지 빌드,56개 회귀 테스트 통과.
준비 물리 시험: Prepare1회, 재호출 시 추가 계획0회,3초 유지 확인.
최종 관절 오차 약0.000463rad, 제어 스텝에서 관측된 환경 침투0m, 물리 경고0.
영상: validation/ros2-sim-ready/preparation-recording/dashboard.html.
상세 조건/한계: docs/architecture/simulation.rst 마지막 절.
추가 RunTask 시험은 Prepare→접근→파지→상승 후 AlignBarcode의 라벨 이동 감시로 중단됐습니다.
후퇴한 배치에서 전체 이송 성공을 주장하지 않습니다.
이번 변경은 아직 커밋/푸시하지 않았습니다.

최신 변경 — 미사용 구독8개 제거 (2026-10-25)
============================================

RuntimeNode는 노드 역할에 필요한 구독만 선택적으로 생성합니다.
StateObservation: task/planner/execution. BarcodeTarget: task/execution.
/state/status: execution/arm/hand. task의 중복 라벨 구독도 제거했습니다.
직접 관절 피드백, planner의 최신 관측 비교 및 실행 감시는 유지합니다.
정적 구조도와 sim trace의 수신 대상 목록도 갱신했습니다.

Jazzy8개 패키지 빌드, 실제 구독 개수 검사 포함53개 테스트 통과.
sim에서 변경 전과 동일한 PlanMotion 요청 순서 및 바코드 수신을 확인했습니다.
Place는 기존과 동일한 IK unreachable로 종료했습니다. 전체 태스크 성공 검증이 아닙니다.
새 실행 오류/물리 경고 없음. 짧은 BT 상태는 발행/샘플링 시점에 따라 기록 여부가 달라
모션 순서는 Action goal 기록으로 비교했습니다.
증거: validation/ros2-sim-subscriptions/의 build.log, regression.log,
flow-comparison.json 및 scenario-recording/. 이전 녹화는 보존했습니다.
이번 커밋에서도 tools/validation은 제외하고 푸시는 수행하지 않습니다.

최신 변경 — ROS 2 sim 연결 및 동기화 녹화 (2026-10-25)
======================================================

실기 적용 준비 문서: ``docs/architecture/hardware_rollout.rst``.
현재 커밋 범위/제외 항목, 추가 구현·설정, 단계별 검증 및 통과 조건을 정리했습니다.

사용자 범위: 태스크 성공 튜닝 없이 실행 흐름 녹화와 시간별 노드/메시지 시각화.
deux_task_sim 추가: 기존 MuJoCo를 ROS 팔 Action/손 토픽 드라이버와 /clock에 연결.
공통 runtime/bridge를 그대로 실행하고, sim 관측은 가시성 기반 truth6D/라벨 센서입니다.
실제 YOLO/ICP 실행 및 하드웨어 검증은 포함하지 않습니다.

최종 기록은 Approach → Descend → Close → Lift → ScanSearch → AlignBarcode →
WaitBarcode → Place 요청까지 진행 후 Place IK unreachable로 Failed 종료했습니다.
가상 바코드 문자열은 수신했습니다. 전체 시나리오 정상 완료를 확인한 것은 아니며,
Release/Retreat/VerifyPlace는 실행되지 않았습니다. 성공 튜닝은 요청에 따라 중단했습니다.

* 검증: Jazzy8개 패키지 빌드,52개 회귀 테스트 통과.
* 영상24.276초,238프레임 / 이벤트5,946건,14개 토픽 및6개 Action 계측.
* 최종 화면: validation/ros2-sim/scenario-recording/dashboard.html.
* 브라우저 재생/시간 이동/노드 필터/메시지 상세 검사 통과, JS 오류 없음.
* 실기 전환 항목: 녹화 화면 하단 표 및 simulation.rst 마지막 절에 구동/센서/리더/TF/설정 교체와 유지 노드를 정리했습니다.
* 재현: bash tools/validate_sim.sh. 상세 조건/공통 코드 변경: docs/architecture/simulation.rst.
* 원본 DEUX/DEUX_mujoco는 수정하지 않았습니다. 사용자 요청으로 소스·설정·문서를 커밋합니다.
  tools/validation 및 구조도 검토용 산출물은 제외하며, 푸시는 수행하지 않습니다.

아래 과거 인계보다 이 절과 simulation.rst가 우선합니다.

최신 변경 — 단계별 인식과 Behavior Tree (2026-10-24)
====================================================

공통 YOLO에서 봉투/바코드를 검출하고 ICP·라벨 표면 worker를 분리했습니다.
PerceptionRequest lease/request_id로 모드 전환과 오래된 결과 폐기를 처리합니다.
ScanSearch → AlignBarcode → WaitBarcode BT, 라벨 기반 왼팔 정렬/실행 감시,
이미지 없는 StateObservation 제어 경로를 연결했습니다.

권장 경로8개 노드이며 기존 standalone 경로는 별도 옵션입니다.
상세 실행 조건/제약: docs/architecture/controlled_perception.rst.
새 구조도: docs/architecture/nodes.html, docs/architecture/scenario.html.
정적 설치 YAML 직접 로드는 유지하며 scene_node는 없습니다.

검증: Jazzy7개 패키지 빌드,49개 합성 테스트 통과. 하드웨어 미실행.
barcode_classes는 데이터셋 기준 설정 필요. yaw ICP는 full_orientation=false이며,
Place에 필요한 파지 후6D 인식은 여전히 현장 통합 과제입니다.
변경사항은 커밋/푸시하지 않았습니다. 아래 과거 인계보다 이 절이 우선합니다.

최신 변경 — scene_node를 설정 파일 직접 로드로 대체 (2026-10-24)
================================================================

사용자 요청에 따라 scene_node, Scene.msg, /task/scene, /task/reload_scene를 제거했습니다.
현재 전체9개 노드입니다. 아래 이전 인계의 scene_node/10개 노드 설명보다 이 절이 우선합니다.

* task/planner/execution/arm/hand는 RuntimeNode에서 config_file YAML을 시작 시 직접 로드.
* 공통 로더: src/deux_task_core/deux_task_core/configuration.py.
* launch의 scene_config 인자를5개 소비 노드에 config_file로 전달.
* YAML과 URDF 검증 실패 시 시작 거부. URDF 상대 경로는 YAML 디렉터리 기준.
* 설정은 시작 시 snapshot. 파일 변경의 자동 반영/폴링/재로드 서비스 없음.
  YAML 수정 후 작업 종료 및 관련 노드 전체 재시작. config_file 파라미터는 read-only.
* Action의 scene_version 해시는 유지. BridgeMotion에도 추가하여 bridge 설정 불일치 거부.
* planner/실행/팔 URDF 재사용 시 시작 버전과 달라지면 재시작 요구.
* 실제 장비 명령은 전송하지 않았으며 변경사항은 아직 커밋하지 않았습니다.

최신 검증: Jazzy7개 패키지 빌드, 41개 테스트 통과. 실제 하드웨어 미실행.
최신 실행 안내: docs/architecture/runtime.rst. 검증 결과: validation/jazzy-container.log.

최신 진행 — 설계의 나머지7개 노드 구현 (2026-10-24)
===================================================

사용자가 design.html 기반 나머지 노드 전체 구현을 요청했습니다.
아래 과거 인계의 runtime 미구현 설명보다 이 절과 docs/architecture/runtime.rst가 우선합니다.
기존 DEUX 원본 및 하드웨어는 수정/실행하지 않았습니다. 이번 변경은 아직 커밋하지 않았습니다.

추가: barcode_bridge_node, scene_node, task_node, motion_planner_node,
execution_node, arm_bridge_node, hand_bridge_node. 전체10개 노드, overlay6개 패키지입니다.
별도 프로세스 계획, URDF FK/IK, Cartesian 접근/하강/상승, 라벨 기준 스캔 후보,
Hermite 관절 경로, 명시적 collision envelope 및 관절/속도/가속도 검사,
팔/손 공통 시간 실행·취소·피드백 감시, 파지2회·스캔3회 제한 및 독립 관측 검증을 추가했습니다.
observation_node의 선택적 XY/yaw Kalman과 엄격한 legacy JSON adapter도 추가했습니다.

핵심 파일:

* src/deux_task_core/deux_task_core/{model,planning,filter}.py
* src/deux_task_runtime/deux_task_runtime/{scene_node,task_node,motion_planner_node,execution_node}.py
* src/deux_task_adapters/deux_task_adapters/{arm_bridge_node,hand_bridge_node}.py
* src/deux_task_bringup/launch/task.launch.py 및 config/*.yaml
* src/deux_task_msgs/action/*.action 및 Scene/MotionPlan/BarcodeRead/ScanSession/TaskStatus.msg
* docs/architecture/runtime.rst: 설치 조건·구현과 설계의 차이·검증 범위

설치 템플릿에는 미확정 frame/손 매핑/기하를 임의로 채우지 않았습니다.
hardware_enabled=false, dry_run=true가 기본입니다. String 바코드 stamp는 수신 시각입니다.
Scan/Place는 full_orientation=true가 필요하므로 현재 yaw-only ICP만으로는 실행 불가합니다.
실기 expanded URDF, collision envelope, grasp/reader/label 보정, 손 피드백/프로토콜,
파지 후6D 인식 생산자가 남은 현장 통합 입력입니다.
이전 MuJoCo controller와 궤적/접촉 동등성, 실제 성공률은 검증하지 않았습니다.

검증 도구는 overlay 전체와 기존 DEUX의 deux_hand_msgs를 읽기 전용으로 복사해 빌드합니다.
합성 태스크 결과는 시험 코드의 물체/리더 관측이며 물리 시뮬레이션 결과가 아닙니다.
최종 검증: ROS 2 Jazzy/Python3.12.3, 7개 패키지 빌드, 전체39개 테스트 통과.
상세 로그는 validation/jazzy-container.log 및 README.rst를 참고하십시오.
이전 사용자 지시에 따라 tools/와 validation/은 Git 미추적 상태로 남겨 두었습니다.

최신 진행 — detection/ICP 구현 완료 (2026-10-24)
================================================

사용자가 이번 세션에서 ``detection/ICP 신규 노드 구현까지 진행``을 명시했습니다.
아래 이전 인계의 "아직 요청되지 않음/미구현" 설명보다 이 절이 우선합니다.
장기 실기 실행 목표를 재개하거나 로봇 명령을 보내지는 않았습니다.

추가 구현:

* detection_node: YOLO 1회 로드, RGB→BGR 입력, s1/s1_1 bbox 검출.
* DetectionFrame: 촬영 header/sequence + 검출 bbox + 같은 bundle의 depth/CameraInfo.
* icp_node: 보정 reference TF, depth 역투영, 원본 yaw-only ICP, 품질 필터,
  명시적 model 원점/SHA256, base 변환, 거리 기반 track ID.
* 실행 중1개 + 최신 대기1개 worker. stale/future/duplicate 입력과 오래된 결과 거부.
  ROS clock 역행 epoch 이전 결과 폐기. 보정 TF 없으면 pose 발행 안 함.
* ObjectDetection에 fit_metrics_valid/trimmed_rmse_m/target_coverage_10mm 및
  covariance_estimated 추가. 관련 외부 ROS 패키지는 재빌드 필요.
* perception.launch.py와 config/perception.yaml 추가. weights/model_path/
  model_origin/reference_frame은 필수 설치 설정이며 기본값을 임의로 채우지 않음.

현재 구조/실행 지침: docs/architecture/perception.rst.
기존 HTML/JSON 구조도는 이전 8노드 제안 스냅샷으로 보존했습니다.
최신 문서의 구조가 우선하며, 전체 계획은 detection/ICP 포함10노드 중3노드 구현입니다.

최종 검증:

* bash tools/validate_jazzy.sh: Jazzy/Python3.12.3, 2개 패키지 빌드, 24 passed.
  기존17개 + 신규7개(코어5개, ROS2개). 합성 검출기→실제 ICP ROS 통신,
  TF 누락 거부, 원본 stamp/sequence 유지, ID 연결, depth 시각 불일치,
  점 부족, 입력 중복/지연, 완료 결과 만료/epoch 거부 검증 포함.
* validation/icp-replay.json: 저장3프레임5물체, 원본 ICP 변환 행렬 차이0.
  현재 품질 필터는 이중1개를 수락하며4개를 제외. 원본 reference 좌표 비교임.
* validation/yolo-replay.json: 실제 기존 weight로 저장 이미지1장 CPU 추론 확인.
  기존 conda 환경에서 실행. Jazzy Python에서 YOLO 통합 실행/GPU 검증은 아님.
* 실제 ZED/DEUX 입력, 실기 base 보정, 파지/스캔/적재는 미검증.

다음 작업:

1. 실제 Jazzy Python 환경의 YOLO 의존성을 준비하고 저장 RGB-D로 통합 실행.
2. 설치별 reference/base TF와 모델 원점 정책 확정 후 실시간 ZED 수신 검증.
3. 가림/교차 ID 유지, 품질/지연 분포, 파지 후 tilt 관측 검토.
4. Kalman 필터, barcode, scene, BT, planner, execution 및 하드웨어 bridge는 미구현.

기존 DEUX, 원본 detection/ICP 코드 및 mesh는 수정하지 않았습니다.
docs/reference_manifest.json은 이전 인계 시점의 경로/해시 기록입니다.

이전 세션 인계 — DEUX sim → real / ROS 2
====================================

작성: 2026-10-24. 이 문서는 대화의 기술적 인계 요약이며, 실제 파일 상태가 우선입니다.
새 세션 작업 디렉터리: /home/xyz-ai/git/DEUX_ikworld

바로 이어서 읽을 것
-------------------

1. 이 문서 → ../README.rst → docs/architecture/design.html.
2. src/deux_task_perception/deux_task_perception/observation_node.py 및 core.py.
3. src/deux_task_msgs/msg/*.msg, config/observation.yaml, test/.
4. detection/ICP 원본(아래 경로)을 실제로 읽고 입력/출력과 의존성을 확인.

최신 요청과 작업 범위
--------------------

* 사용자는 기존 DEUX 실기 코드에 손대지 않고 새 ROS 2 노드를 추가해
  sim의 빵봉투 파지→바코드 스캔→적재를 수행하려고 합니다.
* 신규 코드 위치는 반드시 /home/xyz-ai/git/DEUX_ikworld.
  기존 /home/xyz-ai/git/DEUX 및 zed-ros2-wrapper는 참고/연동 대상입니다.
* 이번 턴은 자료 정리 요청입니다. detection/ICP 노드를 구현하라는 추가 지시는
  아직 없었으며, 다음 작업 후보가 detection/ICP 연동이라는 점을 설명했습니다.
* 이전 장기 sim→real 목표는 사용자가 일시정지했습니다. 새 세션에서 이를 근거로
  임의의 장기 실험/실기 실행을 재개하지 말고 최신 사용자 요청을 따릅니다.
* 실기 로봇 명령 전송은 아직 구현하지 않았습니다. 현재 노드는 관측만 발행합니다.

사용자가 확정한 조건
-------------------

* 물체 pose는 로봇 베이스 좌표계로 맞춰서 제공한다고 가정.
* 카메라는 기존 URDF/head camera가 아니라 더 위에 추가된 별도 ZED 카메라.
  추후 URDF에 장착 위치를 넣을 예정. 현재 extrinsic을 지어내지 않습니다.
* ZED 입력은 등록된 RGB+depth, 640x480을 가정. 각 CameraInfo 필요.
* 인식 비교는 FoundationPose의 학습 기반 방법보다 기존 ICP 방식 기준.
* 입력 봉투 pose만 있고 출발 박스 높이는 7cm로 일정.
  봉투 pose만으로 박스 벽 XY·적재함 위치를 모두 알아낼 수는 없습니다.
* 실제 바코드 판독 문자열 수신 가능. 필요하면 라벨 detection/depth 사용 가능.
* 실기 I/O 공통 인터페이스까지 먼저 구현하는 방향을 승인한 상태.
* 이전 sim에서 등속도 Kalman XY/yaw 필터 적용 승인.
  현재 신규 observation_node에는 Kalman 필터가 아직 없음. 이중 필터링 주의.
* 로컬 HTML 제공 시 완전한 file:/// URL을 보여주고 GUI 가능하면 열어 줍니다.

완료된 코드
-----------

패키지:

* deux_task_msgs: ObjectDetection, ObjectDetectionArray, Observation.
* deux_task_perception: observation_node, core, launch, config, 테스트.

observation_node 기능:

* RGB·depth·각 CameraInfo 4개 동기화(기본 slop20ms, queue10).
* rgb8/bgr8/rgba8/bgra8 → rgb8.
* 32FC1(m), ZED OpenNI mono16/16UC1(mm) → 32FC1(m).
* depth 0/음수/NaN/Inf는 NaN 유지, 유효 depth 비율 발행.
* 해상도640x480, optical frame, 투영행렬, 시각, row padding/endian 검증.
* 임의 resize/crop, 가짜 extrinsic, head camera 대체 없음.
* 선택 TF: base_from_camera 및 설정된 left/right grasp frame의 EE TF.
  각 valid flag가 false이면 해당 transform은 사용 불가.
* /arm/joint_states와 신규 /task/hand_state에서 실제 측정28관절 조립.
  관절 이름 기준 정렬. 누락/오래된 값은 joints_valid=false, 0으로 대체 안 함.
* /task/object_detections의 외부 pose를 검증 후 관측에 포함.
  원본 촬영 stamp/sequence 유지. objects_valid는 파지 성공/품질 보장이 아님.
* /task/observation/diagnostics에 수신·거부·대기 상태 발행.

미구현:

* 물체 detection, depth ROI/포인트 추출, ICP 추정 자체.
* 안정적인 track ID 할당, ICP 품질 기반 수락/공분산 추정, Kalman 필터.
* barcode_bridge, scene, task BT, planner, execution, arm/hand bridge.
* 기존 sim JSON ObservationFrame으로 변환하는 ROS adapter.
  새 Observation 메시지와 sim JSON은 직접 호환되지 않습니다.

검증 증거
---------

validation/jazzy-container.log: ROS_DISTRO=jazzy, Python3.12.3.
colcon build: 두 패키지 성공. pytest: 17 passed.
이 중16개는 변환/검증 단위 테스트, 1개는 ROS 합성 RGB-D pub/sub 통신 테스트.
실제 ZED/DEUX 하드웨어 입력·실기 파지·성공률을 검증한 것은 아닙니다.

호스트 /opt/ros는 없음. Docker는 사용 가능.
재현: bash tools/validate_jazzy.sh
컨테이너는 소스를 read-only로 읽어 임시 workspace에서 빌드하고 제거됩니다.
호스트 ROS 설치·장치 연결·host network 접근 없이 검사했습니다.
README.rst에 image digest와 실행 명령이 있습니다.

ZED 토픽 근거
-------------

/home/xyz-ai/git/zed-ros2-wrapper/zed_components/src/zed_camera/src/zed_camera_component_video_depth.cpp
initVideoDepthPublishers를 확인했습니다. 이 버전의 RGB는 구형 토픽명이 아닙니다.
기본 camera_name=zed, node_name=zed_node, namespace 미지정:

* /zed/zed_node/rgb/color/rect/image
* /zed/zed_node/rgb/color/rect/camera_info
* /zed/zed_node/depth/depth_registered
* /zed/zed_node/depth/camera_info

실제 namespace가 지정되면 node_name 변경 분기가 있으므로 현장 토픽을 확인합니다.
ZED pub_downscale_factor만으로 모든 센서가640x480이 되는 것은 아닙니다.
640x480 등록 입력을 받는 계약이며, 리사이즈가 필요하면 영상과 K/P를 같이 조정해야 합니다.

Detection / ICP 참고 코드 — 다음 작업의 출발점
----------------------------------------------

사용자가 지정했던 비교 화면:
file:///home/xyz-ai/Downloads/foundationpose2/outputs/zed_ros2_wrapper_2d/index.html#compare

디렉터리 /home/xyz-ai/Downloads/foundationpose2/outputs/zed_ros2_wrapper_2d/:

* run_detection.sh: /home/xyz-ai/Downloads/run_deux_detection.sh 호출.
* process.py: detections → bbox depth ROI → 역투영 → reference 변환 → ICP.
* fast_fit.py: 24개 yaw 초기각, coarse5회, 상위2후보 refine최대20회,
  조기 종료. XYZ이동+yaw만 추정하며 roll/pitch는 고정.
* calibrate.py, fixed_references.json: 기존 데이터의 기준 평면 좌표계.
* summary.json, pose_results.csv, metrics.json: 이전 배치 평가 결과.

추가 의존 코드:

* /home/xyz-ai/Downloads/foundationpose2/outputs/detection_yaw_icp/fit.py
  model_samples, downsample, rot, solve_yaw.
* /home/xyz-ai/Downloads/foundationpose2/outputs/zed_deux_detection/detect_rgb.py
  Ultralytics YOLO, imgsz640, conf0.25, icp_target 클래스 s1/s1_1.
  기존 모델 경로 /home/xyz-ai/Downloads/deux_1021_A_v2.0only_adamw.pt.
  이 detect_rgb.py는 참고 구현이며 위 run_detection.sh가 실제 호출하는
  외부 스크립트의 현재 내용/모델 선택을 다시 확인해야 합니다.

중요한 차이:

* process.py의 calibrated_robot_base=False. T_reference_camera는 실제 로봇
  베이스 변환으로 검증된 값이 아닙니다. base_link로 이름만 바꾸면 안 됩니다.
* 원본 코드의 >.01m 높이, <=1.5m 거리, 점40개 기준 등은 당시 reference
  평면에 종속. 실제 base 좌표에서 base.z>.01을 그대로 적용하면 안 됩니다.
* quality_warning은 trimmed_rmse>.015m 또는 coverage10mm<.5.
  이는 ground-truth 실패율이 아니라 fitting 품질 지표입니다.
* yaw-only ICP로 들어 올리고 기울어진 봉투를 완전한6D 관측으로 표시하면 안 됩니다.
  ObjectDetection.full_orientation=false로 표현하고 파지 후 추적 방안을 분리합니다.
* 모델 원점이 중요합니다. sim의 중심 원점과 원본 mesh 원점이 다를 수 있습니다.
  centered pose 변환을 확인한 후 model_id로 명시해야 합니다.
* 오래된 배치 스크립트를 통째로 ROS callback에 넣지 않습니다.
  모델1회 로드, 최신 프레임 제한 큐, 별도 계산 worker, 촬영 시각 보존이 필요합니다.

다음 구현 순서 제안 (아직 구현 안 함)
-------------------------------------

1. 실제 detection 실행 스크립트/weight·class 매핑/의존 환경을 확인.
2. detection_node: 동기화 RGB-D의 RGB → bbox/score/class/frame sequence.
3. icp_node: 같은 RGB-D frame의 bbox+depth+CameraInfo → 포인트 → ICP.
   frame_id/촬영stamp/sequence로 결합, 늦은 검출에 다른 프레임 depth를 쓰지 않기.
4. 명시된 reference→base 변환과 모델 원점 변환 후 ObjectDetectionArray 발행.
5. track_id 연결, 중복 관측 억제, 품질/지연/실패 결과 및 full_orientation 구현.
6. 저장 RGB-D 재생 비교 → Jazzy 합성/회귀 테스트 → 실제 ZED 수신 검증.

현재 observation_node는 /task/observation을 발행하고 /task/object_detections를
다시 받아 묶습니다. detection/ICP는 이미지를 소비할 수 있지만 동일 pose를 다시
처리하는 피드백 루프가 생기지 않도록 RGB sequence로 딱1회만 처리해야 합니다.
필요하면 다음 구현에서 원시 RGB-D bundle과 제어 관측 토픽을 분리하십시오.
현재 8노드 구조도에는 별도 detection/ICP 생산자가 외부 입력으로 묶여 있습니다.
이를 신규 노드로 만들면 전체 수는 늘어나므로 구조도도 갱신해야 합니다.

기존 실기 레포 인터페이스
-------------------------

/home/xyz-ai/git/DEUX (기존 소스 수정 금지):

* 양팔14관절 Action /arm/deux_arm_controller/follow_joint_trajectory.
* 측정 /arm/joint_states. arm_l_joint1..7, arm_r_joint1..7.
* 손 /hand/left/cmd, /hand/right/cmd: deux_hand_msgs/HandCtlMessage.
* 손 드라이버는 원격PC. 실제 feedback 토픽·ID/부호/전류단위 아직 미확정.
* /state/status는 JSON, /state/set_mode는 실제 팔/리프트 이동 유발.
* teleop/state_node도 직접 명령을 발행하므로 자동 태스크와 동시 제어 금지.
  기존 소스 수정 없이 launch 분리/remapping 등으로 실행 경계를 구분할 설계.

sim 측 최신 참고
----------------

/home/xyz-ai/git/DEUX_mujoco/sim/:
transfer_controller.py, transfer_interface.py, transfer_observation.py,
transfer_motion.py, transfer_contact_planner.py, transfer_scan.py, transfer_geometry.py.

* 입력28관절 순서: 왼팔7, 왼손7, 오른팔7, 오른손7. 단위 m/rad.
* 물체는 v6: assets/breadbag_model_v6. 양면 좌측 스티커, 하단여백20mm.
* 오른팔을 앞으로 뻗는 스캔 자세 하나로 통일. 양면 라벨 후보 선택.
* 스캔 후 왼손 리더 후퇴+오른팔 적재 동시 이동.
  스캔 전 자세 역재생 제거. 직접 경로가 불가하면 안전 경유1개를 Hermite 연속 통과.
* 최근 seed1 성공: grasp/scan/place 모두 성공, 물리경고0, IK4475.
  outputs/reader-pick/sim-to-real/direct-place-final-seed1/recording.json, validation.json.
  영상 excerpt/scenario.mp4. 이1회 성공을 모든 위치/각도 성공으로 확장하지 않습니다.
* 기존 관측 제어기에 스캔최대3회 있음. 예전 다른 BT의 파지최대2회는 아직 미이식.
* ControlIO.step 계획은 동기식, 명령 기본 유효기간0.1s.
  real 실행기로 바로 넘기기 전에 계획 worker/시간 기반 실행을 분리해야 합니다.
* 목표는 궤적을 유지하되 설치 높이/위치에 맞춰 관측 기반 재계획.
  미관용 자세 선호를 완화해도 관절한계/충돌/실제 판독 조건은 유지.

이 디렉터리에 복사한 설계 자료
-------------------------------

* docs/architecture/architecture.html: archify 구조도, 고정 Viewer UI는 영어.
* docs/architecture/design.html: 노드별 책임·인터페이스·패키지·검증 단계.
* docs/architecture/architecture.json: 구조도 원본.
* docs/architecture/delivery.json, handoff.json: 당시 artifact 검증 기록.
  기록의 output 경로는 생성 당시 DEUX_mujoco 위치이며 파일 내용은 동일한 복사본.
* docs/reference_manifest.json: 중요 파일의 현재 절대경로와SHA256.

다음 세션 첫 메시지 예시
------------------------

  /home/xyz-ai/git/DEUX_ikworld 에서 이어서 작업해줘.
  AGENTS.md와 docs/HANDOFF.rst, README.rst를 먼저 읽어줘.
  observation_node는 Jazzy 빌드와17개 테스트 통과 상태야.
  detection/ICP는 아직 미구현이고 FoundationPose2의 기존 ICP 파이프라인을
  참고해야 해. 기존 DEUX 코드는 수정하지 말고 새 노드는 여기 작성해줘.
