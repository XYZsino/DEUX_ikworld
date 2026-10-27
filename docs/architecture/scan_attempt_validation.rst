왼손 리더 정렬까지 진행한 스캔 시나리오 (2026-10-26)
===================================================

요청: 현재 빠른 Lift/손등 위쪽 ScanPose를 사용해 실제 왼팔 스캔 동작까지 녹화.
모든 동작/검증은 ROS2 Jazzy와 MuJoCo SIM이며 실제 로봇 검증이 아니다.

이전 중단 원인과 수정
--------------------

quick-lift-restored/run-02 저장 물리 상태를 조사하면 bread_1은 Lift 종료까지
엄지/검지 접촉으로 유지되지만 ScanPose 회전 중 이탈한다. 따라서
ReacquireHeldTarget이 새 봉투를 선정하지 못하고 시간 초과되는 것이 타당했다.
인식 조건이나 성공 조건을 완화하지 않았다.

동일 Lift 종료 qpos와 인접 기록에서 복원한 qvel에서 저장 팔 경로를 재생하고
ScanPose 이후 약3.8초 유지했다. 이는 온라인 전체 시나리오의 완전한 재현이
아닌 SIM 파라미터 후보 비교다. hold-trial.json:

* 기존 유지강도1배/마찰1배: 최종 손-물체 원점 거리40.98cm, 높이13.68cm 하락.
* 유지강도2배/마찰1배:44.04cm,13.68cm 하락.
* 유지강도4배/마찰1배:6.09cm, 높이0.80cm 상승.
* 유지강도4배/마찰1.5배:53.23cm,13.68cm 하락.

오른손 SIM 위치서보만 Kp6→24, Kd0.6→1.2로 변경했다.
토크상한5N·m, 손가락 미끄럼 마찰3, 엄지·검지 목표와 third 고정값은 유지했다.
손가락 목표/빠른 Lift/높인 ScanPose/실기 hand_bridge 전류 명령은 변경하지 않았다.
해당 수치는 SIM 가정이며 실측 모터/마찰 보정값이 아니다.

구현은 sim 전용 grip_physics.py와 head_calibration.yaml의 sim_grip_physics.
MujocoNode 시작 때 적용하고 실행 폴더의 grip-physics.json에 기록한다.
같은 설정을 재적용해도 배율이 누적되지 않는다. 왼손/벨트/봉투 재료는 보존한다.
실제 로봇이나 외부 DEUX/DEUX_mujoco 코드는 변경하지 않았다.
물체 위치 재설정/강제 부착/weld는 사용하지 않았다.

최종 전체 ROS 시나리오
---------------------

validation/head-rgbd/scan-attempt/run-01:

* Approach6.732 → Descend9.350 → Close10.540 → Lift12.240초.
* ScanPose15.232 → ReacquireHeldTarget20.196초.
* 영상 기반 대상 연결 visual-12→visual-20 후 VerifyVisualPick20.400초.
* AlignBarcode20.706초: 왼팔 리더 정렬 시작.
* ExecuteMotion 완료23.766초, WaitBarcode23.868초: 정렬된 스캔 자세.
* 24.174초 가상 리더2087231250396 응답, BT Succeeded.

ZED2i 렌더링 RGB-D→YOLO/ByteTrack→선택 ICP/Kalman→ROS 계획/실행/bridge를
사용했다. 새 ID 연결은 손 근처 영상 후보 연결이며 영속 객체 ID 증명이 아니다.
가상 리더는 인쇄 UV/광선 가림/거리·각도 조건을 확인해 fixture EAN을 반환한다.
실제 이미지 디코더의 바코드 판독이나 실제 로봇 성공으로 해석하면 안 된다.

물리 경고0, 설비 침투0, 물체 접촉 최대 침투22.79mm, 활성 weld0.
봉투/손가락 접촉 관통 문제는 남아 있다. 단일 SIM 성공이며 반복 성공률은 미검증.
요청 범위는 스캔까지이며 배치 시나리오는 포함하지 않는다.

ROS8패키지 빌드,123테스트 통과(build.log/tests.log).
영상/메시지: run-01/dashboard-rgbd.html, run-01/scenario-rgbd.mp4.
정렬 완료 장면: run-01/13-WaitBarcode.png.
원본 비교 스크립트/결과: contact_audit.py, contact-audit/result.json,
hold_trial.py, hold-trial.json. commit/push 요청 없음.
