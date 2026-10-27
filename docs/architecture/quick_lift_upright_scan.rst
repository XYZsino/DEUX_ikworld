빠른 Lift와 손등 위쪽 ScanPose (2026-10-26)
==========================================

손 닫힘 완료 후 별도 파지 성공/접촉/영상 판정 없이 Lift를 실행한다.
기존 head BT에도 Lift 전 파지 성공 판정은 없었다. 이번 변경은 Lift 궤적과
명령 예약 여유가 만드는 지연을 줄인다. 제어기 완료/추종 오차/충돌 검사는 유지한다.
ReacquireHeldTarget은 Lift와 ScanPose 이후의 관측 대상 연결 단계다.

설정과 구현
-----------

head_calibration.yaml:

* lift_height=0.13 유지.
* lift_cartesian_step=0.04: Lift IK 경유점 간격을1cm에서4cm로 변경.
* lift_minimum_segment_duration=0.06: Lift 경유구간 최소시간0.3→0.06초.
* lift_start_lead=0.15: 팔/손 동시 시작 예약 여유0.5→0.15초.

Planner.hermite는 기존 관절 속도/가속도 제한을 적용해 시간을 다시 계산한다.
Lift에만 새 시간/간격 설정을 적용하고20ms 간격 샘플과 충돌 검사를 유지한다.
새 설정이 없는 설치는 기존 동작을 유지한다. URDF/관절 제한은 변경하지 않았다.

ScanPose는 기존 고정 자세의 EE XYZ를 기준으로 XY는 유지하고 Z를10cm 높였다.
right_hand_base_link의 -Y 축(손등)을 base_link의 +Z 방향에 정렬했다.
기존 방향에서 최소 회전을 적용한 목표로 IK를 풀고, 후보 중 관절 꺾임이 작은
첫 번째 해를 scan_right_joints에 저장했다. 오른손 닫힘 목표를 유지한다.
새 EE 목표는 base_link 기준 [0.674890,-0.096768,0.918668]m,
이전 대비 [+0.000203,-0.000019,+0.099917]m, 손등 수직 오차0.076도다.

검증
----

ROS2 Jazzy8패키지 빌드,119개 회귀 테스트 통과. 설정 변경 후 실제 URDF를 사용한
추가 ScanPose 경로 검사에서 위치/손등 방향/관절 제한/닫힌 손 유지/충돌 검사 통과.

validation/head-rgbd/quick-lift-restored/probe.json:
동일한 저장 Lift 시작 관절값에서 계획시간4.68→2.18초(약53% 단축).
상승13cm와 손목 방향 유지, 새 경로 속도/가속도 최대 사용률약8%/83%.
scan-check.json: Lift→새 ScanPose 계획4.14초, 손등 수직 오차0.076도.

run-02 실제 ROS/MuJoCo 단계:
Close10.676→Lift12.376→ScanPose15.334→ReacquireHeldTarget20.230초.
Lift 단계2.958초. 직전 pregrasp-direct/run-05의5.882초와 비교하면약50% 감소지만,
접근 인식 시점/시작 관절값이 다른 실행이므로 같은 입력의 비교는 probe.json을 사용한다.
actual-audit.json: 도착 측정 EE의 높이 변화+9.9815cm, XY 변화0.32mm,
손등 위쪽 방향 오차0.1117도. 파지 성공 판정 없이 Lift로 넘어간 단계 기록 확인.

이후 ReacquireHeldTarget의 fresh visual evidence timeout으로28.254초에 종료했다.
전체 파지/스캔 성공, 반복 성공률, 실기 검증은 아니다. 물리 경고0,
설비 침투0, 물체 접촉 최대 침투7.25mm, weld0. 관통/파지 안정성 문제는 남아 있다.
run-01은 Lift 이전 TrackGrasp에서 conveyor forecast expired로 중단됐으며,
이번 범위에서 기존 관측 시각 검사 문제를 수정하거나 감시를 완화하지 않았다.

기록
----

validation/head-rgbd/quick-lift-restored/ 아래 build.log, tests.log,
probe.py/probe.json, check_scan.py/scan-check.json, audit.py/actual-audit.json.
영상과 메시지: run-02/dashboard-rgbd.html, run-02/scenario-rgbd.mp4.
외부 DEUX/DEUX_mujoco 수정 없음. 기존 GUI preview 재시작 및 커밋/푸시 없음.
