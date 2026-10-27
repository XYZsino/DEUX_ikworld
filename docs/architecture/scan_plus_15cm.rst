ScanPose Z축15cm 추가 상승 (2026-10-26)
======================================

직전 손등 위쪽 ScanPose의 EE를 base_link Z축으로15cm 추가 상승시켰다.
최초 자세 대비 누적 상승은약25cm다. XY와 EE 방향, 닫힌 손 목표는 유지한다.
head_calibration.yaml의 scan_right_joints만 갱신했으며 Lift/그립/실기 제어는 변경하지 않았다.

기존 목표 [0.674890,-0.096768,0.918668]m → 새 목표
[0.674984,-0.096723,1.068591]m. IK 위치 오차0.13mm,
목표 변화 [+0.000095,+0.000046,+0.149923]m, 손등 위쪽 오차0.045도.
이전 성공 실행의 Lift 종료 관절값에서 새 ScanPose 경로를 검사했다.
관절 속도/가속도/충돌/손가락 유지 검사 통과, 계획시간3.9초.

SIM run-01은 Lift12.376→ScanPose15.334→재인식20.332→AlignBarcode20.808→
WaitBarcode23.834→Succeeded24.140초로 진행했다. 가상 리더2087231250396 응답.
측정된 ScanPose의 직전 목표 대비 변화는Z+14.972cm, XY0.177mm,
손등 위쪽 오차0.080도다. 실제 하드웨어/실영상 디코딩 검증이 아니다.
물리 경고0, 설비 침투0, 물체 접촉 최대 침투23.40mm. 단일 SIM 결과다.

검증 위치: validation/head-rgbd/scan-plus-15cm/
설정 이전값: before.yaml. 계획 검사: solve.py/check.json.
측정 감사: audit.py/actual-check.json. SIM 패키지 빌드: build.log.
영상/메시지: run-01/dashboard-rgbd.html, run-01/scenario-rgbd.mp4.
설정값만 변경했으므로 회귀 테스트를 새로 추가하지 않았으며 경로 검사와
영상 기반 실제 ROS/MuJoCo 전체 스캔 실행으로 검증했다. 커밋/푸시 없음.
