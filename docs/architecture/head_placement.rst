헤드 RGB-D BT의 수거함 적재
============================

구현 범위
---------

기존 수거함에 한 개씩 내려놓는 flow를 HeadTaskNode에 연결했다.
placement.enabled=true이면 workflow는 head_rgbd_pick_scan_place이고,
스캔 응답 이후 아래 단계가 이어진다. 비활성화하면 기존 스캔까지만 실행한다.
단계 지정 시작은 추가하지 않았다. RunTask goal은 기존 target_id만 받는다.

1. ClearReader: 오른손의 물체를 유지하며 왼팔을 보정된 대피 관절값으로 이동.
2. PlaceApproach: 오른손을 수거함 상부 경유 EE 자세로 이동.
3. Place: 같은 방향과 XY를 유지하며 수직 하강.
4. Release: 양팔과 왼손 스캐너 그립을 유지하고 오른손만 개방.
5. PlaceRetreat: 측정된 오른손 개방을 확인하고 수직으로 손을 빼냄.
6. Retreat: 열린 손을 유지하며 준비 대기 위치로 복귀.

Retreat 완료 후 flow 성공을 반환한다. 사용자 요청으로 적재 확인은 수행하지 않는다.
성공은 스캔 응답과 개방/복귀 동작 완료를 뜻하며 수거함 내부 관측 증거를 뜻하지 않는다.

기존 Prepare → TrackAndSelect → EstimatePose → Approach → WaitIntercept →
TrackGrasp(Descend/Close) → Lift → ScanPose → ReacquireHeldTarget →
VerifyVisualPick → ScanRetry(TrackBarcode/AlignBarcode, WaitBarcode)는 유지했다.
Lift 전에 별도 파지 성공 판정을 추가하지 않았다.

계획과 실행 계약
----------------

각 동작은 기존 PlanMotion → ExecuteMotion → arm/hand BridgeMotion을 사용한다.
새 노드나 적재 전용 별도 명령 토픽은 만들지 않았다.
정적 placement 설정은 각 노드가 동일 설치 파일로 읽으며 버전 검사를 유지한다.

* 헤드 적재는 고정 EE 경유/개방 자세로 계획한다. 파지 전 ICP pose를 손에 든
  물체의 현재 pose로 재사용하지 않는다. 일반 TaskNode의 기존 Place 구현은 유지한다.
* ClearReader 완료 전에는 PlaceApproach/Place/Release/PlaceRetreat를 허용하지 않는다.
  planner와 executor가 측정 왼팔 관절과 clear_left_joints를 검사한다.
* Place는 hover 자세, Release/PlaceRetreat는 release 자세에서 시작해야 한다.
  위치와 회전 오차는 설정 허용값으로 검사한다.
* PlaceRetreat 전에 실제 손 피드백이 개방 목표에 도달했는지 검사한다.
* 관절/속도/가속도/관측 유효시간/경로 충돌/하위 action 취소 처리는 유지한다.
* SIM 수거함 바닥과 네 벽을 axis-aligned obstacle로 설정했다.
  기존 충돌 모델은 주로 EE 구와 물체 부피 구이므로 실기의 팔 전체·스캐너·변형 봉투에
  대한 완전한 충돌 검증을 의미하지 않는다.

설정
----

SIM의 head_calibration.yaml에 placement 설정을 추가했다.
base_link 기준 hover EE 위치 [0.208, -0.560, 0.865]m,
release EE 위치 [0.208, -0.560, 0.785]m로8cm 수직 하강한다.
손목 방향은 수거함에서 도달 가능한 보정 방향이며 ScanPose 방향과 다르다.
ClearReader의 왼팔 목표는 기존 준비 관절값을 사용한다.

설정 항목은 enabled, calibrated, clear_left_joints, hover_right_pose,
release_right_pose, position_tolerance_m, rotation_tolerance_rad,
left_tolerance_rad, hand_tolerance_rad이다.

실기에서는 calibrated=true와 실제 보정 pose/수거함 영역이 필요하다.
현재 calibrated=false는 SIM에만 허용된다. 이 플래그를 켜는 것만으로 실측 보정이
완료되는 것은 아니다. 전체 설치의 calibration/ownership/hardware gate도 그대로 적용된다.

적재 확인 제외
--------------

사용자는 적재 확인이 필요 없다고 명시했다. VerifyPlace leaf와 관련 관측 대기,
확인 설정/함수를 제거했다. 손 개방 전 위치 검사와 손이 열린 뒤 후퇴하는 실행 조건,
관절/피드백/경로 감시는 유지한다. task 성공 이유에 placement confirmation disabled by
operator를 명시하며, GUI는 flow 완료로 표시한다.

GUI와 SSH
----------

GUI의 시작·정지·재시작은 기존 RunTask action을 사용한다.
새 적재 단계와 실패/취소 직전 마지막 단계도 표시한다.
실제 GUI/ROS 실행과 SSH 터널 명령은 task_gui.rst의 최신 절을 따른다.
SIM은 serve_sim --gui, 실기는 ros2 launch deux_task_gui control.launch.py로 실행한다.
브라우저에는 ROS가 필요 없으며 기본 주소는127.0.0.1:8080이다.
브라우저/SSH 터널 단절 자체는 작업 취소가 아니다.

검증 기록
---------

검증 산출물: validation/head-rgbd/placement/.
최종 실행/테스트 결과는 해당 폴더 verification.json과 result.json을 참고한다.
실제 로봇에 SSH 접속하거나 하드웨어를 구동한 검증은 하지 않았다.

실시간 SIM 화면은 serve_sim --gui가 MuJoCo executor에서 JPEG를 생성하고,
인증된 /api/frame이 해당 이미지 사본만 제공한다. HTTP 스레드는 물리 상태를 조작하지 않는다.
현재 실행 인스턴스는 localhost8090 / ROS_DOMAIN_ID230이며 작업 시작은 사용자가 누른다.
