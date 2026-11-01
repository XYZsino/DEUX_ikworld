회전 후보를 포함한 저장 격자 파지
===============================

현재 run_rotation_grasp_trial.bash는 90도 전용 분기도 포함한다.
새 분기 안내는 TRANSVERSE_GRASP_SETUP.rst를 따른다. 아래 문서는
run_normal_rotation_grasp_trial.bash에 보존한 일반 회전 경로의 설명이다.

실제 준비자세에서 시작하는 별도 실행 경로다. 기존 ROI XAlign 실행기는 보존한다.
XY 118개 위치 / 저장 궤적 1,587개. 현재 왼팔 자세 검사에서 탈락한 궤적은 제외한다.

선택 순서
---------

1. 촬영 시각의 XY를 알려진 (0,-0.09,0)m/s로 현재 시각까지 보정한다.
2. X 오차 3cm 이하, 격자까지 ETA 3.8..10초인 위치만 후보로 남긴다.
3. X 오차 최소, 그다음 ETA가 가장 짧은 위치를 고른다.
4. 같은 위치에서는 관측 yaw만 적용한 목표 회전에 가장 가까운 저장 자세를 고른다.
   SO(3) 전체 회전 거리로 비교하며, 동률이면 최대 가속도 사용률이 낮은 후보를 고른다.
   관측 roll/pitch는 회전 선택에 쓰지 않는다. yaw 차이는 탈락 조건이 아니다.

기준 대비 전체 30도 제한은 저장 궤적의 검사 조건이다. 추정 yaw가 그 범위를
벗어나도 검증된 후보 중 가장 가까운 것을 고르며 새 관절 경로를 만들지 않는다.
Z는 실제 파지점 base_link 0.97m다. 시작=격자 도착 시각-3초,
닫힘 2.5..3초, 접촉/상승 시작 3초, 상승 완료 4.5초, 정지 5초.
전송 후 위치·회전·절대 시각을 고정하며 이후 인식 변화는 로그로 남긴다.
비상정지 및 명시적 Action 취소는 유지한다.

준비와 실행
-----------

기존 /task/grid_grasp 서버와 동시에 실행하지 않는다. 먼저 실제 고정 왼팔로
전 후보를 재검사한다. 이 도구에는 팔/손 명령 인터페이스가 없다::

  source setup_env.bash
  ROS_DOMAIN_ID=12 python tools/audit_rotation_grid_left.py

명령 발행 없는 미리보기::

  bash run_normal_rotation_grasp_trial.bash --ros-args \
    -p stationary_left_file:=$PWD/.setup/grid-grasp-rotation-test/stationary-left.json

이번 배포는 여기까지다. 미리보기에서 actual ready 오차를 확인한다.
오른팔을 저장 준비자세로 이동시키거나 손을 여는 동작을 자동으로 수행하지 않는다.
실제 실행에는 아래 enable 및 별도 Action 요청이 모두 필요하다::

  bash run_normal_rotation_grasp_trial.bash --ros-args -p enable_execution:=true \
    -p stationary_left_file:=$PWD/.setup/grid-grasp-rotation-test/stationary-left.json
  ros2 action send_goal /task/grid_grasp deux_task_msgs/action/MoveToReady '{}' --feedback

새 라이브러리는 artifacts/right_grasp_grid_rotations/이며 기존39개 라이브러리는 보존한다.
연속80개 격자만 사용하려면 recommended_only:=true를 전달한다.
새 라이브러리는 예전 준비/왼팔 감사 도구의 하드코딩된 입력과 다르므로,
예전 감사 파일을 복사하거나 manifest 해시만 바꿔서 사용하면 안 된다.
준비 이동이 필요할 때도 이 라이브러리에 맞춘 새 검증을 수행해야 한다.

근거와 한계
-----------

artifacts/right_grasp_rotation_search/에 원 탐색·독립 검증·지도·분할 계산 기록을 보존한다.
기존 설치 YAML/URDF/관측 표면 및 생성 소스 해시를 실행기 시작 시 확인한다.
실측 왼팔 재검사는 기존 충돌 모델에 한정된다. 전체 몸통·자기충돌·숨은 설비·
파지 물체 형상 및 실제 손 닫힘/파지 성공은 별도 실기 검증 대상이다.
