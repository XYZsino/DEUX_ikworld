DEUX lookup table 이동 파지
==========================

현재 실행 진입점은 run_rotation_grasp_trial.bash다. 실제 설치 YAML/URDF와
관측 컨베이어 표면을 사용해 미리 계산한 저장 궤적을 선택해 재생한다.
일반1,587개와 90도 계열 전용590개, 총2,177개 경로를 포함한다.

* 관측 XY와 알려진 base_link -Y 0.09m/s로 접촉 시각을 계산한다.
  실제 파지점 Z=0.97m, X 오차3cm, 접촉까지3초를 유지한다.
* 일반 각도는 XY/도착 시간을 우선하고 같은 위치의 회전 후보를 yaw로 고른다.
* +90도 및 -90도 각각 ±30도 범위는 전용 고정축 경로만 고른다.
  봉투 긴 축의180도 반전은 허용한다. 유효한 전용 경로가 없으면 대기한다.
* 전송 후 궤적·각도·시각은 고정한다. 이후 인식/추종 오차는 기록하며,
  비상정지와 명시적 Action 취소는 유지한다. 자동 반복은 하지 않는다.

기본 실행은 팔/손 명령 없는 미리보기다. ROS 2 Jazzy와 외부 드라이버 overlay를
준비하고, 현재 고정 왼팔 자세로 경로를 검사한 뒤 시작한다.
구체적인 준비/실행 명령과 모델 검증 한계는 TRANSVERSE_GRASP_SETUP.rst를 따른다.
GRID_GRASP_SETUP.rst와 ROTATION_GRID_SETUP.rst에는 보존한 이전 경로를 설명한다.

lookup table은 artifacts/right_grasp_grid_classified/에 있고, 원 탐색/검증은
artifacts/right_grasp_rotation_search/와 artifacts/right_grasp_transverse_search/에 있다.
입력 해시 검증에 필요한 보관 생성기 한 파일은 .setup/rotation-search-original-solver/
아래에 명시적으로 버전 관리한다. 나머지 .setup/ 실행 상태와 backups/는 로컬 파일이다.

최근 실기 기록은 artifacts/thor-five-classified-trials/summary.json에 있다.
순차5회에서 일반1회/전용4회의 팔·손 명령 일정이 제어기 성공으로 끝났다.
별도의 90도 전용 실기1회 기록은 artifacts/thor-transverse-actual-test/summary.json이다.
제어기 성공과 물체를 실제로 잡았다는 성공은 구분하며, 물리적 파지 성공은 미확인이다.

보존한 ROI 추종 경로
-------------------

ROS 2 Jazzy에서 외부 ZED 관측을 받아 오른팔과 오른손을 제어하는 시험 코드다.
외부 DEUX 원본은 수정하지 않는다. docs/는 사용자 요청으로 삭제했으며,
현재 실행 안내는 이 파일과 GRID_GRASP_SETUP.rst에 있다.

ROI 실행 경로
-------------

run_roi_grasp_trial.bash → tools/roi_grasp_trial.py.
ROI에서 선택된 동일 대상의 안정된 관측을 받은 후 XAlign → AwaitGrasp →
ExecuteGrasp → DescendClose → LiftBrake 순서로 진행한다.

* 최초 정렬은 현재 자세에서 상부 대기선으로 이동한다. 이후 X 보정은 Y·Z와
  회전을 유지하며 수행한다. 각 정렬 구간을 끝낸 뒤 최신 관측으로 다시 계산한다.
* base_link -Y 0.09m/s로 취득 지연을 보정한다. 추정 속도·모델 중심 Z는
  파지 시각 선정에 사용하지 않는다. 물리적 파지 높이는 기존 기준 0.97m다.
* 실측 X 오차가 2cm 이내이고 설정된 파지 진입 영역에 들어오면 격자를 선택한다.
  실제 정렬 자세에서 IK·관절 한계·실측 표면 충돌 검사를 통과한 하강 시간을 사용한다.
  시작 시각 = 물체의 격자 도착 시각 − 하강 시간이다.
* 하강 마지막 0.5초에 닫힘 명령을 반복한다. 접촉 시각에 닫힘 완료를 목표로
  하며, −Y 속도를 유지하면서 1.5초 상승 후 0.5초 감속한다.
* 파지 셀을 선택하면 대상 ID·파지점·절대 접촉 시각을 고정한다. 이후 인식은
  로그 전용이며, 전송 전에도 최신 관측으로 재선정하거나 시각을 변경하지 않는다.
  검증된 하강 시간으로 시작 시각을 정하고 전송 후에는 경로도 고정한다. 추적 소실, 위치/관절 오차, 상태 수신 지연,
  손 명령 거부는 기록하며 팔 궤적을 취소하지 않는다. 명시적 Action 취소와
  비상정지는 유지한다. 제어기 자체 고장/드라이버 정지를 무시하지는 못한다.
  손 드라이버가 거부한 명령은 성공한 명령으로 기록하지 않는다.

설정과 실행
-----------

calibration/roi_grasp_policy.json에 X 오차/갱신 기준, 상부 대기 Y/Z,
파지 진입 XY 범위, 하강 시간 후보, 전송 여유 시간이 있다. 기본 파지 진입 영역은
설계값이며 전 영역의 실기 도달성을 입증한 값이 아니다. 각 경로는 전송 전에 검사한다.

Jazzy overlay와 외부 DEUX 메시지 패키지를 먼저 빌드/source한다.
DEUX_INTERFACE_SETUP으로 외부 패키지 setup.bash 경로를 지정할 수 있다.
카메라 보정·ROI·인식·팔/손 드라이버는 별도로 준비한다. 다른 격자/BT/준비자세
실행 서버와 동시에 사용하지 않는다. 기본값은 명령 발행 없는 미리보기다::

  bash run_roi_grasp_trial.bash

명시적으로 실행 가능한 서버를 시작해도 Action 요청 전에는 움직이지 않는다::

  bash run_roi_grasp_trial.bash --ros-args -p enable_execution:=true
  ros2 action send_goal /task/grid_grasp deux_task_msgs/action/MoveToReady '{}' --feedback

왼팔은 저장 기준 자세와 일치해야 한다. 다른 왼팔 자세는 기존 도구로 검사한
stationary_left_file을 명시적으로 전달한다. 새로운 경로도 그 왼팔 자세로 검사한다.
이 실행 경로는 .setup의 준비자세/ROI 파일을 직접 읽지 않는다. ROI 판정은 기존
인식 노드의 tracking 상태를 사용하므로 인식 프로세스의 보정/ROI 입력은 필요하다.
한 번 완료한 서버는 재시작해야 한다. 자동 복귀·파지 재실행은 없다.
실행 전 일시적인 관측/피드백 지연이나 계획 거부는 로그 후 재시도한다. 선정 후에는 같은 셀과 접촉 시각만 사용한다.
접촉 시각을 놓쳤다면 다른 물체로 자동 전환하지 않고 경고와 함께 취소를 기다린다.
관절 속도 0.03rad/s 기준은 진단용이며 정렬 시작을 차단하지 않는다.
120초 대상 대기 시간도 경고용이며, 명시적 취소까지 계속 기다린다.
전송 전 IK·관절/충돌 검증 및 하드웨어 모드 확인은 유지한다.
제어기 거부/고장은 자동 재실행하지 않고 실제 종료 원인을 보고한다.

상태는 /task/grid_grasp/preview 및 GUI의 ROI 패널에서 확인한다.
실행 결과와 시계열 로그는 artifacts/roi-grasp-runs/에 저장한다.
roi_policy_file 및 output_directory 파라미터로 경로를 지정할 수 있다.

소스와 검증
-----------

* tools/roi_grasp_planner.py: ROS 없는 선정·정렬·하강 계획과 모델 검사.
* tools/roi_grasp_trial.py: 비동기 계획, 단계 전환, 예약 실행.
* tools/grid_grasp_trial.py, parallel_grid_hand.py: 공통 관측·오류 기록·손 일정.
* artifacts/right_grasp_grid_fast/: 입력 해시로 확인하는 기존 격자 후보.
* src/deux_task_core/, deux_task_runtime/, deux_task_perception/, deux_task_gui/:
  운동학·ROS 인터페이스·인식·관제. 나머지 기존 실행 경로는 별도로 남아 있다.

오프라인 테스트::

  PYTHONPATH=src/deux_task_core:tools /usr/bin/python3 -m pytest \
    tools/test_roi_grasp_planner.py tools/test_parallel_grid_hand.py -q

실제 URDF를 사용한 경로 검증도 소프트웨어 모델 검사다. 전체 자기충돌·가려진
설비·파지 물체 형상·손의 실제 닫힘 시간·실기 파지 성공을 보장하지 않는다.
아래는 ROI 경로 초기 구현 당시의 검증 기록이다. 당시 실기 명령은 보내지 않았다.
오프라인 22개 테스트 통과. 원격 Thor Jazzy의
격리 도메인221~224에서 가짜 팔/손으로 새 ROI 정상 실행, 추적 소실·ID 변경,
0.35rad 관절 오차에도 일정 완료를 검증했다. 기존 격자 경로의 관절 오차 시험도
통과했다. 실제 손 프로파일/상태 검사는 합성 시험에서 대체했으므로 실기 증거가 아니다.
로컬에는 Jazzy 및 AGENTS.md에 언급된 tools/validate_jazzy.sh가 없다.
