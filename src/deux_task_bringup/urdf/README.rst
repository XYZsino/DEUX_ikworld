DEUX + ZED 2i 고정 베이스 URDF
==============================

``deux_fixed_zed2i.urdf``는 2026-10-28 기준 SIM 실행에서 사용하는 전체 로봇과
ZED 2i 조립체의 URDF 스냅샷이다. 원본 DEUX description은 변경하지 않았다.
로봇 관절/파지점/장착 transform은 실행 URDF에서 보존했다.

* lift_joint는 고정이며 base_link 기준 origin은 [0.114, 0, 0.543164]m이다.
* 이동 베이스 조향/구동 링크가 제거된 조작 전용 SIM 모델이다.
* 카메라는 lift에 장착된다. 아래 45도 pitch, 눈 디스플레이 기준 수직 간격 3mm.
* 눈 디스플레이 기준 위치/전방 오프셋은 SIM 추정값이다. 실측 보정 완료 모델이 아니다.
* 오른손/왼손 grasp 프레임과 ZED 좌우 광학 프레임을 포함한다.
* URDF에는 초기 준비 관절각이나 준비 동작 경로가 저장되지 않는다.

메시 경로 및 사용
----------------

로봇 메시는 기존 ``deux_description`` 패키지의 ``package://`` URI를 사용한다.
따라서 DEUX description 패키지가 설치되고 해당 workspace가 source되어야 한다.
SIM의 경량화 메시 대신 원본 description 메시를 참조하며, 카메라 메시는 이
패키지의 ``meshes/zed2i.stl``이다. 로봇 메시를 이 레포에 중복 복사하지 않는다.
컨테이너 절대 경로와 MuJoCo 전용 compiler 설정은 포함하지 않는다.

colcon 빌드 후 다음 경로에 설치된다::

  $(ros2 pkg prefix deux_task_bringup)/share/deux_task_bringup/urdf/deux_fixed_zed2i.urdf

robot_state_publisher/RViz 등의 description 입력으로 사용할 수 있다.
현재 SIM 생성기와 real launch의 기본 URDF를 자동 교체하지는 않는다.
SIM은 기존처럼 실행 설정에 맞춰 별도 URDF를 생성한다.
실기는 실제 리프트/장착 보정과 원래 이동 베이스 구성을 반영한 description을 사용한다.
장착값을 변경하면 고정 스냅샷도 다시 생성해야 한다.
카메라 생성 코드와 설정은 ``deux_task_core.camera_mount`` 및
``config/zed2i_mount.yaml``이며 상세 내용은 ``docs/architecture/zed2i_mount.rst``를 참고한다.
