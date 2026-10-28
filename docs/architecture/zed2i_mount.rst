ZED 2i 장착 및 영상 입력
=======================

사용자 확인 조건은 수평에서 아래로45° pitch, 카메라 바닥과 눈 디스플레이
사이3mm 간격입니다. 좌우 중앙에 장착합니다. 원래 head_camera_pose를 사용하지
않으며 ZED 자체 프레임을 URDF에 추가합니다.

``src/deux_task_bringup/config/zed2i_mount.yaml``의
``eye_display_top_xyz``와 ``mount_forward_offset_m``은 이미지/CAD 기반 sim 추정값입니다.
사용자가 확인한 실측 외부 파라미터가 아닙니다. 현재3mm는 회전한 카메라 mesh의
최저점과 눈 디스플레이 상단 기준점 사이의 수직 간격으로 구현했습니다.
실기에서는 눈 디스플레이 기준점/카메라 장착 위치를 측정하고 외부 보정을 해야 합니다.
``calibration_confirmed: false``로 이 상태를 기록합니다.

공식 Stereolabs zed-ros2-description의 ZED 2i STL과 광학 프레임 오프셋을
사용합니다. 원본 commit 및 Apache-2.0 라이선스는 bringup/meshes/README.rst와
bringup/licenses/zed_description.LICENSE에 보존합니다.

프레임 및 sim
-------------

lift → zed_eye_display_top_reference → zed_camera_link → zed_camera_center
→ zed_left_camera_frame → zed_left_camera_frame_optical.
오른쪽 카메라도 별도 프레임으로 추가하며 좌우 광학 중심 간격은0.12m입니다.

sim RGB-D는 ``zed2i_left`` 카메라에서 생성합니다. ``top_cam``으로 자동 대체하지
않습니다. Observation, RGB, depth, CameraInfo, TF 모두
``zed_left_camera_frame_optical``을 사용합니다. sim 깊이는 렌더링된 metric depth로,
실제 ZED stereo matching 오차/결측의 재현은 아닙니다. 640×480, 수평100°는
명시적 sim pinhole 근사값이며 실기 CameraInfo로 대체합니다.

``load_sim``이 외부 로봇 URDF의 별도 사본에 조립체를 추가합니다. 원본 DEUX 및
DEUX_mujoco 파일은 수정하지 않습니다. 생성 결과는 실행 폴더의 ``robot.urdf``와
``camera-mount.json``입니다. 카메라 mesh는 시각/충돌 요소로 들어갑니다.
카메라 질량/관성을 측정하지 않았으므로 실기 동역학 모델 검증은 별도입니다.

별도 URDF 생성
--------------

ROS overlay를 source한 뒤::

  SHARE=$(ros2 pkg prefix deux_task_bringup)/share/deux_task_bringup
  ros2 run deux_task_core add_zed2i_urdf \
    --input /path/to/expanded_robot.urdf \
    --output /path/to/robot_with_zed2i.urdf \
    --config "$SHARE/config/zed2i_mount.yaml" \
    --mesh "$SHARE/meshes/zed2i.stl"

입력은 xacro가 확장된 URDF여야 합니다. 입력 경로로 덮어쓰기는 거부합니다.
출력 mesh 경로는 실행 환경의 절대 경로이므로 배포 대상에서 다시 생성하십시오.

실기 전환
---------

실기 로봇 description에 측정·보정한 장착 transform을 반영하고 ZED wrapper를
별도로 실행합니다. robot_state_publisher와 wrapper가 동일 카메라 TF를 중복
발행하지 않도록 TF 소유자를 하나로 정합니다. 드라이버 버전의 image/depth
header.frame_id와 실제 TF 트리도 확인해야 합니다.

``head_rgbd.launch.py``의 observation_config에는 실제 드라이버의 rectified RGB,
registered depth, 양쪽 CameraInfo 토픽과 실제 영상 크기를 설정합니다.
인식 노드는 CameraInfo와 취득 시각의 base→camera TF를 사용합니다.
기존 헤드 카메라 외부 파라미터나 sim100° intrinsics를 실기에 복사하면 안 됩니다.
실기 영상의 모델 성능, 깊이/시간 동기화, 작업 영역/관절/충돌 제한은 별도 검증 대상입니다.

검증 근거
---------

``validation/head-rgbd/zed-mount-check/validation.json``에서 URDF와 MuJoCo
광학 변환 최대 원소 오차3.33e-16, baseline0.12m, 광축
[0.70710678,0,-0.70710678]을 확인했습니다. 카메라 아래 방향45°에 해당합니다.
단위 시험은 mesh 최저점 기준3mm, 방향, baseline, 중복 장착 거부를 확인합니다.
이 결과는 sim 기하 검증이며 실제 장착/캘리브레이션 검증이 아닙니다.

버전 관리되는 전체 URDF
-----------------------

``src/deux_task_bringup/urdf/deux_fixed_zed2i.urdf``에 ZED를 포함한 고정 베이스
SIM 스냅샷을 제공한다. 임시 컨테이너 경로 대신 ROS package URI를 사용한다.
로봇 메시는 외부 deux_description 패키지가 필요하며 카메라 메시는 이 레포에 있다.
리프트 고정 높이/사용 제약과 재생성 조건은 같은 디렉터리의 README.rst에 기록했다.
실기 보정 모델이 아니며 기존 실행기의 URDF 선택을 자동 변경하지 않는다.
