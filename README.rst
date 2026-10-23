DEUX external-camera observation overlay
========================================

세션 이어가기
-------------

현재 작업 인계와 다음 detection/ICP 구현 자료는 ``docs/HANDOFF.rst``를 먼저 읽으세요.
구조도는 ``docs/architecture/architecture.html``, 상세 설계는 ``docs/architecture/design.html``입니다.

현재 구현 범위
--------------

이 폴더는 DEUX/zed-ros2-wrapper 원본을 수정하지 않는 독립 ROS 2 overlay입니다.
``deux_task_msgs``와 ``deux_task_perception``의 observation/detection/ICP 노드를 추가했습니다.
모터 명령·bringup·카메라 드라이버를 시작하지 않습니다.

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
icp_node가 검출·yaw ICP·거리 기반 ID 연결을 수행합니다. Kalman 필터·바코드
브리지는 아직 구현하지 않았습니다. 기존 sim tracker를 연결하는 경우
동일 pose sequence/header를 재갱신하지 않아야 합니다. 본 Observation은 RGB-D 및
선택 측정 묶음이며 기존 JSON ObservationFrame과 직접 호환되는 메시지는 아닙니다.
이후 BT adapter에서 유효성·시간·측정 축을 확인하고 변환해야 합니다.

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

촬영 프레임의 depth를 DetectionFrame에 함께 전달하며, 추론/ICP worker는 실행 중
1프레임과 최신 대기 1프레임으로 제한합니다. yaw-only 결과는 full_orientation=false,
설정 공분산은 covariance_estimated=false입니다. ObjectDetection의 ICP 품질 필드
추가로 해당 메시지를 사용하는 외부 패키지도 재빌드해야 합니다.

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
  colcon build --symlink-install --packages-select deux_task_msgs deux_task_perception
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

2026-10-24 최신 검증: ROS 2 Jazzy에서 두 패키지 빌드, 24개 테스트 통과.
신규 테스트에는 합성 검출기→실제 yaw ICP의 ROS 통신과 TF 누락·만료 거부가
포함됩니다. 저장 데이터 ICP 비교와 실제 YOLO 단일 이미지 추론 결과는
``validation/icp-replay.json``, ``validation/yolo-replay.json``에 있습니다.
YOLO의 Jazzy 환경 통합 실행 및 실제 ZED/DEUX 하드웨어 동작은 미검증입니다.

ROS 환경에서 빌드 후::

  source install/setup.bash
  python3 -m pytest src/deux_task_perception/test -q

ROS 통신 테스트(test_ros_smoke.py)는 640x480 BGRA와 OpenNI depth, CameraInfo를
합성 발행해서 Observation 수신/변환 및 URDF/관절 없이 수신 가능함을 검증합니다.
로봇 명령을 발행하지 않습니다. 실제 드라이버와 테스트는 별도 ROS_DOMAIN_ID에서
실행하십시오. 예: ROS_DOMAIN_ID=91 python3 -m pytest src/deux_task_perception/test -q

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
