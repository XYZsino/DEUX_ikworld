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
