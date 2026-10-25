단계별 인식과 Behavior Tree — 현재 구현
======================================

이 문서가 이전 runtime.rst/design.html의 고정 Scan 설명보다 우선합니다.
현재 권장 경로는8개 노드입니다: perception_node, state_observation_node,
barcode_bridge_node, task_node, motion_planner_node, execution_node,
arm_bridge_node, hand_bridge_node. 기존 standalone observation/detection/ICP는
독립 실행 옵션으로 남아 있습니다. 같은 출력 토픽에 두 인식 경로를 동시에 실행하지 마십시오.

정적 설치 설정은 여전히 파일입니다. scene_node나 설정 배포 토픽은 없습니다.
PerceptionRequest는 태스크가 실행 중 보내는 처리 요청이며 설치 설정과 별개입니다.

RGB-D와 처리 요청
-----------------

* perception_node는 등록 RGB/depth/각 CameraInfo를 동기화하고 YOLO를 한 번 실행합니다.
  bag_classes와 barcode_classes를 분리합니다. weight의 b1이 바코드라는 근거가 없으므로
  barcode_classes는 설치 템플릿에서 명시적으로 지정해야 합니다.
* /task/detections: DetectionArray. 촬영 header/sequence, request_id, detector_id, bbox만 포함.
  검출 메타데이터는 TF/ICP 성공 여부와 독립적으로 먼저 발행됩니다.
* ICP와 바코드 표면은 서로 다른 worker에서 실행합니다. worker마다 실행1개와 최신 대기1개.
  원본 RGB-D는 프로세스 안에서 공유하고 이미지가 포함된 DetectionFrame은 외부로 보내지 않습니다.
* /task/perception_request: PerceptionRequest. header, request_id, target_id,
  mode(locate/track/barcode/idle), expires_at. 최대120초 lease를 수락하며 task는60초로 요청합니다.
  작업이 길면 잔여10초 전에 갱신합니다. 입력 요청은 시각순으로만 수락합니다.
  요청 교체/만료/시계 역행 후 이전 token의 계산 결과는 폐기합니다.
* 요청 없는 시작 상태와 lease 만료 후에는 수동 명령 없이 locate로 돌아갑니다.
  idle lease 동안은 처리하지 않습니다. barcode에서는 ICP를 독립적으로 계속 실행하여
  봉투 관측을 끊지 않습니다. 현재 locate/track은 같은 yaw ICP를 사용합니다.
* /task/object_detections: ObjectDetectionArray. merged 경로는 request_id도 보존합니다.
  yaw-only 결과의 full_orientation=false는 유지합니다. 기존 standalone 생산자는 request_id가 비어 있습니다.

라벨 표면과 정렬
---------------

BarcodeTarget은 원본 촬영 시각/sequence/request_id/target_id, base 좌표의 표면 pose,
plane_rmse_m, valid_fraction, orientation_valid를 포함합니다.

1. 같은 RGB-D의 라벨 bbox 내부60% 영역을 역투영합니다.
2. 유효 depth70% 이상, 최소30점, 중앙 depth에서15mm 이내 점70% 이상을 요구합니다.
3. 평면 RMSE3mm 이하 및 두 번째 주축 분산을 검사합니다.
4. 라벨 중심을 포함하는 봉투 bbox가 정확히 하나여야 합니다. 최근 ICP 목표의
   카메라 투영점도 그 bbox 안에 있어야 합니다. 같은 bbox의 다른 track은 거부합니다.
5. 최근 track과 라벨의3D 거리가20cm를 초과하면 거부합니다. 최근 track은
   track_max_age_s(기본0.6초) 이내여야 하며 동일 프레임 track을 보장하지 않습니다.
6. 표면 +Z는 카메라를 향하고 +X는 이미지 수평축을 평면에 투영한 방향입니다.
   바코드 줄무늬 방향이나 봉투 전체 방향을 추정했다는 뜻이 아닙니다.

AlignBarcode는 관측 표면, read_distance, 보정된 left_ee_from_reader로 왼팔 목표를
계산합니다. 오른팔 목표는 현재 측정 위치를 유지합니다. bbox 중심만으로 명령하지 않습니다.
계획 시 라벨의 유효성/최신성을 검사하고 MotionPlan에 snapshot을 보존합니다.
실행은 같은 요청의 최신 BarcodeTarget을 지속 검사하며 위치/각도 이탈,
관측 단절 또는 다른 request_id 수신 시 중단합니다.

겹침·심한 변형·가림·추적 ID 교차를 해결하는 학습 기반 association은 구현하지 않았습니다.
이 경우 보수적으로 실패할 수 있습니다. 임의 카메라 extrinsic이나 손·리더 보정값을 만들지 않습니다.

태스크 트리
-----------

behavior_tree.py는 await 중 RUNNING인 메모리 Sequence/Leaf/Retry 구현입니다.
ROS executor를 막지 않는 비동기 작업이며, 별도 주기 tick 기반 BT 프레임워크는 아닙니다.

::

  Transfer: Sequence
    GraspAndVerify: Leaf
      Approach → Descend → Close → Lift → VerifyLift
      새 물체 관측2개로 상승 확인
      실패 시 원래 지지면에 남은 경우만 Recover → Release; 최대2회
    ScanRetry: Retry(3)
      ScanSequence: Sequence
        ScanSearch: barcode 요청 → 새 라벨 표면 대기
        AlignBarcode: PlanMotion → ExecuteMotion
        WaitBarcode: ScanSession 활성화 → 실제 BarcodeRead 대기
    PlaceAndVerify: Leaf
      track 요청 → Place → Release → Retreat → VerifyPlace
      새 관측2개 + 적재 영역 + EE 분리 확인

탐색 실패/판독 시간 초과만 스캔 재시도를 허용합니다. 정렬 모션 실패, 취소,
관측/실행 감시 오류는 상위로 전달하여 태스크를 종료합니다.
ScanSearch는 정지한 상태에서 새 영상을 기다립니다. 라벨이 뒤를 향하면 자동 회전
탐색하지 않으며,3회 후 실패합니다. 기존 고정 라벨 Scan 계획 함수는 호환용으로 남지만
태스크의 새 기본 경로에서는 호출하지 않습니다.

BarcodeRead는 활성 session_id/target_id, 수신 시각 범위, simulated 정책이 맞는
실제 리더 문자열일 때만 성공으로 사용합니다. 표면 검출 성공은 판독 성공이 아닙니다.
Place는 full_orientation=true가 계속 필요합니다. 현재 merged yaw ICP만 연결하면
그 조건을 만족하지 못하므로, 파지 후6D 인식 생산자 통합은 여전히 필요합니다.
실기 자동 이송 전체가 준비되었다고 해석하면 안 됩니다.

경량 관측 및 실행
-----------------

StateObservation.msg와 perception_node/state_observation_node 실행 항목을 등록했습니다.
RuntimeNode의5개 소비자는 /task/state_observation을 구독하고 PlanMotion도 이 타입을
사용합니다. 이미지가 제어 관측과 계획 Action에 포함되지 않습니다. 메시지 변경으로
관련 overlay 및 외부 클라이언트를 재빌드해야 합니다.

::

  ros2 launch deux_task_perception controlled_perception.launch.py config:=/absolute/controlled_perception.yaml
  ros2 launch deux_task_bringup task.launch.py scene_config:=/absolute/installation.yaml hand_mapping:=/absolute/hand_mapping.yaml

인식 템플릿은 src/deux_task_perception/config/controlled_perception.yaml입니다.
모델/클래스/카메라·EE TF 및 설치 설정을 채워야 합니다. 드라이버는 별도 실행입니다.
기본 dry_run=true, hardware_enabled=false를 유지합니다.

구조도와 검증
-------------

* nodes.html: Graphviz/WASM 자동 배치, 노드·토픽 이분 그래프. 제어/인식/하드웨어/설정/전체 탭,
  검색, 확대·축소, SVG 저장. 코드 정적 분석 결과이며 실행 중 rqt 캡처가 아닙니다.
  nodes.graph.json은 연결 목록, nodes.dot은 전체 Graphviz 입력입니다.
* scenario.html: archify workflow. 스캔 retry와 협력 노드, 파지/적재 검증 조건.
  본문은 한국어이며 고정 Viewer UI는 영어입니다. source: scenario-final.workflow.json.
* ROS 2 Jazzy disposable container:7개 패키지 빌드,49개 테스트 통과.
  신규8개는 표면 추정/연결 거부, 공통 검출·요청 폐기, 라벨 식별 보존,
  경량 관측, BT 재시도, 오른팔 유지 정렬 및 실행 라벨 감시를 검사합니다.
  기존 태스크 통신 시험도 새 AlignBarcode 경로로 갱신했습니다.
* 실제 YOLO 클래스 의미, ZED 실시간 입력/보정, 로봇 명령 및 하드웨어 성공률은
  이번 변경으로 검증하지 않았습니다. 합성 테스트는 실제 물리 결과를 의미하지 않습니다.
