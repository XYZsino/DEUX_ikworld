.. note:: 최신 권장 경로는 controlled_perception.rst를 참고하십시오. 아래 분리 인식 및 고정 Scan 설명은 이전 구현 설명입니다.

Detection / ICP 구현 구조 — 2026-10-24
=====================================

이 문서는 perception 단계의 기록입니다. 이후 나머지7개 노드 구현을 추가했으며
최신 전체 구현/설정은 runtime.rst를 참고하십시오.
perception 단계의 구현은 observation_node, detection_node, icp_node 세 노드입니다.
기존 architecture.html/design.html은 2026-10-23의 장기 설계 스냅샷입니다.
그 문서의 8노드 제안에 detection/ICP 두 노드를 추가하면 전체 계획은 10노드이며,
나머지 7노드는 아직 구현되지 않았습니다. 현재 실행 흐름은 아래와 같습니다::

  외부 ZED RGB + depth + CameraInfo ──> observation_node
  측정 관절 / 선택 TF ────────────────> observation_node
                                         │ /task/observation
                                         v
                                    detection_node
                                         │ /task/detection_frames
                                         │ 촬영 header, sequence, bbox,
                                         │ 같은 bundle의 depth + CameraInfo
                                         v
  촬영 시각의 calibrated TF ─────────> icp_node
                                         │ /task/object_detections
                                         └──────────────> observation_node

관측으로 되돌아오는 pose는 detection 입력 계산에 사용하지 않습니다.
observation_node는 새 RGB bundle에서만 관측을 발행하고, 두 생산자는 RGB 촬영
stamp가 증가할 때만 작업을 받습니다. 같은 pose가 여러 관측에 붙더라도 재검출을
유발하지 않습니다. ROS clock 역행 시 pending 입력을 버리고 이전 epoch의
실행 중 결과도 발행하지 않습니다.

각 계산 노드는 동시 실행 1개와 교체 가능한 대기 프레임 1개만 유지합니다.
긴 추론/ICP는 worker에서 수행하고 ROS callback은 계산 완료를 기다리지 않습니다.
입력과 출력 모두 max_age_s를 검사합니다. 이 제한은 실행 시간 보장이나 강제
추론 중단이 아니며, 느린 결과를 폐기하는 정책입니다.

좌표와 모델
-----------

reference_frame은 설치 시 보정한 지지 평면 좌표계이며 Z가 평면 법선입니다.
min_height_m은 이 좌표계의 Z 기준입니다. 기본 base_link의 Z를 지지 높이로
간주하지 않습니다. reference←camera와 base←reference TF가 촬영 시각에 없으면
결과 발행을 거부합니다. 같은 frame을 명시적으로 설정한 경우만 identity가 의미를
가집니다. 과거 fixed_references.json의 평면 변환은 로봇 base 보정이 아닙니다.

model_path는 미터 단위 OBJ이며 model_origin은 mesh 또는 bounds_center를 반드시
선택합니다. model_id에 OBJ SHA256, 원점 정책과 단위를 기록합니다.
원본 v5와 현재 v6 opaque_envelope.obj의 bounds 중심은 확인 당시
(0, -0.000015307, 0.086788809)m로 같았지만, 두 모델 전체가 같다는 의미는 아닙니다.
중심 원점을 선택할 때 회전된 중심 offset이 translation에 포함되므로 downstream은
동일한 원점 정책을 사용해야 합니다. 파일명만 같은 샘플 캐시는 사용하지 않습니다.

품질과 추적의 한계
-------------------

* 기존 24 yaw 초기값 → 5회 coarse → 상위 2개 최대 20회 refine 알고리즘을 이식.
* trimmed RMSE <= 15mm, 10mm 이내 target coverage >= 0.5를 기본 수락 기준으로 사용.
  원본 배치의 quality_warning 기준을 발행 필터로 적용한 것이며 정답 정확도 보장은 아님.
* full_orientation=false. 평면 위 XYZ+yaw만 관측하며 들린 봉투 tilt는 추정하지 않음.
* confidence는 YOLO 점수. ICP 품질은 fit_metrics_valid, trimmed_rmse_m,
  target_coverage_10mm 필드에 별도 기록.
* covariance_estimated=false. 설정된 표준편차를 reference에서 base로 회전한
  공분산이며 데이터로 추정한 통계적 공분산이 아님.
* ID는 거리 gate + 시간 만료 + 일대일 최근접 할당. 프로세스 시작별 ID prefix 사용.
  교차·가림·봉투 중첩에서 신원 유지 보장 없음. Kalman 필터는 아직 미구현.
* 품질 미달/점 부족은 해당 물체 제외. 유효 프레임에 수락 물체가 없으면 빈 배열 발행.
  TF/메시지 오류 및 지연은 결과를 발행하지 않고 진단 토픽에 거부 원인을 기록.

실행
----

ROS 2 Jazzy에서 기존 observation launch를 실행하고 별도 perception launch를 실행합니다.
config/perception.yaml을 설치별 파일로 복사한 뒤 weights, model_path, model_origin,
reference_frame을 실제 값으로 채웁니다. 빈 필수 설정은 시작 시 오류 처리됩니다::

  ros2 launch deux_task_perception observation.launch.py
  ros2 launch deux_task_perception perception.launch.py config:=/absolute/path/perception.yaml

rosbag 사용 시 두 launch 모두 use_sim_time:=true를 지정합니다.
기본 width/height는 640x480, 동기화 slop은 20ms입니다.
Observation와 DetectionFrame은 사용자 정의 메시지이므로 이번 필드 추가 후 관련
생산자/소비자를 모두 재빌드해야 합니다.

YOLO는 선택 의존성입니다. ROS 노드가 실행되는 Python 환경에 torch와 ultralytics가
설치되어야 합니다. rosdep은 scipy를 설치하지만 YOLO/CUDA 패키지를 설치하지 않습니다.
원본 실행 스크립트는 torch==2.10.0, torchvision==0.25.0, ultralytics==8.4.71을
설치하도록 작성돼 있으며, 이 스크립트를 노드에서 실행하지 않습니다.
기존 conda 환경의 Python은 Jazzy Python 3.12와 그대로 혼용할 수 있다고 가정하지 않습니다.
YOLO backend는 생성 시 모델을 한 번 읽고, RGB를 Ultralytics numpy 입력의 BGR로 변환합니다.

로컬에서 확인한 기존 weight:
/home/xyz-ai/Downloads/deux_1021_A_v2.0only_adamw.pt
SHA256: 814f48b2ea1e1f018985d88add5ef709a4ffaaf4e60028753a9bf376a9cdb46d
클래스: 0=b1, 1=m1, 2=s1, 3=s1_1. 기본 ICP 대상은 s1/s1_1입니다.

검증 구분
---------

* validation/jazzy-container.log: Jazzy 빌드 및 단위/합성 ROS 테스트.
  ROS 통신 테스트의 검출기는 합성이며, ICP 계산은 실제 이식 알고리즘입니다.
* validation/yolo-replay.json: 기존 conda 환경에서 실제 weight로 저장 이미지
  1장 CPU 추론. ROS 환경에서의 YOLO 설치·GPU·실시간 속도 검증은 아님.
* validation/icp-replay.json: 저장 RGB-D/기존 bbox의 3프레임 5개 물체를 비교.
  변환 행렬 차이 0. 품질 기준을 통과하는 것은 1개이며 4개는 제외 대상.
  과거 reference 좌표 비교이며 실기 base pose 정답 검증은 아님.

ICP 비교 재현::

  python3 tools/replay_icp.py \
    --root /home/xyz-ai/Downloads/foundationpose2/outputs/zed_ros2_wrapper_2d \
    --model models/breadbag_model_v5/mesh/opaque_envelope.obj \
    --output validation/icp-replay.json

실제 카메라→base 보정, 실시간 ZED 수신, 파지 후 자세 추적, 하드웨어 동작은 미검증입니다.
기존 DEUX 및 외부 detection/ICP 원본은 수정하지 않았습니다.

Coarse 관측점 축소 비교 — 2026-10-25
-----------------------------------

``fit_icp``와 standalone/merged ROS 노드에 선택적 coarse 관측점 제한을 구현했습니다.
``icp_initial_angles`` 기본24, ``icp_yaw_span_deg`` 기본360.0,
``icp_coarse_point_limit`` 기본0(비활성)이며 기존 실행 기본값은 유지합니다.
초기각 범위는 fitted yaw의 제한이 아닙니다.

coarse 반복만 기존 downsample 점군에서 균등 인덱스로 최대400/600점을 선택합니다.
원래 점 수가 제한 이하이면 그대로 사용합니다. 모델 점, 정확한 최근접점 검색,
후보 순위 평가, refine 및 최종 품질 평가는 기존 최대1,600점 기준을 유지합니다.
``validation/icp-multiscale/coarse600.yaml``은 기존 설치 설정 뒤에 적용할 수 있는
180도/12개/600점 파라미터 override 예시입니다. 실기 설정에 자동 적용하지 않았습니다.

저장5세션/51프레임/94물체, 조건별3회 교차 실행의 물체별 중앙값을 비교했습니다.
모든 조건은180도/12개 초기각, coarse5회, refine상위2개/최대20회입니다.
시간은 호스트 ICP 함수 내부이며 YOLO/ROI 추출/파일 읽기/ROS 전송을 제외합니다.

* 기존: 평균188.42ms, P95 284.32ms, 품질 통과47/94.
* coarse400: 평균121.94ms, P95 184.56ms, 통과46/94. 기존 통과1개 추가 탈락.
  양쪽 통과46개에서 yaw(mod180) 차이 최대1.72도, 위치 차이 최대2.68mm.
  탈락 사례까지 포함하면 yaw 차이 최대27.90도이므로 위 수치는 전체 성능 보장이 아닙니다.
* coarse600: 평균137.64ms, P95 213.19ms, 기존과 동일한47개 통과.
  양쪽 통과47개에서 yaw(mod180) 차이 최대9.85도, 위치 차이 최대5.72mm.
* 새 API에서 coarse 비활성일 때 기존180도/12개 저장 결과94개와 변환행렬 차이0.
* ROS2 Jazzy9개 패키지 빌드,64개 테스트 통과. coarse 점 수가 달라도 최종 품질 지표가
  전체 fine 점군에서 계산되는지 독립 검산하는 테스트를 포함합니다.

대시보드: ``validation/icp-multiscale/dashboard.html``.
재현: ``python3 validation/icp-multiscale/benchmark.py`` 후
``python3 validation/icp-multiscale/build_report.py``.
원시 결과/로그는 같은 디렉터리의 results.json 및 jazzy-run.log입니다.
정답 자세가 없는 저장 데이터 비교이며 실기 정확도·실시간 성능 검증은 아닙니다.

독립 voxel 점군 비교 — 2026-10-25
--------------------------------

``icp_coarse_voxel_size_m`` 파라미터를 추가했습니다(기본0.0, 비활성).
0.006이면 원본 ROI에서6mm coarse 점군을 만들고, fine은 원본에서 기존3mm로 만듭니다.
voxel 대표점은 기존과 동일한 첫 입력점입니다. 평균점/centroid로 바꾸지 않았습니다.
두 점군 모두 최대1,600점이며 초과 시 기존 균등 인덱스 제한을 적용합니다.
coarse가40점 미만이면 fine으로 복귀합니다. coarse_point_limit과 동시 사용은 거부합니다.
후보 순위·refine·최종 품질 지표는 fine 점군 기준입니다.
``validation/icp-voxel/voxel6.yaml``은 설치별 설정 뒤에 적용할 override 예시입니다.
기본 설정은 유지하며 하드웨어 실행은 수행하지 않았습니다.

동일51프레임/94물체를3회씩 교차 실행한 결과:

* 기존180도/12개: 평균188.85ms, P95 284.72ms, 통과47개.
* 6mm→3mm: 평균173.73ms(8.0% 단축), P95 259.78ms, 동일한47개 통과.
* 함께 재측정한600점 방식: 평균138.12ms, P95 213.89ms, 동일한47개 통과.
* coarse 평균1,125.4점, fine 평균1,472.3점. 6mm 후1,600점 초과23물체,
  coarse/fine 실제 점 수 동일24물체, 점 부족 fallback1물체.
* 양쪽 통과 물체의6mm 방식 차이: yaw(mod180) 중앙값0.17도/최대24.23도,
  위치 중앙값0.53mm/최대11.36mm. 정답 자세 오차가 아닌 기존 결과 대비 차이입니다.
* 기존 및600점 조건은 직전94개 결과와 변환행렬 차이0.
* Jazzy9개 패키지 빌드,70개 테스트 통과. raw ROI 독립 sampling,
  fine 품질 지표 독립 검산, 희소 coarse fallback, 설정 충돌 검증 포함.

시간 범위는 ICP 함수 내부이며 전처리 ROI 추출/YOLO/ROS는 제외합니다.
이 데이터와 설정에서6mm voxel은600점 방식보다 빠르거나 결과를 더 잘 유지하지 않았습니다.
대시보드와3D bbox: ``validation/icp-voxel/dashboard.html`` 및 ``bbox-comparison.html``.
재현: 같은 디렉터리의 benchmark.py, build_report.py.
