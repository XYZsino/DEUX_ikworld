헤드 RGB-D 기반 작업 전환 — 진행 기록
====================================

완료 조건 (사용자 요청 범위 유지)
--------------------------------

1. 기존 헤드 카메라를 RGB-D로 취급하고, 영상에 나타난 뒤에만 물체를 검출/추적.
   실행 중 물체 정답 포즈·물체명·segmentation ID를 perception/controller에 전달하지 않음.
   정답은 오프라인 학습 annotation과 별도 평가에만 사용.
2. sim에서 빵 봉투/바코드 YOLO 모델을 실제 학습하고 별도 held-out 장면으로 평가.
   이미지 기반 tracker를 실제 실행 경로에 연결하고 ID 유지/소실을 검증.
3. 추적 대상으로부터 작업 대상을 고르고 그 대상의 RGB-D ROI로 ICP 수행 후 파지.
4. 파지 후 ICP의 지속 성공에 의존하지 않고 오른팔을 편 스캔 자세로 이동.
   관측된 봉투/바코드 및 대상 연관관계로 파지 후 시각적 유효성을 판단.
5. 바코드 추적과 실제 RGB-D 표면 추정으로 리더 정렬 및 스캔.
6. 이 경로에 맞춘 BT/관련 ROS 노드/GUI 업데이트, 실제 시나리오 MP4 녹화.
7. 완성된 코드 기준 노드 메시지 구조도 및 BT 시나리오 구조도 작성/검증.

현재 확인한 기존 문제
--------------------

world.configure는 truth pose로 파지 설정을 계산하고 node의 센서도 visible truth pose를 전달합니다.
기존 PerceptionNode는 barcode 모드에서도 ICP를 계속 돌려 라벨 연관관계에 씁니다.
둘 다 새 작업에서는 교체 대상입니다. 기존 외부 DEUX/DEUX_mujoco 저장소는 수정하지 않습니다.
헤드 장착은 기존 sim URDF에 정의된 top_cam을 사용합니다. 외부 ZED 보정값으로 재사용하지 않습니다.

구현 중
-------

* rgbd.py: head RGB + metric depth + intrinsics + camera extrinsics만 취득.
* train_dataset.py: 별도 오프라인 렌더에서 가시성 bag mask와 실제 UV 바코드 영역으로 annotation.
  객체 배치/회전/조명/로봇 자세 변화 및 negative 장면. train/val/test seed 분리.
* yolo_backend.YoloTracker: YOLO11n+ByteTrack, 취득 시각 기준 ID 만료.
  관측되지 않은 예측 bbox를 작업 대상의 최신 관측으로 발행하지 않음.
* 학습 환경: /home/xyz-ai/.cache/deux-vision-venv (sam3의 CUDA torch를 읽기 전용 상속).
  ultralytics8.3.228, MuJoCo3.13.0. GPU GB10에서 CUDA 행렬 연산 확인.
* 데이터/실험 증거: validation/head-rgbd/. 아직 학습 완료나 시나리오 성공을 주장하지 않음.

참고: https://docs.ultralytics.com/datasets/track/ — tracker 자체를 별도로 학습하는 대신
검출 모델을 학습하고 model.track()으로 연관시킵니다. 공식 ByteTrack 사용.

2026-10-26: 최종 segmentation 모델 및 실제 ROS 실행
--------------------------------------------------

YOLO11n-seg를 sim에서60epoch 학습 완료했습니다. 독립 test split의 box mAP50
0.9495, mask mAP50 0.8997입니다. 실영상 성능 검증은 아닙니다.
최종 weights: models/bread_barcode_yolo11n_seg_sim.pt.
SHA256: bcf96491081ac0c6e3b9d764f4e29a35c6d0c0d75653e70e994cda8ae09a03ab.
ByteTrack은 별도 학습하지 않고 이 모델의 검출을 연결합니다.

scenario-05는 최종 모델로 Prepare, ObserveScene, TrackAndSelect, EstimatePose,
Approach, Descend, Close, Lift, ScanPose까지 실제 ROS Action을 실행했습니다.
VerifyVisualPick에서 시각 증거 timeout으로 실패했습니다. 전체 성공 아님.
/task/object_detections의 유일한 publisher는 icp_node이며 ICP 발행2회였습니다.
원본 ICP 촬영 시각을 보존한 bounded grasp snapshot을 사용하고 이후 ICP를
중단합니다. 이 고정 소스 설정의 최대 허용 나이는15초이며 움직이는 컨베이어에
그대로 적용할 수 없습니다.

실패 분석: 파지 가림 중 visual-1이 만료됐고 재등장 봉투는 visual-9로 검출됐습니다.
기존 ID를 임의로 살리지 않습니다. 새 ReacquireHeldTarget 단계는 barcode 모드에서
받은 한 프레임의 전체 VisualEvidenceArray 후보 중 측정 손 근처20cm 이내이고
파지 전 표면보다4cm 이상 상승한 후보가 유일하며3촬영 프레임 연속일 때 재선택합니다.
이는 시각적 손–봉투 연관이며 물체 고유 신원/접촉력 증명이 아닙니다.
이후 원래의 봉투+바코드 확인 및 추적/리더 검증을 그대로 수행합니다.
후보 메시지는 RGB-D 표면 중심만 포함하며 ICP/full object pose가 아닙니다.

새 ROS 후보 발행 및 모호한 후보/바닥 봉투/invalid depth 거부 시험을 포함한
전체77개 테스트 통과: validation/head-rgbd/regression-reassociation.log.
이 변경 이후 물리 시나리오는 아직 미검증입니다.
scenario-05의 관측 최대 환경 침투는0.024924m이며 물리 경고는0입니다.
접근 경로의 환경 접촉, 스캔 자세에서 바코드 노출 및 실제 reader 판독을
해결해야 합니다. MP4/최종 구조도/기본 serve_sim 전환도 남아 있습니다.

추가 분석: 기존 max_environment_penetration_m는 봉투를 포함한 모든 비로봇
접촉입니다. scenario-05의359개 녹화 상태 replay에서 접촉은 손가락–bread_3만
발견됐고 최대22.89mm였습니다. 이 표본 검사만으로 모든 제어 스텝의 설비 접촉이
없다고 단정하지 않습니다. 새 실행부터 max_fixture_penetration_m와
max_object_penetration_m를 분리 기록합니다. 물체 registry는 이 감사 지표에만
사용하며 제어 인식에는 사용하지 않습니다. 상세 contact-audit/result.json.

serve_sim의 기본 perception을 head-rgbd로 변경하고 --weights를 필수 입력으로
검사합니다. head_stack.py에서 검증/interactive 노드 구성을 공유합니다.
기존 경로는 --perception legacy-truth로 명시해야 합니다.
실행 중 기존 GUI preview는 교체하지 않았습니다.

scenario-06 진행 중: 파지 가림 후 재선택 및 스캔 어깨0.55rad/팔꿈치-0.25rad/
손목5번-1.0rad를 시험합니다. 오프라인 시야 비교에서 이 후보는 봉투와 바코드를
함께 검출했으나 물리 실행 결과로 아직 검증되지 않았습니다.
process: docker exec deux-head-rgbd python3 -m deux_task_sim.validate_head,
로그 validation/head-rgbd/scenario-06.log. 재실행 전에 docker top으로 생존 확인.
현재 실행은 공유 head_stack 추출/접촉 지표 분리 전 코드를 로드했습니다.
다음 실행에는 두 변경도 포함되므로 regression/startup 확인이 필요합니다.

scenario-06 결과: ScanPose 중 hand_bridge가 right_hand_third_joint_1 한계 초과로
정지했습니다. 최초 녹화 측정 -2.269932231rad, 보정 최소 -2.269928rad.
목표가 URDF 최소값 -2.268928rad였으므로 구동 중 한계 접촉 여유가 부족했습니다.
손 open/closed의 접힌 third joint1/2 목표를 최소값에서0.02rad 안쪽으로 옮깁니다.
한계/신선도/실행 감시 조건은 완화하지 않습니다. 다음 scenario-07에서 재검증합니다.

2026-10-26: 통신도/녹화 화면 및 스캔 간섭 분석
-------------------------------------------

* docs/architecture/head_rgbd_nodes.html: Graphviz 기반 토픽/Action 통신도.
  scenario-07 live ROS 그래프 + Action 계측을 사용했습니다. 녹화 전용 driver
  구독은 실행 경로와 분리하여 표에 표시합니다. 탭3개, 메시지 클릭, 검색,
  가로 넘침/JS 오류 검사 통과. Graphviz는 검증 컨테이너에 설치했습니다.
* docs/architecture/head_rgbd_bt.html: Archify workflow.
  head_rgbd_bt.delivery.json의9/9 showcase 검증 및4개 desktop viewport 검사 통과.
  light1440x900/dark2048x1320 이미지 검토 완료. 고정 메뉴는 영어입니다.
* scenario-05/dashboard-rgbd.html 및 scenario-rgbd.mp4: 원래 실패 기록에
  헤드 영상/실제 검출 ID/시간별 메시지를 붙였습니다.36.618초, 배속 없음.
  영상 seek/단계 이동/메시지 검색 시험 통과. 성공 영상이 아닙니다.
  tools/render_head_recording.py로 새 녹화에도 재현할 수 있습니다.
* 위 HTML3개를 사용자 브라우저에 열었습니다(xdg-open exit0).

scenario-07 및08은 낮은 ScanPose에서 손가락 한계 감시로 실패했습니다.
ScanPose 속도/가속도 scaling0.35를 추가했으며 감시 기준은 유지합니다.
관련 시험 포함 전체78개 회귀 테스트 통과(regression-current.log).
scenario-09는 손목5번 회전을-0.4rad로 줄여 손가락 한계 감시를 넘겼지만
팔 끝 관절 오차로 실행 deadline 실패했습니다. 상세 물리 replay에서
source_box_2 벽과 손의 접촉을 확인했습니다(최대fixture 침투4.026mm).
실패는 단순 하중 추종으로 보지 않습니다. 검토했던 적분 servo는 실행 코드에
넣지 않았으며 validation/hold_servo_experiment.py에 오프라인 실험만 보존했습니다.

scenario-10 실행 중: scan_right_joints=[0.75,0,0,-0.25,-0.4,0,0]rad,
손 목표 한계 여유0.02rad, ScanPose 속도0.35. 높이를 올려 상자 간섭을 피합니다.
로컬 시야 비교에서는 봉투/바코드 모두 검출했지만 물리 최종 결과를 확인해야 합니다.
컨테이너 deux-head-rgbd, 로그 scenario-10.log. 녹화 켬. result.json 저장 후에도
frames 렌더링이 이어지므로 docker top으로 실제 프로세스 종료를 확인하십시오.

실기 입력 연결용 launch
------------------------

head_rgbd.launch.py는 observation → tracking → selected ICP 및 HeadTaskNode를
동시에 구성합니다. 실제 camera/TF/robot/reader 드라이버는 별도 실행합니다.
필수 인자: scene_config, hand_mapping, observation_config, weights, model_path,
reference_frame. scene_config는 head_rgbd 정책/정지된 source/보정된 grasp·EE·scan
값을 갖춰야 합니다. 기본 hardware_enabled=false, dry_run=true입니다.
resting_on_reference_plane=true는 reference TF 원점이 실제 지지면에 보정됐을 때만
사용합니다. 외부 ZED extrinsics를 head camera로 대체하지 않습니다.
--show-args로 launch 해석 확인만 했으며 실기 시작/검증은 하지 않았습니다.
models의 학습 weight는 sim 학습이며 실제 RGB-D 영상 성능을 보장하지 않습니다.

scenario-10 결과 및 판독 방향 후보
----------------------------------

높이를 올린 scan 자세는 fixture 침투0m/물리 경고0으로 완료됐습니다.
ReacquireHeldTarget은 visual-1→visual-9를 새 시각적 손 근접 후보로 선택했고,
VerifyVisualPick의 봉투/바코드 동시 검출3프레임 조건을 통과했습니다.
AlignBarcode의 정면 법선 방향 IK가 실패했습니다. 원래 상태를 보존해
RGB-D를 다시 렌더링한 오프라인 시험에서도 roll9개 및 복수 seed로 정면 도달
해를 찾지 못했습니다. 반면 가상 리더 profile의 허용±60도 안에서 기울인
시야각 후보는 도달 가능했습니다. raw object pose는 재현 감사에만 사용하고
계획 입력은 재생 RGB-D barcode surface로 계산했습니다.

Planner의 barcode_reader_roll_candidates(기본[0])와
barcode_view_angles_deg(기본[[0,0]])를 추가했습니다. 실제 리더에 허용된
값만 설치 설정으로 제공해야 합니다. 기본 동작은 기존 정면 정렬입니다.
head sim은 roll[0,+90,-90,180]도, pitch/skew[[0,0],[0,-45],[-45,-45],[45,-45]]도.
후보는 관측된 코드 중심을 정확히 향하고 read_distance를 유지하며,
IK/관절한계/충돌 검사 후 현재 관절에서 가까운 해를 선택합니다.
정면/기울어진 시야의 중심·빔 방향·오른팔 유지 시험 포함 core13개 통과.
전체 회귀 최신78개 기록 이후 추가2개 테스트이므로 전체 재실행이 필요합니다.

오프라인 전체 계획: 1.32초, 최소 모델 clearance0.1988m.
물리 실행 증명은 아니므로 scenario-11에서 재검증 중입니다(녹화 꺼짐).
컨테이너 deux-head-rgbd, 로그 validation/head-rgbd/scenario-11.log.
scenario-10/scenario-rgbd.mp4와 dashboard-rgbd.html은 시각 확인 통과 후 IK 실패
기록입니다. reader 판독/최종 성공 영상은 아직 아닙니다.

scenario-11은 후보 전체 탐색(영상 취득31.45s→계획 종료32.266s) 중0.6초
barcode freshness 제한에 걸려 실행 전에 중단됐습니다. 감시를 완화하지 않고
설치 설정의 우선순위에서 첫 번째 IK/endpoint clearance 유효 후보를 선택하도록
바꿨습니다. 현재 sim 선호 순서: pitch/skew[-45,-45] 우선, roll-90도 우선.
최종 경로 전체 충돌·속도/가속도 검사는 유지합니다. 변경 후 core13개 통과.
이전 전체 회귀는80개 통과(regression-views.log); 최종 변경 후 전체 재실행 필요.
현재 scenario-12 실행 중(녹화 꺼짐), 로그 scenario-12.log/result.json 확인.
성공 시 같은 설정으로 최종 녹화 실행이 필요합니다.

학습/재현 명령
--------------

ROS 2 Jazzy Python3.12와 호환되는 torch를 먼저 준비한 별도 환경에서
requirements.txt 및 requirements-vision.txt의 의존성을 준비합니다.
검증 환경은 torch2.10.0+cu130, ultralytics8.3.228, lap0.5.13, MuJoCo3.13.0입니다.
ROS 기본 NumPy/SciPy와 학습 환경의 패키지를 무분별하게 덮어쓰지 않습니다.

새 출력 폴더를 사용해 sim 데이터 생성 후 segmentation 모델을 학습합니다::

  MUJOCO_GL=egl python -m deux_task_sim.train_dataset \
    --sim-repository /absolute/path/DEUX_mujoco \
    --output /absolute/path/new-dataset --counts 900 180 180 --seed 4100
  python -m deux_task_sim.train_vision \
    --dataset /absolute/path/new-dataset/segmentation/dataset.yaml \
    --output /absolute/path/new-training --model yolo11n-seg.pt --epochs 60

생성 annotation/segmentation은 오프라인 학습에만 사용합니다. online 노드의 입력은
rendered RGB/depth/intrinsics/보정 TF이며 sim 물체 pose/mask는 전달하지 않습니다.
실제 checkpoint는 models/bread_barcode_yolo11n_seg_sim.pt 및 training.json에 보존합니다.

가상 리더와 통신 격리 재검증 (2026-10-26)
-----------------------------------------

scenario-12는 AlignBarcode를 통과하고34.102초에 WaitBarcode에 진입했지만
판독 없이 실패했습니다. 녹화 재생 감사에서 기존 리더의 메타데이터 중심이
실제 mesh에 인쇄된 바코드 UV 중심과 일치하지 않는 것을 확인했습니다.
외부 sim 원본은 수정하지 않고 visual_barcode_reader.py에 인쇄 UV의
앞/뒤 표면을 보간하는 가상 광학 리더를 추가했습니다. 기존 거리/각도/
0.25초 연속 정렬 조건을 유지하고 렌더 mesh 광선으로 가림도 검사합니다.
발행값2087231250396은 인쇄 fixture의 EAN입니다. 이미지 디코딩 결과나
실제 스캐너 검증이 아니며 내부 body 변환은 센서 판정 외 제어에 전달하지 않습니다.

최신 전체 회귀82개 통과: validation/head-rgbd/regression-reader-final.log.
scenario-13은 Descend에서 stale/future observation으로 중단됐습니다.
ICP 진단에 synthetic_optical 및 시스템 시각1792946772가 나타나, 같은
ROS_DOMAIN_ID=128에서 병행한 합성 테스트 메시지 혼입이 확인됩니다.
scenario-14는 별도 ROS_DOMAIN_ID=129에서 녹화와 함께 실행합니다.
13의 결과를 가상 리더 검증 통과로 해석하지 않습니다. 최종 결과는
scenario-14/result.json 및 실제 ROS 이벤트/녹화로 확인해야 합니다.

현재 소스8개 패키지 독립 build/install 경로 빌드 통과
(build-final.log; 외부 deux_hand_msgs는 기존 underlay 제공).
head_rgbd.launch.py --show-args 및 head_task_node/serve_sim 설치 확인.
scenario-14 실행 중 캡처한27개 토픽의 이름/타입/발행/구독 관계는
구조도 원본 scenario-07과 모두 같았습니다(graph-comparison-14.json).

scenario-14: reader 성공과 BT 완료 조건 수정
------------------------------------------

14는33.626초 WaitBarcode 진입 후33.932초 raw EAN 및 동일 세션의
BarcodeRead(visual-9)를 실제 수신했습니다. 그러나 기존 완료 조건이
헤드 카메라의 barcode_visible까지 요구해서 timeout 후 실패했습니다.
같은 구간 visual_evidence는 bag_visible=True, barcode_visible=False이며
헤드 녹화에서 접근한 리더가 인쇄 바코드를 가립니다.
WaitBarcode는 신선한 선택 봉투 검출과 해당 세션 reader 응답으로 판정하도록
수정했습니다. 정렬 중 바코드 추적/표면 freshness/이동 감시는 유지합니다.
시각 확인3프레임, 요청/대상/session/시간 검사는 유지하며 pose truth는 추가하지 않습니다.
scenario-15를 ROS_DOMAIN_ID=180으로 녹화 재실행 중입니다.
14의 지표: ICP2회, 유일 pose publisher icp_node, detection reject0,
physics warning0, fixture penetration0, active weld0.
최종 성공 여부는15의 결과/영상으로 확인해야 합니다.
