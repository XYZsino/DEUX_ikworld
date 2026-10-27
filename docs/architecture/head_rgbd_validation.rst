헤드 RGB-D PickScan 구현 및 검증
==============================

현재 동작
---------

헤드 RGB-D 영상에서 sim 학습 YOLO11n-seg가 빵 봉투와 바코드를 검출하고,
ByteTrack이 영상 ID를 연결합니다. 3개 취득 프레임에서 관측한 봉투 중 작업
대상을 선택한 뒤 해당 mask/depth만 ICP에 전달합니다. 정지한 소스에서 얻은
원 취득 시각의 pose를 최대15초 이내 파지에 사용합니다. 움직이는 컨베이어를
전제로 한 추적 파지 구현은 아닙니다.

파지 후에는 ICP를 요청하지 않습니다. 오른팔을 편 ScanPose로 이동한 뒤,
가림으로 ID가 만료되면 손 근처의 유일하고 상승한 시각 후보를3프레임 확인해
재선택합니다. 봉투와 바코드 확인 후 관측된 바코드 표면으로 왼팔 리더를
정렬합니다. 정렬 실행 중에도 대상/요청/바코드 ID, 취득 시각 및 이동을 검사합니다.
판독 대기에서는 신선한 선택 봉투 검출과 해당 ScanSession의 reader 응답을
확인합니다. 스캐너가 코드 영역을 가릴 수 있어 이 단계는 헤드 카메라의
barcode_visible을 동시에 요구하지 않습니다. 스캔 재시도는 최대3회입니다.

학습
----

모델: models/bread_barcode_yolo11n_seg_sim.pt.
900 train /180 validation /180 test의 별도 seed 장면에서60 epoch 학습했습니다.
held-out box mAP50=0.9495, mask mAP50=0.8997.
SHA256: bcf96491081ac0c6e3b9d764f4e29a35c6d0c0d75653e70e994cda8ae09a03ab.
상세: 동일 이름의 .training.json. ByteTrack은 검출 결과를 연결하며 별도 학습하지
않습니다. 실카메라 정확도나 다른 봉투 종류의 성능을 검증한 수치는 아닙니다.

실제 ROS/MuJoCo 검증
-------------------

validation/head-rgbd/scenario-15/result.json 및 control-audit.json 기준:

* 0.102초 Prepare,5.746초 TrackAndSelect,6.052초 EstimatePose.
* 6.664초 Approach →9.792초 Descend →15.232초 Close →17.986초 Lift.
* 24.888초 ScanPose →30.838초 재선택 →31.110초 봉투/바코드 확인.
* 31.722초 AlignBarcode →34.034초 WaitBarcode →34.340초 Succeeded.
* /task/object_detections publisher는 icp_node만 존재. ICP2회, 마지막 수신6.562초.
* 선택 visual-1은 가림 후 visual-8로 재선택. 영구 신원 보존을 주장하지 않음.
* 실제 DDS /barcode/raw → /task/barcode_read 수신 및 대상/세션 일치 확인.
* 물리 경고0, 설비 침투0, active weld0. 봉투 접촉 침투 최대 약24.1mm.
* 최신8개 overlay 패키지 빌드 및82개 테스트 통과. 외부 deux_hand_msgs는 underlay 제공.

학습 정답 pose/mask는 오프라인 annotation에만 사용합니다. online 영상 센서는
RGB/depth/intrinsics/카메라 TF만 전달합니다. 평가 로그의 정답 물체 위치는
제어 입력이 아닙니다. 가상 광학 reader 내부의 body 위치는 광학 조건 판정에만
사용하고 제어에 pose를 발행하지 않습니다.

가상 리더는 mesh의 실제 인쇄 UV 중심, 거리/각도/0.25초 연속 정렬 및 광선
가림 조건을 검사합니다. 반환값2087231250396은 인쇄 fixture의 EAN이며
이미지 디코딩 결과가 아닙니다. 실기에서는 실제 reader 드라이버로 교체합니다.
이 성공은 집기 후 스캔 흐름 한 건의 검증이며 배치/이송 완료, 장시간 파지력,
실물 충돌 안전성 또는 실제 로봇 검증을 뜻하지 않습니다.

실행과 실기 전환
----------------

ROS 2 Jazzy overlay와 sim/vision 의존성을 준비한 뒤::

  ros2 run deux_task_sim serve_sim \
    --sim-repository /absolute/path/DEUX_mujoco \
    --output /absolute/path/interactive-output \
    --weights /absolute/path/models/bread_barcode_yolo11n_seg_sim.pt

자동 작업을 시작하지 않고 /task/run을 기다립니다. GUI는 같은 ROS 도메인에서
/task/status와 /task/run에 연결합니다. 현재 이미 실행 중인 기존 GUI preview는
legacy-truth 경로이며 이 문서의 새 실행으로 자동 교체되지는 않습니다.

실기 overlay: deux_task_bringup/head_rgbd.launch.py. 기본 hardware_enabled=false,
dry_run=true. RGB-D 드라이버, 실제 보정 TF, 팔/손 피드백·명령 드라이버 및
barcode/raw 리더 드라이버를 제공해야 합니다. head 장착값을 외부 ZED의 보정값으로
재사용하면 안 됩니다. 실기 모델 재학습/평가, 정지 소스 조건, 손 매핑/그립력,
충돌 모델/정렬 허용각 검증 항목은 hardware_rollout.rst를 참고하십시오.

증거와 산출물
-------------

* scenario-15/ros-events.json: 실제 DDS 수신과 Action 경계 기록.
* scenario-15/recorded-qpos.npz 및 video-frames.json: 실제 시뮬레이션 상태/시각.
* scenario-15/control-audit.json: 성공·pose 입력 경로·ICP 중단·세션/봉투 검출 대조.
* regression-final-15.log, build-final-15.log: 최종 회귀와 빌드 결과.
* docs/architecture/head_rgbd_nodes.html:27개 토픽의 실제 연결에 근거한 노드 구조도.
* docs/architecture/head_rgbd_bt.html: Archify BT 흐름,9/9 검사 및 브라우저/시각 확인.

scenario-15/scenario-rgbd.mp4:1440×720 H.264,35.394초,347개 실제 상태 프레임.
scenario-15/dashboard-rgbd.html: 로봇/검출 영상과 시각별 실제 메시지.
브라우저 재생, 단계 이동, 판독 메시지 필터, 페이지 오류0 및 화면 확인 완료.
세 HTML 화면을 로컬 브라우저로 열었습니다.
실패했던 과거 시도는 head_rgbd_progress.rst에 이력으로 보존합니다.
