YOLO 검출 모델
==============

파일: deux_1021_A_v2.0only_adamw.pt

SHA256: 814f48b2ea1e1f018985d88add5ef709a4ffaaf4e60028753a9bf376a9cdb46d

클래스: 0=b1, 1=m1, 2=s1, 3=s1_1. 기본 ICP 대상은 s1/s1_1입니다.
저장 이미지 CPU 추론에 사용한 체크포인트를 포함합니다.

설치별 perception.yaml의 detection_node.ros__parameters.weights에 이 파일의
절대 경로를 지정하십시오. 현재 workspace에서는 다음 경로입니다::

  /home/xyz-ai/git/DEUX_ikworld/models/deux_1021_A_v2.0only_adamw.pt

colcon 설치 디렉터리에 자동 복사하지 않으므로 workspace를 옮기면 설정 경로도
갱신해야 합니다.

빵 봉투 ICP 3D 모델 (v5)
========================

파일: breadbag_model_v5/mesh/opaque_envelope.obj

SHA256: b8dcee2925eff9d0235e8f5139ff1b7317e81b048e3910d7cb3b104cffaa1dce

저장 RGB-D 회귀 검증에 사용한 DEUX_mujoco/assets/breadbag_model_v5의 OBJ를
원본 그대로 포함합니다. OBJ가 참조하는 breadbag.mtl과 breadbag_albedo.png도
같은 디렉터리에 포함합니다. MuJoCo 시뮬레이션 패키지 전체가 아닌 ICP용 mesh와
표시용 재질/텍스처입니다.

단위는 미터이며 +X는 오른쪽, -Y는 스티커/앞면, +Z는 봉투 입구 방향입니다.
mesh 원점은 바닥 footprint 중심이며 바닥 가장자리 중간 표면이 z=0입니다.
기존 회귀 결과와 같은 원점을 사용하려면 설치별 perception.yaml에 다음을 지정합니다::

  icp_node:
    ros__parameters:
      model_path: /home/xyz-ai/git/DEUX_ikworld/models/breadbag_model_v5/mesh/opaque_envelope.obj
      model_origin: mesh

reference_frame과 보정 TF는 별도로 설정해야 합니다. bounds_center 원점으로 바꾸는
경우 downstream 기하도 동일하게 맞춰야 합니다. 이 모델은 최신 sim의 v6가 아니라
기존 ICP 검증에 사용한 v5입니다.

Head RGB-D simulation detector
-----------------------------

``bread_barcode_yolo11n_sim.pt``: YOLO11n, classes0=bread_bag/1=barcode.
Ultralytics8.3.228,60epochs, synthetic900train/180validation/180held-out scenes.
Held-out mAP50=.9521, mAP50-95=.7839, precision=.9598, recall=.9133.
These are rendered-scene metrics, not hardware performance claims.
ByteTrack consumes this detector's output; no separate tracker training is claimed.
Provenance: bread_barcode_yolo11n_sim.training.json.
Dataset generator: deux_task_sim.train_dataset. Trainer: deux_task_sim.train_vision.
Barcode labels follow the asset's visible UV barcode region with occlusion.


Sim RGB-D instance segmentation 모델
------------------------------------

``bread_barcode_yolo11n_seg_sim.pt``: YOLO11n-seg, sim RGB-D head 영상의
bread_bag/barcode 두 클래스.60epoch 학습, 독립 sim test box mAP50=0.9495,
mask mAP50=0.8997. provenance는 같은 이름의 ``.training.json``.
SHA256: bcf96491081ac0c6e3b9d764f4e29a35c6d0c0d75653e70e994cda8ae09a03ab.
실영상/하드웨어 성능 검증 아님. ByteTrack 연결에 별도 tracker 학습은 없습니다.


실물 검출 모델 (2026-10-26 fine-tune)
------------------------------------

``deux_1026_finetuned_from1021.pt``: 사용자가 제공한 실물 바코드 포함 검출 모델.
클래스는 0=b1, 1=m1, 2=s1, 3=s1_1, 4=bar이며 Ultralytics task는 ``detect``이다.
SHA256: 395b0f20e93eca32b20ff20c1d06f9ec2479c59b5360e43a8991596736d32eec.
파일 정보와 확인 범위는 같은 이름의 ``.metadata.json``에 기록했다.

기존 detection_node 사용 시 perception.yaml의 weights에 이 파일의 절대 경로를
지정한다. 실제 클래스 이름을 사용하며 봉투 ICP 대상은 s1/s1_1, 바코드 클래스는 bar이다.
현재 head_rgbd.launch.py의 bread_bag/barcode 클래스 및 instance mask 필수 설정과는
계약이 다르므로 해당 경로의 SIM 분할 모델을 이 파일로 단순 교체하면 안 된다.
이 커밋은 모델 파일 포함이며 실기 통합 설정 전환은 하지 않는다.

모델 로딩 및 빈 640×480 이미지 CPU 추론만 확인했다. 학습 지표는 제공되지 않았으며,
실영상 검출 정확도와 실제 로봇 동작은 검증하지 않았다.
