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
