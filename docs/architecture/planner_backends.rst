선택 가능한 IK 백엔드와 비교 결과
================================

2026-10-27. 설치 YAML의 ik_backend를 scipy / pyroki / curobo 중 하나로 지정한다.
배포 YAML과 코드의 기본값은 사용자 요청으로 pyroki로 변경했다.
ik_backend를 생략한 실기 설치도 PyRoKi를 사용하며, 설정을 읽을 때 유효 값을 명시해
장면 버전에 포함한다. SciPy/cuRobo는 해당 값을 명시했을 때 선택한다.
경량 합성 테스트 fixture는 scipy를 명시한다.
잘못된 이름·누락된 선택 라이브러리·CUDA 사용 불가·허용 오차 초과는
오류로 중단한다. 다른 계산기로 자동 대체하지 않는다.

후속 검증: 기본 PyRoKi로 MuJoCo 파지→스캔→적재 동작→후퇴41.548초 완료.
동일 시점7패키지 빌드/266테스트 통과. 아래 비교는 이전 오프라인 측정이며,
전체 흐름 검증의 조건과 한계는 commit_validation.rst에 기록했다. 실제 로봇 미검증.

교체 범위
---------

이번 구현은 IK 엔진 선택이다. 컨베이어 시간 예측, 후보 생성/점수, 접근 보간,
하강·닫힘 궤적, 속도/가속도 retiming, Lift 및 전체 로봇 충돌 검사는 공통이다.
cuRobo의 MotionPlanner/TrajectoryOptimizer나 PyRoKi의 전체 궤적 최적화는 사용하지 않는다.
따라서 이 결과를 GPU 배치 모션 플래너 전체의 성능 비교로 해석하면 안 된다.

motion_planner_node만 수정하는 것으로는 충분하지 않다.

* deux_task_core: IK 공통 검증, URDF 팔 체인 내보내기, PyRoKi/cuRobo 어댑터 추가.
* motion_planner_node: 작업 프로세스에서 초기 준비 및 모델/컴파일 캐시 재사용.
  설정/URDF 버전이 달라지면 캐시를 무효화한다. 초기 준비는 작업 접수 전에 끝낸다.
* MotionPlan: ik_backend 기록. 실행기는 설치 설정과 다른 백엔드의 계획을 거부한다.
* SIM CLI/설치 설정/bringup: 백엔드와 플래너용 Python을 선택한다.
* BT: 실행 순서는 동일하며 상태 사유에 선택한 IK 엔진을 표시한다.

ROS 토픽/Action 연결 구조와 팔·손 명령 인터페이스는 그대로 사용한다.
메시지 ABI가 바뀌었으므로 관련 생산자/소비자를 함께 재빌드해야 한다.

동일하게 유지하는 검사
---------------------

PyRoKi/cuRobo는 base_link에서 각 grasp 프레임까지의 7관절 체인을 사용한다.
원본 URDF를 수정하지 않고 임시 파일에 내보낸다. 설치에 고정값으로 명시한
lift 등은 그 값으로 변환에 반영한다. 손가락과 반대 팔은 IK 명령에서 변경하지 않는다.
관절은 이름으로 재배열하고 반환된 해를 원래 28관절 모델의 FK로 다시 검사한다.
위치 오차2mm·회전 오차0.02rad·관절 제한을 넘으면 성공 플래그와 관계없이 거절한다.

팔 체인 IK 내부의 충돌 검사는 비활성화되어 있다. 대신 기존 전체 로봇 모델로
관절 샘플마다 자기/설비 충돌을 검사하고, Lift 등 loaded 단계에는 파지 물체 외곽도 넣는다.
이 검사는 손가락·반대 팔도 포함하며 실행기가 다시 수행한다. IK 성공만으로 실행하지 않는다.
기존 충돌 구와 설비 AABB의 모델 정확도·샘플 검사 한계는 남는다.

세 엔진의 내부 비용/seed 전략은 완전히 같지 않다.
SciPy/PyRoKi는 seed 연속성 정규화를 사용하고, cuRobo는8개 GPU seed 해 중
현재 관절에 가장 가까운 해를 고른다. cuRoboV2 CUDA graph는 활성화한다.
PyRoKi는 기본 JAX CPU이며 CUDA JAX를 설치한 환경은 JAX_PLATFORMS로 선택할 수 있다.

선택과 실행
-----------

실기용 scene_config 또는 SIM 설치 파일::

  ik_backend: pyroki  # scipy | pyroki | curobo

라이브 SIM / 녹화 검증 CLI에는 --ik-backend를 추가했다.
지정하지 않으면 head_calibration.yaml 값을 사용한다::

  python -m deux_task_sim.serve ... --ik-backend pyroki
  python -m deux_task_sim.validate_head ... --ik-backend curobo

세 가지 설정은 동일한 ROS 계약을 사용한다. 작업 도중 즉석 전환은 하지 않는다.
설정을 바꾸고 플래너 및 동일 설치 파일을 사용하는 노드를 다시 시작한다.
이번 사용자 요청으로 실행 중이던 SIM은 종료했고 다시 시작하지 않았다.

head_rgbd.launch.py와 task.launch.py의 planner_python은 플래너 프로세스만 별도 Python으로 시작한다::

  ros2 launch deux_task_bringup head_rgbd.launch.py \
    scene_config:=/absolute/path/installation.yaml \
    planner_python:=/absolute/path/planner-venv/bin/python \
    ...

나머지 필수 인자(hand_mapping, observation_config, weights, model_path, reference_frame)는
기존 설치 값을 전달한다. hardware_enabled/dry_run의 기본값은 변경하지 않았다.
Jazzy 바이너리와 같은 Python3.12 환경이 필요하다. 로컬 cuRobo .venv는3.11,
기존 pyroki conda 환경은3.10이므로 그 인터프리터를 Jazzy 노드에 바로 사용하면 안 된다.
venv 활성화만으로 ros2 run의 설치된 shebang이 바뀌지 않으므로 planner_python을 사용한다.

선택 의존성
-----------

일반 ROS 패키지 빌드는 CUDA/JAX를 필수 의존성으로 추가하지 않았다.
PyRoKi를 사용하는 Python3.12 venv에서::

  python -m pip install -r src/deux_task_core/requirements-pyroki.txt
  python -m pip install --no-deps \
    'pyroki @ git+https://github.com/chungmin99/pyroki.git@388e43e1fc0d0ee382968d3dd72970fd62a0450c'

cuRobo는 로컬 /home/xyz-ai/git/curobo의 V2 API를 사용한다.
검증한 core 소스 commit은 78fd485fa82d9b9a063fb4985e371814587e666a이다.
이 저장소를 수정하지 않았다. CUDA13/ARM64 검증 환경은 Torch2.10.0+cu130,
Warp1.18.0, cuda-core와 cuRobo 소스를 설치한 Python3.12 환경이다.
환경에 맞는 Torch/CUDA를 설치한 뒤 cuRobo가 제공하는 설치 extra를 사용할 수 있다::

  python -m pip install '/home/xyz-ai/git/curobo[cu13-torch]'

선택 환경의 기존 NumPy/OpenCV/ROS 바이너리 호환성을 확인해야 한다.
이번 disposable Docker 환경은 /opt/deux-planner-venv/bin/python,
cuRobo 소스 사본은 /opt/deux-curobo-source이고 실제 소스 저장소는 그대로이다.
Jazzy overlay는 /tmp/backends-install이다. 위 /opt, /tmp 경로는 해당 컨테이너 내부 경로다.

비교 방법과 결과
---------------

SIM·실제 로봇을 실행하지 않았다. 실제 DEUX URDF와 이전 녹화의 관절/ICP 입력으로
각 방법을 순서대로 실행했다. 같은 시작 관절, 목표 포즈, 설치 충돌 형상, 허용 오차를 사용했다.

* 양팔 근처 목표24개 + 이전 작업 경로 목표12개 = 도달 가능한 목표36개, 각각3회.
* 도달 불가 목표4개, 각각3회. 총 IK 호출120개/백엔드.
* 물체 X=0.5168m / 0.61m / 0.65m에서5후보 전체 경로 검사, 각각2회.
* 독립 ROS_DOMAIN_ID214의 PlanMotion Lift 요청. 실행 노드·팔 명령·MuJoCo 없음.
* 기본 패키지 빌드7개 및264개 테스트. 세 방법 모두 ROS 계획 응답 통과.

============================= ============= ============= =============
지표                          SciPy         PyRoKi CPU    cuRobo GPU
============================= ============= ============= =============
도달 가능한 IK 통과           108/108       108/108       108/108
도달 불가 IK 거부             12/12         12/12         12/12
IK 중앙값                     5.33ms        1.87ms        8.74ms
IK p95                        7.02ms        3.23ms        9.98ms
위치 오차 p95 (FK 수치)        0.584mm       0.587mm       0.000161mm
후보 전체 검사 중앙값          2.475s        1.304s        2.705s
후보 전체 검사 p95             3.683s        1.398s        2.837s
전체 경로 통과                4/6           4/6           4/6
프로세스 초기 준비             0.008s        5.09s         4.02s
============================= ============= ============= =============

후보 검사 시간은 실패 시도도 포함한다. X=0.65m의 이번 예측 시각에서는 세 방법 모두
TrackGrasp 중 허용 오차를 넘어서 거부했다. 고정 X 상한으로 거부한 것이 아니다.
이전 X=0.65m 통과 시험은 시작 시각/기다림/예측 위치가 달랐다.

초기 준비의 cuRobo 수치는 이미 생성된 디스크 커널 캐시를 사용한다.
별도 최초 연결 시험에서는 CUDA/Warp 컴파일 포함21.62초였다.
위 수치는 단일 머신의 순차 IK 방식이며 실시간 스케줄링 보장이나 실제 로봇 정확도가 아니다.
특히 cuRobo의 작은 FK 오차는 센서/설치/기구 오차를 포함하지 않는다.

이번 입력에서는 PyRoKi가 빠르고 같은 통과율을 보였다. 비교 당시에는 기존 SciPy가 기본값이었고
사용자의 SIM 종료 요청에 따라 전체 SIM 동작은 실행하지 않았다. 이후 사용자 요청으로
배포 YAML 기본값을 PyRoKi로 변경하고 관제 SIM을 다시 실행했다. 전체 시나리오 검증과는 구분한다.
cuRobo는 다수 후보/경로의 GPU 배치화,
native 경로 최적화까지 바꿀 때 별도 비교가 필요하다.

증거 및 보고서
--------------

validation/head-rgbd/planner-backends/dashboard.html: 비교표, 지연 차트, 위치별
통과 결과, 실패 사유, 실행 선택 방법, 원본 JSON 링크.
benchmark-*.json / ros-*.json / build.log / tests.log에 원시 결과를 저장했다.
benchmark.py 및 check_ros.py가 재현 스크립트다.

공식 API 참고:

* https://github.com/chungmin99/pyroki
* https://nvlabs.github.io/curobo/latest/api/curobo.inverse_kinematics.html
