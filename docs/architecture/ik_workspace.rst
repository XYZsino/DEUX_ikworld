IK와 경로 검증에 따른 접근·파지 허용
=====================================

2026-10-27 사용자 요청으로 고정 XYZ 영역 차단을 제거했다.

기존 영역의 근거
---------------

기존 acquisition_region/grasp_region은 head_calibration.yaml에 직접 지정한
SIM용 직육면체 후보 영역이다. conveyor_tracking.rst에도 모든 점의 실기 도달성을
보장하지 않는다고 기록되어 있다. URDF 관절 공간을 샘플링하거나 IK를 전수 계산하여
산출한 도달 영역이 아니며, 그 수치를 실제 로봇의 도달 한계로 해석하면 안 된다.

특히 X 상한 0.60m를 넘는다는 이유만으로 IK 호출 전에 거절하는 경로가 있었다.
이번에 기존 관측 자세를 사용하여 실제 로봇 URDF의 X=0.61m, 0.65m에 대해
접근 및 하강·닫힘 연속 경로가 계산됨을 확인했다. 이것은 두 입력의 오프라인
계획 검증이며 모든 방향/자세의 도달성이나 실제 파지 성공을 보장하지 않는다.

현재 정책
---------

head_calibration.yaml::

  conveyor_tracking:
    workspace_policy: ik

이 모드에서는 다음 고정 XYZ 검사를 하지 않는다.

* 접근 시작 물체 위치의 acquisition_region 포함 여부
* 접근 도착·닫힘 완료 예측 위치의 grasp_region 포함 여부
* TrackGrasp 시작 위치와 하강 합류 시점의 grasp_region 포함 여부
* 실행 중 현재 위치·접근 도착 예측·가림 중 불확실성 반경의 영역 포함 여부

실제 접근 목표와 하강·닫힘 경유 자세를 기존 IK로 계산한다.
IK 관절 제한, 경로의 관절 제한·속도·가속도·설비/자기 충돌 검사는 유지한다.
실행 직전 시작 자세 및 경로를 다시 검사하고, 실행 중 실제 관절 피드백과
물체 예측의 편차·최신성·가림 불확실성 한계를 감시한다.
IK 성공만으로 충돌하는 경로나 잘못된 센서 입력을 허용하지 않는다.

하강은 기존처럼 예측 파지 위치와 측정 손 위치가 가까워졌을 때 시작한다.
고정 공간 박스 조건만 생략하며 합류 거리/높이 조건은 유지한다.
실행 중 IK를 반복 계산하는 대신, 사전 검증된 궤적과 측정 편차를 감시한다.

RGB ROI 선택, 같은 프레임의 depth 3D 박스 조건, 가까운 컨베이어의 Y=264~480px 표시는 유지한다.
acquisition_region 값은 ROI의 좌우 투영 및 후보 순위에 여전히 사용되지만 이동을 차단하지 않는다.
같은 ROI 안에서 도달 가능한 경로를 만들면 파지 시도를 진행한다.

기존 설치의 호환 모드는 workspace_policy: box이며 키가 없는 설치의 기본값도 box이다.
실기에 새 정책을 적용할 때는 해당 설치 파일에 ik를 명시하고 task/planner/execution/perception을
동일한 설치 파일로 재시작해야 한다. 알 수 없는 정책 이름은 시작 시 거부한다.

검증
----

Jazzy 7패키지 빌드 및 252테스트 통과.
이전 박스 밖의 Approach/Descend/Close/TrackGrasp 계획 허용과 충돌·IK 실패 거부,
실행 중 영역 차단 제거와 예측 편차·만료·가림 공분산 한계 유지 등을 검증했다.

실제 URDF 오프라인 경로 검증:

* X=0.61m: 접근 1.50초, 하강·닫힘 2.50초. 기존 충돌 모델의 경로 검사 통과.
* X=0.65m: 접근 1.50초, 하강·닫힘 2.66초. 기존 충돌 모델의 경로 검사 통과.

증거: validation/head-rgbd/ik-workspace/build.log, tests.log,
real-model-paths.json 및 check_real_model.py.
검증용 기존 관측·시작 자세: roi-selection/scenario-02.
신규 설치: /tmp/ik-workspace-install. 실기 구동 검증은 수행하지 않았다.

실제 MuJoCo + YOLO/ICP 시나리오에서는 Prepare → Approach → TrackGrasp/Descend/Close →
Lift → ScanPose → 재인식 → 바코드 정렬·가상 판독 → 수거함 개방 → 후퇴까지
40.936초에 완료했다. 적재 확인은 수행하지 않는다.
물리 경고0, 설비 침투0, weld0, 물체 접촉 침투 최대 22.99mm는 남아 있다.
단일 SIM 실행이며 실기 파지/바코드 장치 검증이 아니다.
증거: scenario/result.json, scenario/ros-events.json, verification.json.

8090 관제는 비활성 상태 확인 후 새 설치로 재시작했고 가까운 컨베이어 ROI를 유지했다.
기존 토큰을 사용해 브라우저를 열고 실제 설치 파일의 workspace_policy=ik를 확인했다.
확인 중 사용자가 시작한 관제 작업이 Release 단계로 진행되어 해당 작업은 유지했다.
