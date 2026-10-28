미래 시점별 배치 IK 파지 탐색
============================

구현 및 설정
------------

공통 motion_planner_node에서 이동 물체의 필터링된 포즈/속도/공분산과
실측 관절을 사용한다. SIM과 real에 같은 알고리즘을 사용하며 실기는 미검증이다.
실기 installation.yaml은 미완성 템플릿이므로 실측 설치값과 moving_source,
conveyor_tracking 설정을 갖춘 뒤 활성화해야 한다. 실기 기본 IK는 PyRoKi다.

설정은 conveyor_tracking.candidate_search 아래에 둔다::

  enabled: true
  time_search:
    enabled: true
    arrival_min_s: 1.0
    arrival_max_s: 5.0
    step_s: 0.25
    handoff_s: 0.5
    prefetch_grasp: true
    reuse_validated_path: true
    full_path_candidates: 5
    max_prediction_std_m: 0.04
    planning_budget:
      initial_search_wall_s: 2.5
      initial_revalidation_wall_s: 0.05
      safety_factor: 1.25
      dispatch_lead_s: 0.25
      max_research_attempts: 2

위 블록은 전체 설치 설정이 아니다. 파지 자세 candidates 및 관절/충돌/추종 설정은
src/deux_task_sim/deux_task_sim/head_calibration.yaml을 참고하되 실기에는 보정해야 한다.
time_search.enabled=false이면 기존 자세 후보 검색을 사용한다.
candidate_search.enabled=false이면 기본 파지 자세로 계획한다.
둘 모두 이동 예측 자체를 끄는 옵션은 아니다. 변경 후 관련 노드를 재시작한다.
GUI 토글은 없으며 GUI에는 계산 결과와 거절 사유를 표시한다.

계산과 실행
-----------

* 예약된 접근 시작 기준 1~5초, 0.25초 간격 17시각 × 11자세 = 187후보.
  관측 지연과 계산 예산은 별도로 반영하므로 현재 시각부터 정확히 1~5초는 아니다.
* 접근/닫힘/Lift 대표 포즈의 IK를 배치 계산한다. PyRoKi는 JAX jit/vmap,
  cuRobo는 goal별 seed를 가진 batch solver, SciPy는 직렬 호환 경로를 쓴다.
* 각 결과에 FK 오차/관절 제한, 충돌 여유, 접근 속도·가속도, 예측 불확실성을 검사한다.
  상위 최대 5개 전체 경로는 직렬 검사한다. 탐색 완전성과 항상 더 좋은 해는 보장하지 않는다.
* 관절 이동량과 충돌 여유로 선택하고 접근 도착/파지 시작/닫힘 완료 시각을 보존한다.
  계산 실측 시간과 전달 여유를 포함해 출발을 예약하며 예산 초과 시 제한적으로 재탐색한다.
* Approach 경로는 실행 전 시작 관절, 시각, 설정, 새 관측과 예측 차이를 검사한다.
  작은 차이는 기존 경로를 재사용하고 큰 변화는 재탐색한다. 새 관측이 필요하다.
* 접근 중 TrackGrasp 사전 계산을 수행할 수 있다. 원래 획득 시각/예측을 보존하며
  실행 전 관절/설정/시간/관측 변화가 허용 범위를 벗어나면 거절한다.
* 접근과 하강은 분리되어 있다. TrackGrasp 하강 중 손가락 닫힘을 일부 겹치고,
  실제 손가락 궤적의 속도·가속도 제한으로 소요 시간을 산출한다.
* 재사용은 Approach에 적용된다. TrackGrasp 검증과 Lift 계획은 별도다.

PlanMotion과 MotionPlan의 grasp_start_at, grasp_phase_durations로 절대 파지 시작과
Track/Descend/Close 시간을 전달한다. 실행기는 해당 시간 계약을 검사한다.
메시지 형식이 바뀌므로 메시지 및 모든 소비 패키지를 재빌드하고 노드를 함께 재시작한다.
관절 속도·가속도·충돌 제한 완화와 SIM 전용 가림 검사 완화는 이번 기능에 포함하지 않는다.

관제 및 검증 범위
-----------------

/task/grasp_candidates JSON을 GUI 시간×자세 표로 표시한다.
전체 통과/대표 검사만 통과/거절을 구분하며 셀별 사유를 확인할 수 있다.
최신 재검증 결과를 받아도 원래 후보 격자를 보존한다.

테스트는 후보 탐색, 시간 계약, 지연 시 재탐색, 사전 계산, 경로 재사용,
ON/OFF 분기와 GUI 결과 보존을 포함한다. 합성 테스트 통과는 실제 파지 성공을 뜻하지 않는다.
실기에는 카메라 외부 보정, 충돌 모델, 실제 팔·손 동작 및 명령 전달 시간을 검증해야 한다.

2026-10-28 선별 코드 검증: ROS 2 Jazzy 8패키지 빌드 및 307테스트 통과.
실제 PyRoKi/cuRobo 백엔드로 각각 3개 FK 생성 목표의 배치 IK 성공과 비작업 관절 보존을 확인했다.
이는 임의 목표의 도달성·실기 성공률 검증이 아니다. GUI 후보 표/거절 사유도 Chromium에서 확인했다.
실행 로그: validation/time-search-commit-20261028/ (커밋 제외).
