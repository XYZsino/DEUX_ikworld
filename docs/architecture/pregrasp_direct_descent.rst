반닫힘 준비와 Track 없는 예측 하강 (2026-10-26)
================================================

사용자 요청
-----------

복원한 엄지·검지 파지를 유지하면서 준비 완료 시 반닫힘 자세를 만들고,
독립적인 Track 0.65초 구간 없이 예측 접근→추종 하강→닫힘을 수행한다.

구현
----

* head_calibration.yaml의 hand_pregrasp는 기존 열린/닫힌 관절값의 중간값이다.
  오른손의 third 두 관절은 기존 고정값을 유지한다. 왼손은 변경하지 않는다.
  기존 URDF 기본자세→준비 경유점은 유지하며 최종 home_joints만 반닫힘이다.
* Approach는 마지막 ICP/Kalman 포즈와 확인된 컨베이어 속도 -0.08m/s로
  도착 이후 물체가 올 지점을 계획한다. intercept_wait_s=1.0은 예측 지점의
  시간 여유이며 고정 정지 시간이 아니다. 실제 진입은 측정 EE와 시작 예정
  물체 위치의 XY 차이≤25mm, 높이 차이≤20mm 조건으로 결정한다.
* Track=0.0. TrackGrasp는 ROS 계획 이름으로 유지하지만 별도 Track 동작은 없다.
  측정된 EE 자세에서 바로 하강하면서 이동 대상에 부드럽게 합류한다.
  Descend 기본0.8초, Close 기본1.0초이며 관절 속도/가속도에 따라 늘어난다.
  하강 동안 반닫힘을 유지하고 이후 완전히 닫는다. 중간 정지는 없다.
* 최신 관측을 우선 사용한다. 관측이 없으면 원 취득 시각을 보존한 캐시를
  사용한다. 접근은 최대1초, 하강/닫힘/정착은 손-예측 물체 거리25cm 이내에서
  최대3초 예측을 허용한다. 예측을 새 ICP 관측으로 발행하지 않는다.
* 위치 편차+2σ≤25mm, 방향 불확실성, 작업 영역, 속도 일치 및 관절 제한을
  검사한다. 벨트 속도 불확실성2mm/s, 가속도 불확실성1mm/s²는 SIM 가정이다.
  손가림 마스크로 소실 원인을 확인하는 기능이나 매 프레임 재계획은 아니다.
* GUI에 물체 진입 대기를 표시한다. 실제 하드웨어나 기존 GUI 프로세스는
  연결/재시작하지 않았다. 커밋/푸시 없음.

검증
----

ROS 2 Jazzy 8패키지 빌드 및114테스트 통과.
validation/head-rgbd/pregrasp-direct/build-final.log, tests-final.log 참고.
손 닫기 단독 궤적 비교는 동일한 관절 제한에서1.74→1.24초(약29% 감소).
이는 컨베이어 작업 전체 소요시간 단축률이 아니다(closure-timing.json).

영상 기반 ROS2/MuJoCo run-05: ZED2i 렌더링→YOLO/ByteTrack→ICP/Kalman.
Prepare0.102, EstimatePose6.528, Approach8.296, TrackGrasp10.574,
Descend10.846, Close12.036, Lift13.736, ScanPose19.618,
ReacquireHeldTarget25.364, Failed33.388초. 시뮬레이션34.408초, MP4 34.476초.
계획 phase_end_times=[0,1.051883,2.375456]으로 Track0초,
하강 약1.052초/닫힘 약1.324초. BT 단계 시각에는 예약·통신·정착 시간이 포함된다.
예측 기록15회, 원 관측 나이0.612~0.680초. 이번 실행의 Close 중 지속 가림은
검증하지 않았으며 최대3초/닫힘 관측소실은 합성 ROS 시험으로 검증했다.

ReacquireHeldTarget에서 fresh visual evidence timeout으로 종료했다.
실제 파지 성공, 바코드 판독, 전체 시나리오 성공은 검증되지 않았다.
physics warnings0, 설비 침투0, 물체 접촉 최대 침투23.35mm, weld0.
물체 관통/접촉 문제는 남아 있으며 고정 파지 보조는 재도입하지 않았다.
실기 검증이 아니며 실기 속도/미끄러짐/좌표 보정은 별도 측정이 필요하다.

기록: validation/head-rgbd/pregrasp-direct/run-05/dashboard-rgbd.html
영상: 같은 디렉터리의 scenario-rgbd.mp4.
run-01은 DDS domain 범위 오류, run-02는 준비 동작 지연에 따른 인식 시간 초과,
run-03은 접근 전 손 근접 조건, run-04는 EE 변환 함수 누락으로 중단된 중간 기록이다.
위 문제를 수정한 최종 동작 증거는 run-05이다.
