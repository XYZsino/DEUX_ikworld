검출·ICP 통합 및 관절 중계 제거
==============================

2026-10-27 구현·검증. 공유 메모리/zero-copy IPC는 적용하지 않았다.

현재 실행 구조
--------------

head_rgbd.launch.py와 MuJoCo head_stack은 기존 tracked_detection_node와
icp_node 대신 ROS 이름 ``perception_node`` 하나를 실행한다.
실행 파일은 ``tracked_perception_node``이다. 기존 별도 노드와 이전
merged_perception_node 실행 옵션은 호환·회귀 검증용으로 유지했다.

* Detection worker: RGB-D 입력, YOLO/ByteTrack. 검출 결과를 처리할 때
  기존 시각 증거·바코드 표면 계산 및 발행 계약을 유지한다.
* ICP worker: 선택 대상의 같은 depth·마스크를 참조하여 ICP와 Kalman을 실행한다.
  IcpEstimator는 별도 모듈이며 ROS 노드를 생성하지 않는다.
* /task/detection_frames 발행·구독은 head 경로에서 제거했다. RGB-D 카메라 입력,
  /task/detections, /task/object_detections, visual 및 barcode 출력 계약은 유지한다.
* 두 worker는 각각 실행 중 1개 + 교체 가능한 최신 대기 1개를 가진다.
  submit 즉시 계산을 시작하고, 완료 시 GuardCondition으로 executor를 깨운다.
  기존 20ms 결과 폴링 타이머를 제거했다. RGB-D 수신 큐는 KEEP_LAST(1)/best-effort이다.
* ROS 요청 처리·TF 조회·큐 변경·결과 발행은 executor callback에서 처리한다.
  ByteTrack은 Detection worker, Kalman은 ICP worker만 변경한다.
* 프레임의 depth·CameraInfo·마스크·원본 촬영 시각을 함께 유지한다.
  다른 시점의 최신 depth를 끼워 넣지 않는다. worker 간 메시지는 읽기 전용으로 사용한다.
* 요청 변경 시 pending ICP를 버린다. 실행 중 계산은 완료하도록 두되
  요청 generation/clock epoch/lease/촬영 시각/target ID 검사로 낡은 출력을 차단한다.
  다음 generation의 첫 ICP 계산에서 필터를 초기화하므로 이전 작업 상태가 섞이지 않는다.
* ICP가 느리더라도 검출은 계속 실행된다. 단, Python/GPU/CPU 자원 경쟁이 없어지는
  것은 아니며 hard real-time 실행을 보장하지 않는다.

관절 및 상태 경로
-----------------

StateObservation에서 joints, arm_state, hand_state 배열을 제거했다.
joints_valid, joints_acquired_at, joints_reason만 전달하며 물체/EE 상태는 유지한다.
task_node의 시작·작업 중 guard는 원본 관절 측정 시각과 유효성 요약을 검사한다.
PlanMotion.observation에도 관절 배열이 포함되지 않는다.

motion_planner_node는 /arm/joint_states와 /task/hand_state를 직접 구독한다.
요청 수락 및 계산 시작 시 최신 측정을 검사하고, 시작 자세를 계산용 스냅샷으로
고정한다. 계획 완료 후 최신 측정과 비교하여 로봇 이동 여부를 재검사한다.
각 채널은 KEEP_LAST(1)/best-effort이며 중복·역순·미래·오래된 측정을 거부한다.
새로운 잘못된 관절 메시지를 받으면 이전 정상값으로 숨기지 않는다.
시계 역행 시 캐시를 비우고 새 팔·손 측정을 기다린다.

execution_node의 실제 실행은 기존 ArmAdapter/HandAdapter의 원본 피드백을
계속 사용한다. dry-run에서는 플래너와 같은 직접 측정 캐시를 사용한다.
state_observation_node는 상태 요약을 만들기 위해 관절 구독을 유지한다.
물체/EE 중계는 이번 변경에서 유지했다.
legacy JSON adapter는 측정 JointState를 명시적인 joints 인자로 받아야 한다.

검증
----

Jazzy 7패키지 빌드, 전체 227테스트 통과. 이후 강화한 동시성 테스트 부분집합 6개 통과.
합성 검출 + 실제 yaw ICP ROS 통신, 느린 ICP 중 검출 지속, 최신 대기 교체,
mode/lease/clock 변경 후 이전 결과 폐기, 직접 관절 측정 유효성 검사를 포함한다.

통신·스케줄링 비교는 같은 컨테이너에서 이전 설치와 새 설치를 각각 실행했다.
640×480 RGB-D 70프레임, 앞 5프레임 제외, 검출 3ms/ICP 6ms의 고정 합성 부하이다.
같은 프로세스에서 일반 rclpy pub/sub를 사용했으며 IPC 최적화를 활성화하지 않았다.
입력 발행 직전부터 ICP 출력 subscriber 수신까지 wall clock으로 측정했다.
실제 YOLO/ICP 계산 시간이나 실기 네트워크 성능의 비교가 아니다.

.. list-table:: 동일 합성 부하의 측정 결과
   :header-rows: 1

   * - 항목
     - 변경 전
     - 변경 후
   * - 입력 발행 → ICP 결과 수신 평균
     - 58.50ms
     - 16.66ms
   * - 입력 발행 → ICP 결과 수신 p95
     - 67.31ms
     - 17.74ms
   * - ICP 계산 완료 → 결과 수신 평균
     - 14.04ms
     - 1.61ms
   * - 검출 → ICP depth 전달 메시지
     - 약 1.23MB/프레임
     - 0 (내부 작업 전달)
   * - 물체 없는 상태 fixture 직렬화 크기
     - 1,740 bytes
     - 364 bytes

평균 지연 약 71.5% 감소. 상태 크기는 동일한 합성 관절 이름과 빈 물체 목록 기준이며,
실제 joint 이름/물체 수에 따른 메시지 크기는 달라진다. p99는 65표본의 참고값이다.

실제 MuJoCo·YOLO/ByteTrack·ICP 전체 흐름을 별도 ROS_DOMAIN_ID=212에서 실행했다.
Prepare 0.102 → Approach 6.630 → Descend 9.214 → Close 10.404 → Lift 11.900 →
ScanPose 14.790 → ReacquireHeldTarget 19.550 → WaitBarcode 22.678 →
Place 28.254 → Release 31.926 → Retreat 38.318 → Succeeded 40.630초.
관측 그래프에 perception_node만 존재하며 검출→ICP 영상 토픽 및
PlanMotion의 관절 배열이 없는 것을 확인했다.

물리 경고 0, 설비 침투 0, 강제 부착 weld 0. 물체 접촉 침투 최대 22.82mm는 남아 있다.
바코드는 SIM fixture 판독이며 실제 장치 판독/실기 검증이 아니다.
Succeeded는 판독·개방·후퇴 동작 완료이며, 사용자 요청에 따라 적재 확인은 수행하지 않는다.
단일 SIM 실행이므로 파지 성공률이나 전체 작업 속도 개선을 의미하지 않는다.

증거: validation/head-rgbd/realtime-integration/verification.json,
before-1.json, after-1.json, tests.log, concurrency-tests.log,
scenario/result.json 및 scenario/ros-events.json.
벤치마크 재현 스크립트: 같은 폴더 benchmark.py.
이전 설치 /tmp/barcode-final-install, 신규 설치 /tmp/realtime-install.

적용 시 주의
------------

StateObservation 메시지 ABI가 변경되어 생산자·소비자를 함께 재빌드하고 재시작해야 한다.
관절 토픽이 다르면 플래너에도 동일한 remap을 적용해야 한다.
ICP 필터 시간 파라미터는 통합 노드에서 icp_track_max_age_s이며,
영상 tracker의 track_max_age_s와 분리했다.
기존 8090 관제 SIM은 이번 작업에서 재시작하지 않았다. 신규 코드 검증은 별도 설치로 수행했다.
실기·원격 네트워크·장시간 운전은 미검증이다.
