누적 기능 커밋 후 검증 (2026-10-27)
=================================

검증 코드: ``b0605cd``. 원격 main의 홈 자세와 팔 점검 도구 커밋을 보존했다.
실행·인식 통합, 이동 파지 후보, 준비/스캔/적재, IK 선택, 바코드, 관제,
카메라/SIM 및 테스트·모델·문서가 대상이다. tools/validation과 구조도 검수 산출물은 제외했다.

* ROS 2 Jazzy 7개 패키지를 새 /tmp/commit-verify-install에 빌드했다.
* 독립 ROS_DOMAIN_ID216에서 core/runtime/perception/sim/gui 테스트 266개 통과(27.71초).
* 별도 ROS_DOMAIN_ID217에서 기본 PyRoKi + 커밋한 SIM 분할 모델로 실제 MuJoCo
  RGB-D→검출/추적→ICP→파지→Lift→스캔→적재 동작→후퇴까지 41.548초에 완료했다.
  yaw_plus_8 후보 선택 후 최신 관측으로 재검증했다. 물리 경고0, 설비 침투0, weld0.
  봉투 접촉 침투 최대23.73mm는 남는다. 적재 성공 확인은 사용자 설정에 따라 생략했다.
  바코드 판독은 SIM 광학 조건/fixture 값이며 실제 스캐너 검증이 아니다.
* 기존 관제의 MuJoCo/ZED 영상, 현재 단계/흐름 배치, 준비자세 버튼, 모바일 표시 확인.
  실행 중인 관제를 재시작하거나 실제 로봇에 명령하지 않았다.
* 실물 모델 deux_1026_finetuned_from1021.pt는 로딩·빈640×480 영상 CPU 추론 통과.
  detect 모델의 b1/m1/s1/s1_1/bar 계약은 SIM 분할 모델과 다르다. 파일 포함만 수행했고
  실기 head RGB-D 경로로 자동 전환하지 않았다. 실영상 정확도/하드웨어 미검증.

로컬 상세 증거: validation/head-rgbd/commit-verification/의 build.log, tests.log,
scenario.log, verification.json, browser-check.json, pyroki-scenario/result.json.
이 증거 파일과 녹화 산출물은 Git에 포함하지 않는다.
