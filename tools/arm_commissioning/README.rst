팔 소동작 점검 도구
===================

태스크 실행과 별개로 ROS 2 Jazzy의 팔 관절 명령 경로를 점검하는 도구입니다.
ROS 패키지 등록이나 colcon 빌드 없이 이 폴더의 스크립트를 직접 실행합니다.
기본은 preview이며 실제 Action 전송에는 ``--execute-reviewed-plan``이 필요합니다.
손 명령, 컨트롤러 재시작, 게인 변경, e-stop 해제, 자동 재시도/복귀는 없습니다.

범위와 의존성
-------------

* 팔 14관절 중 하나를 최대 ±2°, 3~10초 동안 이동합니다. 기본값은 오른손목
  ``arm_r_joint7`` +1°/3초입니다. 나머지 관절의 목표는 시작 시 실측값입니다.
* ROS 환경에 rclpy, action_msgs, builtin_interfaces, control_msgs,
  controller_manager_msgs, rcl_interfaces, sensor_msgs, std_msgs,
  trajectory_msgs와 NumPy가 필요합니다. 테스트에는 pytest가 추가로 필요합니다.
* 이 폴더에 런타임 의존성 설치나 기존 환경 변경 스크립트는 없습니다.
* 이전 실험의 5°/10° 예외, 추종 검사 해제, 기본자세/Approach/복귀 스크립트,
  실측 기록, CAN 로그, 메시 및 HTML은 포함하지 않습니다.

검사 기준
---------

이 값들은 점검 도구의 기준이며 제조사 사양이 아닙니다.
``arm_step_core.py``의 ``TOLERANCES``와 결과 JSON의 ``tolerances``에 표시합니다.

============================== ========================
항목                           기본 기준
============================== ========================
시작 시 기존 목표↔실측 차이     0.5° 이내 (모든 팔 관절)
이동 중 도구의 추종 오차        0.4° 초과 시 취소 요청
드라이버의 이동 중 허용오차     0.5°
종료 시 목표 도달 허용오차      0.25° (모든 팔 관절)
목표 도달 추가 대기 시간        2초
preview 유효기간               60초
preview 이후 시작 자세 변화    0.002rad 이내
============================== ========================

이동 중 오차와 종료 시 도달 오차는 별개입니다. 계획 시간 이후 추가 2초 안에
목표에 도달하지 못하면 드라이버가 GOAL_TOLERANCE_VIOLATED를 반환할 수 있습니다.
각도 예외나 허용오차 완화를 이 도구가 자동 적용하지 않습니다.

실행 전에는 상태 원본/수신 시각, 정지 상태, URDF 위치 한계, 컨트롤러 활성화,
다른 활성 Action, 경쟁 궤적 발행자 및 관절/컨트롤러 상태 발행자 수를 검사합니다.
상태 발행자가 두 개 이상이면 이름이 같더라도 차단합니다. 다른 PC와 같은 ROS
도메인을 쓰면 피드백이 혼입될 수 있으므로 로봇별 통신 격리가 필요합니다.
이 검사는 분산 제어권 잠금이나 네트워크 격리를 대신하지 않습니다.

사용
----

레포 최상위에서 ROS 환경을 불러온 뒤 preview를 만듭니다::

  source /opt/ros/jazzy/setup.bash
  python3 tools/arm_commissioning/arm_step_check.py --with-gravity \
    --joint arm_r_joint7 --delta-deg 1 --duration 3 \
    --output /tmp/arm-preview.json

``blockers``가 비어 있고 ``stationary``가 true인지 확인합니다. preview는
차단 사유를 기록하면서도 정상 종료할 수 있으므로 종료 코드만 보지 마세요.
현장 실행 승인과 주변 확인 후, 60초 이내의 동일 계획을 명시적으로 실행합니다::

  python3 tools/arm_commissioning/arm_step_check.py --with-gravity \
    --execute-reviewed-plan /tmp/arm-preview.json --output /tmp/arm-result.json

실행 시 preview의 계획 해시, URDF, 시작 자세, 오차 설정을 다시 확인합니다.
명령 권한이 필요한 환경에서는 이 도구 실행 자체가 승인을 대신하지 않습니다.

``--with-gravity``는 기존 deux_gravity의 ``_rival_publishers`` 동작에 맞춘
협조 방식입니다. 직접 메시지를 보내지 않는 trajectory publisher endpoint를
만들어 점검 중 일반 위치 동기화를 억제합니다. preview에서도 이 endpoint가
존재하므로 ROS graph에 영향을 줍니다. 토크·게인은 유지하며, e-stop의 강제
동기화는 차단하지 않습니다. gravity 이외의 궤적 발행자나 실제 직접 궤적
메시지가 관측되면 중단합니다. 일반 setpoint/gravity 상태여야 합니다.

결과와 중단 처리
----------------

* ``goal_sent``는 요청 전송 여부입니다. 성공/정지 확인과 같은 의미가 아닙니다.
* ``driver_status``와 ``terminal_status``는 Action 상태입니다.
  4=SUCCEEDED, 5=CANCELED, 6=ABORTED입니다.
* ``primary_error``는 최초 실패 원인을 보존합니다. 취소/결과 대기 오류는
  ``cleanup_error``에 따로 남겨 원래 추종 실패를 덮어쓰지 않습니다.
* 이미 종료된 결과를 받았으면 다시 취소하지 않고 기존 result future를
  재사용합니다. 취소 응답만으로 완료로 간주하지 않습니다.
* ``cleanup.cancel_requested``는 실제 취소 요청 여부이며,
  ``canceled_after_failure``는 최종 상태가 CANCELED인 경우에만 true입니다.
* ``stop_confirmed``는 최신 실측값으로 정지 여부를 확인한 별도 결과입니다.
  정지와 Action 종료는 다른 사실이며, 정지 확인 실패는 ``stop_error``에 남습니다.

드라이버의 abort와 취소 요청이 동시에 발생하는 경합을 완전히 제거하는 것은
아닙니다. CANCELING 고착이나 통신 상실 시 자동으로 컨트롤러를 재시작하지 않습니다.
하드웨어 watchdog과 현장 정지 수단을 대신하지 않습니다. 전체 로봇 충돌,
부하/마찰/토크, EE 경로, 손 자세의 안전성을 검증하는 도구도 아닙니다.

테스트
------

수학/판정 테스트는 ROS 연결 없이 실행합니다::

  python3 -m pytest tools/arm_commissioning/tests/test_arm_step_core.py -q

ROS 통합 테스트는 가짜 드라이버를 생성하므로 반드시 격리 도메인과 localhost
설정을 함께 사용합니다. 해당 조건이 없으면 테스트가 실행을 거부합니다::

  ROS_DOMAIN_ID=198 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    python3 -m pytest tools/arm_commissioning/tests -q

성공, SIGINT 취소 후 terminal 확인, 드라이버 자체 abort 시 중복 취소 방지,
실험 각도 예외 거부를 검사합니다. 이 결과는 실기 추종 성능 인증이 아닙니다.
