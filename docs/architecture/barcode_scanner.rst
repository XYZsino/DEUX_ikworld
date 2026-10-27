Newland 바코드 노드
==================

구현 기준: /home/xyz-ai/Downloads/barcode/trigger_scan.py,
receive_barcode.py, dongle_port.py. 원본 및 DEUX 레포는 수정하지 않았다.
barcode_node가 시리얼 통신과 BT 세션 처리를 함께 담당한다.
기존 barcode_bridge_node 실행 파일은 제거했다.

입출력
------

* 입력 /task/scan_session: deux_task_msgs/msg/ScanSession.
  session_id, track_id, active, header.stamp, expires_at.
* 출력 /task/barcode_read: deux_task_msgs/msg/BarcodeRead.
  value, header.stamp, session_id, track_id, simulated.
* 진단 /task/barcode/diagnostics: diagnostic_msgs/msg/DiagnosticArray.
  disabled/disconnected/idle/draining/scanning/read/timeout/canceled/error 및 이유.
* REAL에서는 /barcode/raw를 구독하지 않는다. 바코드 표면 pose는 기존 ZED 인식 경로이다.
* SIM에서는 simulated=true일 때만 /barcode/raw(std_msgs/String)를 구독한다.
  가상 리더 문자열을 같은 세션/형식 검사로 연결하며 simulated=true로 발행한다.

스캐너 자체에서 받는 값은 CR로 끝나는 판독 바이트다. session_id와 track_id는
BT 세션에서 붙이며 원격 장치가 보내는 식별자라는 의미는 아니다.
header.stamp는 완전한 판독 데이터를 수신한 ROS 시각이다. 장치 촬영 시각이 아니다.
숫자 문자열의 앞자리0을 보존한다. 신뢰도·코드 종류·자세를 임의로 생성하지 않는다.

시리얼 처리
-----------

* Newland 동글 VID/PID=1eab:3306, 기본115200 baud, 시작01 54 04, 중지01 50 04.
* port가 지정되면 해당 포트만 사용한다. 빈 값이면 VID/PID로 찾고 serial_number로 좁힌다.
  0개 또는 여러 개이면 연결 실패를 진단하며 임의로 하나를 선택하지 않는다.
* enabled=false가 기본값이다. 비활성 상태에서는 포트 검색/열기/쓰기 모두 수행하지 않는다.
* Linux 시리얼 포트를 exclusive로 열며 read timeout=0, write timeout=0.05초.
  20ms steady-clock timer에서 제한된 양만 읽어 ROS 콜백을 장시간 막지 않는다.
* 연결 직후 stop/입력 버퍼 정리. 유효한 새 세션에서 다시 stop/버퍼 정리 후,
  quiet_s(기본0.2초) 동안 수신이 없으면 start를 전송한다.
  drain_timeout_s(기본1초)까지 계속 수신되면 trigger하지 않고 해당 세션을 종료한다.
* 분할 수신을 모아 CR 단위로 해석한다. CRLF의 LF를 처리하고 UTF-8 오류,
  제어 문자, ACK/NAK/OK/ERROR/FAIL 및 value_pattern 불일치는 결과로 내보내지 않는다.
* 기본 value_pattern은 숫자8자리 또는12~14자리이다. 체크디지트 검증이나 바코드 종류
  식별을 수행한다는 뜻은 아니다. 다른 상품 코드 형식은 명시적으로 변경한다.
* max_bytes(기본4096) 초과는 버퍼 오버플로 오류로 중지한다.
* 최초 유효 판독1건에서 stop 성공 후 BarcodeRead를1회 발행한다.
  그 세션은 종료하며 같은 세션 ID의 재전달로 다시 trigger하지 않는다.
* 취소/만료/ROS 시계 역행/종료 시 stop. ROS 시계가 멈춰도 REAL의 monotonic 기한을 적용한다.
  scan_timeout_s=5초 및 BT가 허용한 남은 시간 중 더 짧은 기한을 따른다.
* 연결 단절 시 세션을 폐기하고 포트를 닫는다. reconnect_s 후 연결을 재시도하지만
  이전 스캔을 재개하지 않는다. 재연결 후 새로운 session_id가 필요하다.
* 읽기 전용 진단은 상태 변경 시 및1초 주기로 발행한다. BT 결과 계약은 유지하므로
  실패 시 BarcodeRead를 만들지 않으며 BT가 기존 scan timeout/재시도 로직을 따른다.

지연 응답 한계
--------------

장치 응답에는 요청 ID가 없다. idle/취소/세션 교체의 잔여 입력과 trigger 전 조용한
구간에 도착한 데이터는 버리지만, 새 trigger 이후 도착하는 임의의 오래된 응답을
완벽하게 식별할 수는 없다. 실기에서 무선/동글 지연을 측정하고 quiet_s와
세션 간격을 조정해야 한다. 실측 없이 지연 격리가 완성됐다고 해석하지 않는다.
설정 변경·이미지 캡처 명령은 이 노드에서 보내지 않는다. 동일 포트의 기존 수신 스크립트,
EasySet 등을 함께 실행하지 않는다.

실행
----

ROS 의존성은 package.xml의 python3-serial이며 rosdep으로 설치한다.
전체 overlay를 빌드한 환경에서 단독 실행 예::

  ros2 run deux_task_perception barcode_node --ros-args \
    -p enabled:=true -p port:=/dev/serial/by-id/실제_동글_경로

실제 포트명을 먼저 확인하여 위 예시 경로를 교체한다. 여러 동글이면 serial_number를
지정하거나 포트를 명시한다. 명시하지 않으면 일치하는 동글이 정확히1개일 때만 연결한다.

설정 파일 실행::

  ros2 launch deux_task_perception barcode.launch.py config:=/absolute/path/barcode.yaml

설치 예시는 src/deux_task_perception/config/barcode.yaml이며 enabled=false이다.
BT와 함께 실행하는 head_rgbd.launch.py / task.launch.py에는 다음 인자가 추가됐다::

  barcode_enabled:=true barcode_port:=/dev/serial/by-id/실제_동글_경로

barcode_serial_number도 지원한다. 기존 barcode_topic 인자는 제거했다.
포트/통신 파라미터는 시작 시 고정되므로 변경 시 노드를 재시작한다.
SIM stack은 BarcodeNode(simulated=true)를 자동 구성하며 실제 포트를 열지 않는다.

검증
----

validation/head-rgbd/barcode-integration/에 별도 Jazzy 빌드/테스트/SIM 로그를 저장한다.
/tmp/barcode-build, /tmp/barcode-install 사용. 기존8090 SIM은 재시작하지 않았다.

* tests-final.log: 전체222테스트 통과. 스캔 프레이밍·ACK·부분 수신·형식·앞자리0,
  세션 재전달·교체·늦은 수신 drain·취소·기한·시계 역행·버퍼 초과·재연결 검사.
* test_barcode_ros_serial.py: Linux PTY + 실제 pyserial + ROS 메시지로
  trigger/read/stop/timeout/disconnect 및 REAL의 raw 토픽 미구독을 검증.
* SIM은 가상 리더가 생성한 fixture EAN을 사용한다. 물리 스캐너·실제 판독·동글 지연은 미검증.

구조도: file:///home/xyz-ai/git/DEUX_ikworld/validation/pick-scan-place-flow/dashboard.html

최종 결과와 남은 확인
-------------------

최종7패키지 빌드/222테스트 통과: build-final.log, tests-final.log.
최종 설치는 /tmp/barcode-final-install이다. 닫힌 세션이 활성 세션보다 먼저 전달될 때도
같은 ID를 다시 시작하지 않는 회귀 테스트를 포함한다.

MuJoCo scenario와 scenario-02는 각각9.622초/9.520초에 TrackGrasp 승인 과정의
conveyor forecast expired 검사에서 종료됐다. 바코드 단계에 도달하지 않았으므로
이번 코드로 전체 MuJoCo 파지·스캔·적재 흐름이 완료됐다고 주장하지 않는다.
바코드의 BT 세션 연결·판독·실패 처리는 ROS 합성 BT 및 PTY 테스트로 확인했다.
실기 포트/CR 설정, 실제 코드 형식, 동글 지연·quiet 시간, 취소 후 늦은 응답은 현장 검증 필요.
검증 요약/소스 해시: validation/head-rgbd/barcode-integration/verification.json.
