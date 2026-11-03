스캔 물체 유무 확인 ROI
======================

scan_presence_roi.json은 2026-11-02 Thor GUI에서 저장한
.setup/single-track-roi.json의 사본이다. 원본 기준 해상도는640×360이다.
이 파일은 추적 ROI와 독립적이며 이후 GUI 저장에 따라 자동 변경되지 않는다.

Lookup BT의 ScanBBoxCheck에서 도착 이후 같은 요청에 대한 최신 검출만
검사한다. 빵 bbox≥1,000px² 또는 바코드 bbox≥300px²이고 bbox 중심이
ROI 사각형 내부 또는 경계에 있어야 통과한다. 면적은 기존과 같이 실제
검출 영상의 픽셀 면적이다. CameraInfo 해상도로 ROI를 비례 변환하며,
검출과 CameraInfo의 frame_id가 다르거나 크기 정보가 없으면 통과시키지 않는다.
1초 동안 통과 검출이 없으면 기존 준비 복귀 동작을 따른다.

scan_presence_roi_file 파라미터로 경로를 변경할 수 있다. 각 확인 단계에서
ROI를 한 번 읽어 해당 단계 동안 고정하며 결과 로그에도 저장한다.
ROI 파일이 없거나 손상되면 오류로 종료하고 전체 화면 검출로 대체하지 않는다.

Thor Jazzy 환경에서 tools/test_scan_bbox_gate.py와 tools/test_scan_raw_bags.py의
8개 오프라인 테스트 통과. 실제 스캔 동작의 새 ROI 판단은 별도 확인이 필요하다.
