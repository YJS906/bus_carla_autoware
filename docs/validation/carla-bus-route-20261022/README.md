# 버스 노선·정류장 배치 확인

- `route-validation.json`: 지도 기반 정류장 검사와 참조 노선 생성 결과.
- `tests.log`: 정류장 제한 조건·순환 연결·역방향 샘플·입력 해시 회귀 검사 6개.
- `live-preview.json`: 실제 Town05_Opt에서 임시 정류장 표시 10개 실행 결과.
- `bus-geometry.json`: 실제 Town05_Opt에 생성한 Fuso Rosa의 형상·바퀴·물리 값.
- `stop01-bus-preview.png`: S01에서 버스를 정지 상태로 확인한 CARLA 카메라 원본.
- `capture_station.py`: 정지 차량·카메라 확인에 사용한 절차. 실행 중 Town05와
  비어 있는 S01을 전제로 버스·센서를 생성하고 종료 시 자신이 만든 객체를 정리한다.

관측용 버스에는 handbrake와 brake를 적용했고 Autoware는 실행하지 않았다.
이 기록은 버스 자율주행·제동·정차 정확도·충돌 회피의 검증 결과가 아니다.
촬영 후 임시 버스와 카메라를 삭제했다. 임시 표시에는 유효 시간이 있으며
정류장의 영구 정의는 `scenarios/bus/town05_bus_route.json`이다.

기존 Lanelet2 지도와 실행 이미지·차량 설정은 수정하지 않았다.
기준점이 도로 폴리곤 안에 있다는 검사는 차체 전체가 차로 안에 들어간다는 뜻이 아니다.
보고된 버스 bounding box 폭 3.944m와 차로 폭 3.5m의 차이는 후속 계획·제어에서
반영하고 실제 회전·접근을 검증해야 한다.
