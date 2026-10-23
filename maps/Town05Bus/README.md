# 검증에 사용한 Town05 버스 지도

기존 Town05 지도의 사용자 속도 설정·신호·정지선을 보존하고, 원본 OpenDRIVE의
연결 관계와 점선/차선 변경 허용을 확인한 경계 16개를 추가 복원한 지도다.
경계 위치나 도로 폭은 바꾸지 않았다. 기존 93개 공유 경계에 더해, 버스 지도에서는
road 25의 +1/+2 사이 경계 6522를 추가 복원했다. 양쪽 차선 전체를 각각 맞추면
연결부 오차로 제외되던 경계로, 공유 선 자체를 70개 지점에서 대조했다.
실제 점선 및 양방향 차선 변경 허용을 확인했으며 최대 위치 차이는 약 4.1cm다.
근거는 `carla_bus_lane_seam_report.json`에 기록한다.
도심 속도 태그는 30km/h, 서부 순환로 road 36/37은 50km/h로 설정한다.
버스 노선 시험에서 비교차로 구간의 바깥 도로 경계·반대 방향 구분선 212개를
추가 분류했다. 같은 방향의 내부 경계는 유지하며, 교차로 전체 외곽까지
분류하는 도구는 아니다. 근거는 `carla_bus_road_edge_report.json`에 기록했다.
직선에서 불필요한 경로 편향을 줄이기 위해, 비교차로 경계 174곳(도로 가장자리 87곳, 도색 중앙선 87곳)에 실제
평평한 포장면으로 확인한 여유 폭을 메타데이터로 기록한다. 차선 형상과 통행
권한은 유지하며, 인도·물리 중앙분리대·커브에는 이 여유 폭을 적용하지 않는다.
도색 중앙선의 최대 0.5m 여유는 차체 공간에만 사용하며 반대 차로 경로는 허용하지 않는다.
측정 원본은 `config/carla/bus/town05_straight_margins.json`, 설치 기록은
`carla_bus_straight_margin_report.json`이다.
`manifest.json`에 압축 전 파일과 외부 pointcloud의 SHA256을 기록했다.

기존 Town05 설치가 있다면 다음 명령으로 별도의 Town05Bus 지도를 생성한다.
원본 Town05는 변경하지 않으며, 재실행 시 기존 버스 지도 편집도 보존한다.

```bash
./scripts/carla/carla_autoware stop
./scripts/carla/prepare_bus_map
```

이 폴더는 실제 시험한 Lanelet2/신호 설정의 고정 사본이다. 동일한 사본을 복원하려면
아래 명령을 **새로운 빈 Town05Bus 폴더에서만** 실행한다. 기존 폴더를 교체할 때는
먼저 백업한다. CARLA 원본 에셋, 329MiB pointcloud, ML 모델은 외부 설치 자산이다.

```bash
mkdir /home/a/autoware_data/maps/Town05Bus
cp maps/Town05Bus/*.json maps/Town05Bus/map_projector_info.yaml /home/a/autoware_data/maps/Town05Bus/
gzip -dc maps/Town05Bus/lanelet2_map.osm.gz > /home/a/autoware_data/maps/Town05Bus/lanelet2_map.osm
gzip -dc maps/Town05Bus/carla_lane_change_report.json.gz > /home/a/autoware_data/maps/Town05Bus/carla_lane_change_report.json
cp /home/a/autoware_data/maps/Town05/pointcloud_map.pcd /home/a/autoware_data/maps/Town05Bus/
```

`carla_lane_change_report.json.gz`는 각 경계의 허용/제외 근거를 기록한다.
교차로·서로 다른 방향·실선·불연속 도로·기존 사용자의 금지 태그는 허용으로 바꾸지 않는다.
