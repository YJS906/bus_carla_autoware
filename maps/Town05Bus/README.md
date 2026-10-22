# 검증에 사용한 Town05 버스 지도

기존 Town05 지도의 사용자 속도 설정·신호·정지선을 보존하고, 원본 OpenDRIVE의
연결 관계와 점선/차선 변경 허용을 확인한 경계 16개를 추가 복원한 지도다.
경계 위치나 도로 폭은 바꾸지 않았다. 총 93개 공유 경계가 차선 변경을 허용한다.
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
