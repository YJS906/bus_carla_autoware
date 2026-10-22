# 버스 실행·검증·백업 안내

## 현재 PC

CARLA 0.9.16, Docker/NVIDIA 실행 환경, 기존 Autoware 이미지, Town05 지도와 ML 모델이
설치된 `/home/a/carla_pp` 환경을 기준으로 한다. 두 터미널에서 실행한다.

```bash
# 터미널 1: 서버가 실행 중이면 중복 실행하지 않는다.
./scripts/carla/carla_run -quality-level=Low
```

```bash
# 터미널 2: 다른 차량 실행을 종료하고 버스 실행
./scripts/carla/carla_autoware stop
./scripts/carla/build_bus_image
./scripts/carla/carla_autoware start-bus
```

`start-bus`는 Fuso Rosa 한 대, GT 물체 입력, direct localization, 20km/h 계획 상한,
신호 브리지와 버스 추적 카메라로 시작한다. 자동으로 Engage하지 않는다.
`prepare_bus_map`을 호출해 원본 Town05를 보존한 별도 Town05Bus 지도를 사용한다.
첫 생성/지도 업그레이드는 Autoware를 정지한 상태에서 한다.
기존 Prius는 `start-town05`와 기존 이미지/지도를 계속 사용한다.

정류장과 노선을 CARLA에 표시하려면:

```bash
/home/a/CARLA/venv-0.9.16/bin/python tools/carla_bus_route_preview.py --duration 3600
```

기존 정지 버스 확인 UI는 [노선 안내](../scenarios/bus/README.md)를 따른다.
하차벨 기능이 구현된 승객용 UI와는 다르다.

## 기본 AUTO 주행 시험

아래 명령은 실제 버스를 움직인다. Town05에 버스 한 대만 있고 보행자/배경 차량이
없는 상태를 요구한다. STOP 확인 후 시작 위치를 옮기고, ADAPI 경로 제출과 AUTO로
주행한다. CARLA autopilot이나 직접 페달 명령으로 시험 주행을 대체하지 않는다.

```bash
./scripts/carla/validate_bus_control straight
./scripts/carla/validate_bus_control curve
./scripts/carla/validate_bus_control speed # 기본 20km/h, 곡선 포함 약 478m
# 결과 폴더를 직접 정할 수도 있다.
./scripts/carla/validate_bus_control straight /tmp/bus-straight 20
```

직진은 road 37/lane -2의 s=330→390, 곡선은 s=130→300, 긴 속도 시험은
s=20→490을 사용한다. 기본 속도는 20km/h이고 세 번째 인자로 더 낮은 상한을
지정할 수 있다. `speed` 시험은 상한의 95% 이상을 연속 5초 유지해야 통과한다.
현 단계 검증 도구는 20km/h 초과 입력을 거부한다.
목표 오차 1.5m 미만, 속도 0.05m/s 미만 5초 유지, 최고 속도 2m/s 초과,
중심선 거리 1m 이하와 충돌 없음이 통과 조건이다. 중심선 거리는 rear axle의
3차원 위치 차이이므로 도로 높이 오차도 포함한다. 결과 JSON/로그는 기본적으로
`log/bus-control-날짜시각/`에 저장하며, 성공/실패 후 STOP과 경로 해제를 확인한다.

## 정류장 노선 한 바퀴 시험

```bash
./scripts/carla/run_bus_lap
# 선택: 실패 구간 진단. 전체 완주 결과로 처리하지 않는다.
./scripts/carla/run_bus_lap /tmp/bus-suffix --start-anchor S04
```

S01에 최초 배치한 뒤 S01→S10, S10→S01 두 경로로 운행한다. 같은 시작 lanelet을
한 경로에 중복할 수 없는 제약 때문에 S10에서 정지·경로 전환 후 재출발한다.
정류장과 순환로 경유점의 순서, 충돌 없음, 입력 최신성, 목표 오차 1.5m 미만과
정지 유지 5초를 확인한다. 무진행 120초·충돌·입력 단절 시 중단하고 STOP 처리한다.
서비스 정차나 모든 정류장의 바깥 차로 정렬을 검증하는 도구는 아니다.
속도 상한은 20km/h지만 실제 속도는 곡선·신호·계획 결과에 따라 낮아진다.

## 새 PC와 외부 자산

이 저장소에는 소스, 버스 설정, 노선/정류장, 검증한 압축 Lanelet2 지도·신호 설정,
성공 및 실패 기록을 포함한다. CARLA/Unreal 원본 에셋, Docker 이미지, 대형 PCD,
ML 모델/TensorRT 캐시와 로컬 전체 백업은 Git에 넣지 않는다.
이 파일들은 기존 외부 설치/백업에서 준비해야 한다.

기존 실행 이미지가 없는 PC는 [기반 빌드 안내](build-and-run.md)와
[CARLA 운영 안내](carla-operations.md)에 따라 VTD 기반 오버레이와 CARLA 이미지를
먼저 만든다. 현재 빌드 체인은 VTD 개발 헤더를 포함한 외부 자산에도 의존한다.
`build_bus_image`의 기본 기반은 보존된 `selfcar-2026-carla:lane-start-fix-20260923`다.
새로 기반 이미지를 빌드한 경우 다음처럼 명시한다.

```bash
./scripts/build-vtd-image.sh
./scripts/carla/build_carla_image
CARLA_BASE_IMAGE=selfcar-2026-carla:local ./scripts/carla/build_bus_image
```

정확한 시험 지도 복원은 [Town05Bus 지도 안내](../maps/Town05Bus/README.md)를 따른다.
경로 기본값은 `/home/a/CARLA/0.9.16`, `/home/a/CARLA/venv-0.9.16/bin/python`,
`/home/a/autoware_data/maps`, `/home/a/autoware_data/ml_models`다.
필요하면 `CARLA_ROOT`, `CARLA_PYTHON`, `AUTOWARE_MAP_DIR`,
`AUTOWARE_ML_MODELS_DIR`를 해당 스크립트에 맞게 설정한다.

노선 완주 수치와 남은 문제는 [전체 노선 검증](validation/carla-bus-lap-20261022/README.md),
별도 20km/h 순항 기준은 [속도 시험](validation/carla-bus-speed-20261022/README.md)을 따른다.

## 보존 기준

- 미션 전환 전: `backup/pre-bus-mission-20261022-151442` (`7d65001`).
- 버스 제어 전: `backup/before-bus-control-20261022` (`ab997a6`).
- 로컬 전체 백업: `/home/a/autoware-task-backups/pre-bus-mission-20261022-151442/`.
  `RESTORE.md`와 `manifest.json`에 복원 절차와 파일 해시가 있다.
- 10km/h 버스 기준: `backup/bus-10kmh-20261022`, 로컬 이미지
  `selfcar-2026-carla:bus-10kmh-20261022`.
- 원본 Town05 지도와 Prius 이미지 태그는 이번 버스 수정으로 덮어쓰지 않았다.

빈 도로의 전체 노선 경유 완주는 확인했다. 다음 단계는 일시적 계획/제어 진단과
속도 개선, 정류장별 바깥 차로 접근·정차·재출발, 하차벨·승객 판단 통합이다.
전체 노선 경유 결과를 다른 차량/보행자 대응이나 서비스 정차 검증으로 보지 않는다.
