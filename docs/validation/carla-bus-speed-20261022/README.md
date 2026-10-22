# 버스 20km/h 운행 상한과 정차 유지 검증

2026-10-22. 사용자 확인에 따라 현재 버스 운행 상한을 **20km/h**로 확정했다.
50km/h 상한을 잠시 준비했으나 실제 시험은 20km/h만 수행했고,
30/40/50km/h 증속 시험은 진행하지 않았다.

## 최종 구성의 주행 결과

빈 Town05_Opt, Fuso Rosa 한 대, GT 객체 입력, direct localization, 동기 tick 0.05초.
시작 위치를 STOP 상태에서 설정한 뒤 주행·감속·정지는 모두 Autoware가 수행한다.

| 시험 | 이동 거리 | 실제 최고 속도 | 최대 중심선 거리 | 최종 목표 오차 | 정지 유지 | 최종 모드 | 충돌 |
|---|---:|---:|---:|---:|---:|---|---:|
| 직진 | 59.26m | 19.54km/h | 0.157m | 0.796m | 5.10초 | STOP | 0 |
| 곡선 포함 순환로 | 477.94m | 20.30km/h | 0.219m | 0.705m | 5.05초 | STOP | 0 |

긴 시험은 road 37/lane -2의 s=20→490으로, 약 90도 곡선과 직선이 이어진다.
도로 기준 s 차이는 470m이며 실제 주행 차로의 이동 거리는 약 478m다.
19km/h 이상을 약 68.15초 연속 유지했다. 중심선 거리는 rear axle 위치와 native
중심선의 3차원 거리로 높이 차이도 포함한다. 실제 최고 속도가 설정 상한보다
약 0.30km/h 높았으므로 절대적으로 20.00km/h를 넘지 않는 제어라고 주장하지 않는다.

최종 원본: `stop-fixed-straight/straight.json`, `final20-speed/speed.json`.
`summary.json`에 결과·이미지 ID·소스/지도 해시를 기록했다.
목표 정지 및 종료 시 STOP/시험 제한 해제/경로 해제를 확인했다.

## 변경 내용

- 버스 전용 `planning_common.max_vel=5.5555555556m/s` 및 프로필의
  `operating_speed_cap_kmh=20`. 검증 명령의 기본값도 20km/h다.
- 곡률에 따른 감속, 조향 속도 제한, 신호·장애물 정지 기능은 유지했다.
  일반 최대 가속 0.7m/s²와 lateral acceleration limit 0.455m/s²도 유지한다.
- 긴 `speed` 시험 추가: 상한의 95% 이상을 연속 5초 유지해야 통과한다.
  현재 시험 도구는 20km/h 초과를 거부한다.
- 목표 정지 유지 조건을 3초에서 5초로 늘리고 종료 모드도 기록한다.
- 버스 전용 vehicle command gate 설정을 설치한다. PID의 목표 정지 명령은 -3.4m/s²였지만,
  STOP 모드의 별도 유지 명령 -1.5m/s²가 전환 필터의 ±1.0m/s² 제한으로
  -1.0까지 줄어들면서 페달 변환기가 작은 가속 명령을 내는 현상을 확인했다. 정지 유지/완만한 정지 명령을 -3.4로 맞추고,
  전환 필터의 절대 가속도 한도를 3.5로 조정해 해당 제동 명령을 보존했다.
  일반 가속 계획의 0.7m/s² 상한을 높인 변경은 아니다.
- 기존 Prius 설정과 Town05 지도는 변경하지 않았다.

실제 적용 값은 `velocity-smoother-final20.yaml`, `vehicle-cmd-gate-final.yaml`에 있다.
빌드에서 실제 가감속 변환기의 시작 검사를 통과했고, 최종 두 주행은 STOP 전환 뒤에도
5초간 정지 조건을 유지했다.

## 이전 시도와 재현

- `20kmh/speed.json.gz`: 초기 긴 구간 20km/h 시험 통과. 당시 전역 상한은 50이었지만
  시험의 외부 제한은 20이었으며 실제 최고 속도는 약 20.70km/h였다.
  이 기록은 50km/h 주행 검증이 아니다.
- `final20-straight/straight.json.gz`: 전역 상한을 20으로 고정한 뒤 STOP 전환 시
  미세 이동을 검출해 실패. 최종 설정은 이 문제를 수정한 버전이다.
- 압축 전 JSON 해시는 `compressed-records.json`에 기록했다.
- 10km/h 기준은 Git 태그 `backup/bus-10kmh-20261022`, 로컬 이미지
  `selfcar-2026-carla:bus-10kmh-20261022`로 보존했다.

```bash
./scripts/carla/carla_autoware stop
./scripts/carla/build_bus_image
./scripts/carla/carla_autoware start-bus
./scripts/carla/validate_bus_control straight
./scripts/carla/validate_bus_control speed
```

전체 노선 완주, 교차로·배경 차량·보행자 대응, 10개 정류장 조건부 정차와 하차벨 UI는
이 시험의 범위가 아니다. [미션 진행 단계](../../carla-bus-mission.md)와
[실행 안내](../../carla-bus-operations.md)를 따른다.
