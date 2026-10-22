# Fuso Rosa 연결 및 기본 제어 준비 — 2026-10-22

**단계 1 진행 중. 연결은 확인했지만 Autoware 직진·곡선·목표 정지 시험은 미통과다.**
실제 경로 제출에서 차량 외곽이 차로를 넘어서 목적지가 거부되었다.
기본 주행 성공이나 Town05 완주·신호 준수를 주장하지 않는다.

## 반영한 내용

- 기존 Prius 이미지와 설정은 보존하고 `selfcar-2026-carla:bus`를 별도로 생성.
- 실측 차체 10.2727 × 3.9442 × 4.2528m, 축간거리 5.6301m,
  최대 바퀴 조향각 약 70도. rear-axle ground 기준 위치와 비대칭 외곽을 반영.
- 버스 센서 장착 위치, 지붕 위 LiDAR, 가속·제동 표, PID 제한을 전용 프로필로 분리.
- 실행 시 실제 차량 형상과 설정이 다르면 브리지가 거부.
- 초기 계획 속도 상한 10km/h. 런타임 `velocity_smoother.max_vel=2.7777777778` 확인.
- RViz의 승용차 메시를 실측 버스 외곽 상자로 대체. 상세 버스 렌더링은 CARLA에서 확인.
- 차량 선택을 실행 인자로 전달하며, 실행 중인 차량과 다른 프로필 요청은 양방향으로 거부.

## 검사와 한계

| 항목 | 결과 | 근거 |
|---|---|---|
| 최종 이미지 빌드 | 성공 | `build-final.log`, `provenance.json` |
| 브리지 단위 검사 | 22/22 통과 | `bridge-tests-final.log` |
| 실제 ROS/CARLA 연결 | 21/21 통과 | `runtime-check.json` |
| 페달 측정 | 32 조건 측정 완료 | `pedal-calibration.json` |
| Autoware 직진 경로 제출 | 실패, 출발하지 않음 | `straight-final.json`, `route-rejection.log` |
| 곡선·목표 정지 | 미실행 | 직진과 동일한 차량/차로 폭 불일치부터 해결 필요 |
| 최종 상태 | S01, 0m/s, 버스 1대, 보행자 0명 | `final-scene.json` |

연결 검사는 GT 물체 입력, direct localization, 단일 동기 tick 소유자,
시각·위치·추적·예측 스트림 및 AEB 입력을 검사한다. 센서 기반 승객 인식 검사가 아니다.
카메라·IMU·GNSS·LiDAR가 버스에 부착된 것을 확인했다.

페달 측정은 별도의 빈 월드에서 직접 CARLA 페달을 조작한 **액추에이터 측정**이며,
Autoware 주행 검증과 구분한다. 2.78/5.56m/s 부근의 가속·감속 응답을 측정했고,
그 밖의 속도 열은 측정 경계값으로 고정했다. 정지 출발 및 저속 폐루프 성능은 미검증이다.
고속 구간 제동 응답 변화가 커서 초기 시험 속도를 제한했다. 첫 측정 실패도
`pedal-calibration-attempt1.*`에 보존했다. 보정 표의 단조 회귀 조정은 `map-fit.json` 참조.

## 주행을 막는 원인

Town05 driving waypoint 5,053개를 5m 간격으로 확인한 결과 차로 폭은 모두 3.5m였다
(`town05-lane-widths.json`). 버스 bounding box 폭은 3.944m로 더 넓다.
빈 순환로 중간 차로(road 37, lane -2)의 직선 목표도
`Goal's footprint exceeds lane!` / `The planned route is empty.`로 거부된다.
첫 보정 이미지와 최종 이미지에서 같은 결과를 재현했다.

현재 mission planner는 목적지 차로와 종방향 연결 차로의 외곽 안에 차량이 들어가는지
검사한다. 비어 있는 인접 차로가 있다고 자동으로 목적지 영역이 넓어지지는 않는다.
검사 해제, 차량 치수 축소, 지도 폭 임의 변경은 적용하지 않았다.

다음 결정은 실제 폭이 차로에 맞는 버스 에셋을 확보하거나, 현재 버스가 인접 차로까지
사용하는 것을 명시한 경로 계획을 설계하는 것이다. 후자는 도심 교차로·정류장에서도
전체 차체 여유 공간을 확인해야 하므로 기본 노선 조건에 영향을 준다.

## 재현

```bash
./scripts/carla/build_bus_image
./scripts/carla/carla_autoware start-bus
```

다른 Autoware 실행이 있다면 먼저 `carla_autoware stop`으로 종료한다.
현재 실행은 STOP/route unset 상태이며, 이 상태에서 AUTO 주행이 가능하다는 뜻이 아니다.
`tools/carla_bus_control_validation.py`는 ROS 컨테이너 안에서 실행하는 ADAPI 검증 도구다.
정지 확인 후 시험 시작 위치를 설정하고, 경로·속도 제한·AUTO 모드로 시험한다.
실패 시 STOP과 경로 해제를 확인한다. 도구의 시험 기준은 목표 오차 1.5m 미만,
정지 속도 0.05m/s 미만 3초 유지, 중심선 오차 1m 이하, 충돌 없음이며 아직 달성하지 못했다.

기존 백업: `backup/pre-bus-mission-20261022-151442`.
이번 작업 전 기준: `backup/before-bus-control-20261022` (`ab997a6`).
