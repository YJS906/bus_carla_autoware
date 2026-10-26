# CARLA + Autoware 수요 기반 자율주행 버스

CARLA 0.9.16 **Town05 / Fuso Rosa** 버스로 도심 → 순환로 → 도심 노선을 주행하고,
하차벨과 대기 승객에 따라 정류장 정차 여부를 결정하는 것을 목표로 한다.
기존 Prius/VTD 작업과 검증 이력도 함께 보존한다.

## 현재 상태 — 문서 정리 2026-10-26 / 최신 주행 검증 2026-10-24

**주행 기반과 교통 차량 16대 조건의 노선 완주를 검증했고, 수요 기반 정류장 서비스는
후속 작업이다.** [미션 단계](docs/carla-bus-mission.md#구현-및-검증-순서)는
0·1 완료, 2 부분 완료, 3~6 미완료다. 아래 초기 시험은 당시 조건의 이력이며,
최신 교통 시험의 판정은 `following-validation.json`의 `final_traffic_lap`을 따른다.

- 기존 작업 전체 백업, 도심 정류장 **10개**와 고정 노선, 정지 버스 확인 UI 완료.
- S01~S10 모두 오른쪽 인도에 지붕·벤치가 있는 실제 정류장 시설을 배치했다.
  `start-bus` 실행 시 복원하며, 재실행해도 중복 생성하지 않는다.
- 버스 형상·물리 충돌체 측정, 전용 차량/센서/가감속 설정 완료.
- 차량 폭으로 인한 경로 거부, 순환로 지도 태그 누락, 제동표 로딩 오류,
  목표 정지 후 기어가는 현상을 수정했다.
- **현재 속도 상한은 도심 30km/h / 서부 순환로 50km/h.** 가감속 표를 다시 측정하고
  PID 출력을 조정했다. 빈 순환로 약 478m 시험에서 실제 최고 48.48km/h,
  47.5km/h 이상 연속 7.15초와 목표 정지 5.05초를 확인했다. 충돌 0건.
  이전 10/20km/h 시험 기록도 보존한다.
- **이전 20km/h 설정의 정류장 10개 경유 완주: 1,989.83m, 충돌 0건.**
  S10 정지·재출발과 S01 목표 정지를 확인했다. 상한 20km/h, 실제 최고 18.65km/h,
  대기 포함 평균 5.06km/h다. 일시적 안전 진단 7회는 자동 회복했으며 개선 대상으로 남는다.
- **직선 경계 v2를 적용한 새 설정의 무정차 코스 완주: 2,006.91m, 충돌 0건.**
  S01→S10→S01 복귀·목표 정지를 확인했다. 기록된 정지선 통과 14건은 모두 녹색이다.
  실제 최고 속도는 도심 26.40km/h, 순환로 46.41km/h였다.
  [빈 도로 전체 주행 결과](docs/validation/carla-bus-lap-20261023/blue-boundaries-full-lap.json)
- 순환로는 첫 직선에서 바깥 3차로로 이동하고, 커브를 지난 출구 직선에서
  2차로 → 1차로로 복귀하도록 변경했다. 바깥 차로 순항 최고 48.41km/h와
  출구 복귀→S06 구간 483.78m, 충돌 0건을 확인했다. 차로 변경 상한은 20km/h다.
  [구간별 성공·실패 기록](docs/validation/carla-bus-lap-20261023/ring-section-results.json)
- 직선은 원래 중앙 경로를 유지하도록 보완했다. 같은 30km/h 상한의 직선 비교 구간에서
  바깥 차선 계획 편차 중앙값은 70.1→4.5cm, 실제 편차는 69.0→1.6cm,
  안쪽 차선 실제 편차는 73.3→6.3cm다. 버스·차선 크기와 커브의 차체 여유는 유지한다.
  도심·직선·커브 구간 시험과 v2 경계의 전체 코스를 충돌 없이 통과했다.
  짧은 커브 전후 감속과 일시적 경로 변화 안전 진단은 개선 대상으로 남는다.
  [중앙 주행 비교 결과](docs/validation/carla-bus-lap-20261023/centering-results.json)
- **최종 교통 시험: 차량 16대를 끝까지 유지하며 2,005.33m 완주.**
  S01→S10→S01 복귀·정지, 버스와 NPC 양쪽 충돌 센서 0건, 차량 소실·비정상 제어값 0건.
  정지선 통과는 녹색 13회·황색 1회였고 적색/미상 통과는 검출되지 않았다.
  실제 최고 속도는 도심 26.84km/h·순환로 46.49km/h다.
  경계 여유 폭을 1.0→1.4m로 늘리고 교차로 연결점의 폭 할당을 수정했다.
  일반 차선 변경 최소 길이 30m와 횡가속도 기반 속도 제한을 분리했다.
  현재 차로의 분리된 뒤차를 구분하며 목표 차로 뒤차 검사는 유지한다.
  GT 객체·신호 입력과 NPC 추종 보완을 적용한 한 바퀴 시험이며,
  일시적 긴급 상태 2회는 자동 회복했다. 이동 중 최신 경계 표본 649개 중
  순간 경계 밖 차체 면적이 0.1m²를 넘은 표본 5개는 개선 대상으로 남는다.
  [최종 결과와 실패 이력](docs/validation/carla-bus-lap-20261024/following-validation.json)
- **조건부 정류장 정차·항상 바깥 차로 접근·하차벨 UI·보행자 대응은 아직 미검증이다.**
  기존 승용차 시험을 버스 검증 결과로 간주하지 않는다.

[최종 교통 시험](docs/validation/carla-bus-lap-20261024/following-validation.json) ·
[실행 방법](docs/carla-bus-operations.md) ·
[미션 설계 및 진행 단계](docs/carla-bus-mission.md) ·
[노선·정류장](scenarios/bus/README.md) ·
[검증 지도 사본](maps/Town05Bus/README.md)

초기 검증 이력:
[20km/h 노선 완주](docs/validation/carla-bus-lap-20261022/README.md) ·
[20km/h 순환로 시험](docs/validation/carla-bus-speed-20261022/README.md) ·
[10km/h 형상·경로·정지 문제 수정](docs/validation/carla-bus-envelope-20261022/README.md)

현재 PC에서 버스를 실행하려면:

```bash
./scripts/carla/carla_autoware stop
./scripts/carla/build_bus_image
./scripts/carla/carla_autoware start-bus
# 빈 도로에서만 실행: 시작 위치를 옮긴 뒤 실제 AUTO 주행/정지 시험
./scripts/carla/validate_bus_control straight
./scripts/carla/validate_bus_control curve
./scripts/carla/validate_bus_control speed # 곡선 포함 긴 순환로 시험, 기본 20km/h
./scripts/carla/run_bus_lap # 정류장 10개 경유 코스, 서비스 정차 없이 한 바퀴
```

CARLA 서버가 먼저 실행되어 있어야 한다. 시작 명령은 버스를 STOP 상태로 생성한다.
버스는 `Town05Bus` 지도와 `selfcar-2026-carla:bus` 이미지를 사용하며,
기존 Town05 지도와 Prius 실행 이미지는 보존한다.
외부 설치 자산·새 PC 준비·백업 범위는 [버스 실행 안내](docs/carla-bus-operations.md)에 정리했다.

<details>
<summary>기반 프로젝트 Autoware 안내</summary>

<p align="center">
    <img src="https://user-images.githubusercontent.com/63835446/158918717-58d6deaf-93fb-47f9-891d-e242b02cba7b.png"
        alt="Autoware_RViz" width="75%" />
</p>

<!--- License -->
<p align="center">
    <a href="https://github.com/autowarefoundation/autoware/blob/main/LICENSE">
        <img src="https://img.shields.io/github/license/autowarefoundation/autoware?style=flat&label=License"
            alt="License" /></a>
</p>

<!--- Social Media -->
<p align="center">
    <a href="https://discord.gg/Q94UsPvReQ">
        <img src="https://img.shields.io/discord/953808765935816715?logo=discord&logoColor=white&style=flat&label=Autoware%20Discord"
            alt="Autoware Discord"></a>
    <a href="https://twitter.com/intent/follow?screen_name=AutowareFdn">
        <img src="https://img.shields.io/twitter/follow/AutowareFdn?logo=x&logoColor=white&style=flat"
            alt="Autoware Twitter / X"></a>
    <a href="https://www.linkedin.com/company/the-autoware-foundation/">
        <img src="https://img.shields.io/badge/Linkedin-Autoware%20Foundation-0a66c2?logo=linkedin&logoColor=white&style=flat"
            alt="Autoware Linkedin"></a>
</p>

Autoware is the world's leading open-source autonomous driving framework. Autoware provides a comprehensive, production-ready software stack designed to accelerate the commercial deployment of autonomous vehicles across diverse platforms and use cases.

## 🚀 Get Started

<p align="center">
    <a href="https://autowarefoundation.github.io/autoware-documentation/main/installation">
        <img src="https://img.shields.io/badge/📥_Installation-Get_Autoware_Running-2ea44f?style=for-the-badge"
            alt="Installation" /></a>
    &nbsp;&nbsp;
    <a href="https://autowarefoundation.github.io/autoware-documentation/main/demos/">
        <img src="https://img.shields.io/badge/⚡_Quick_Start-Try_the_Demo-1f6feb?style=for-the-badge"
            alt="Quick Start" /></a>
</p>

> **New to Autoware?**
>
> 1. **[Install Autoware](https://autowarefoundation.github.io/autoware-documentation/main/installation)** → Set up your environment and build the stack from source.
> 2. **[Run the Quick Start demo](https://autowarefoundation.github.io/autoware-documentation/main/demos/)** → Drive a simulated vehicle in just a few minutes.

</details>

## Selfcar 2026 VTD runtime

**2026-09-12 최신 백업(브릿지·지도 포함), 구버전 PC 업데이트:**
[Codex 인수인계 및 설치 가이드](docs/update-existing-pc-20260912.md).
고정 태그: `backup-20260912-intersection-exit`.

This branch preserves the local VTD Autoware changes as complete ROS 2 source
packages under `vtd_overlay/src/` and the runtime configuration under
`config/vtd/`. The custom overlay is built into an immutable image; host-built
`.so` files and host source/configuration mounts are not used. The base image
is fixed to
`ghcr.io/autowarefoundation/autoware@sha256:86d12e0f2504b058b54faead3a6113b5777c778899fc7a7c362e787808fdac24`.

The tracked custom overlay packages are `autoware_mission_planner_universe`,
the behavior-path planner, planner-common, lane-change and static-obstacle
modules, `autoware_behavior_velocity_traffic_light_module`, and the local
`vtd_ros2_bridge`. They are based on `autoware_universe` 0.52.0 at
`6e477c645efec33f7909095eea684474e97f5e3d`; the launch configuration baseline
is `autoware_launch` 0.52.0 at
`f942598d44b5769353167c76b784323d5c14c8c7`.

On a new Ubuntu 24.04 + ROS 2 Jazzy/Docker host, obtain the external map,
model, and VTD installation assets, then run:

```bash
git clone https://github.com/DCAM1/selfcar_2026_.git
cd selfcar_2026_
git checkout backup-20260912-intersection-exit
cp .env.example .env
# Edit .env with the external asset paths.
./scripts/verify-reproducibility.sh
./scripts/build-vtd-image.sh
./autoware routes/route_example.csv
```

See [docs/build-and-run.md](docs/build-and-run.md),
[docs/reproducibility-audit.md](docs/reproducibility-audit.md), and
[`vtd_overlay/origin.yaml`](vtd_overlay/origin.yaml) for the exact upstream
commits, external asset manifest, package provenance, and optional bridge
runtime.

The optional CSV argument map-matches a complete route after Autoware is ready:
nearby lanelets are selected using direction and forward topology, every point
is projected to the chosen centerline, sequence 1 is checked against the VTD
ego start, intermediate sequences become checkpoints, and the final sequence
becomes an in-lane goal. Use `./set_route FILE.csv` for RViz preview and to set
or safely change the route of an already running instance. Manual RViz lanelet
overrides can be saved as `corrected_route.csv`; see
[docs/build-and-run.md](docs/build-and-run.md#csv-routes).

Edit a package only under `vtd_overlay/src/`, then rerun
`./scripts/build-vtd-image.sh`; use `./scripts/verify-reproducibility.sh` to
check that source/configuration inputs remain tracked. The former direct host
`.so` mounts and host source/configuration mounts have been removed. Pull
requests run the static checks; the full image build is a manual workflow for a
self-hosted NVIDIA runner.

## CARLA 0.9.16 integration

**2026-10-22 버스 미션 전환:** 기존 결과를 백업하고 `mission/town05-bus`에서
[도심 정류장 10개와 서부 순환 노선](scenarios/bus/README.md)을 생성했다.
차로 중앙 정차, 하차벨·대기 승객 조건, 구현 단계는
[버스 미션 설계](docs/carla-bus-mission.md)를 따른다.
버스의 직진·곡선·목표 정지, 빈 도로와 후행 보완을 적용한 교통 차량 16대의 전체 노선 경유 완주를 검증했다.
조건부 정차·하차벨 UI는 후속 단계이며, 상세 조건과 남은 문제는 위의 검증 보고서를 따른다.

The current city setup is Town05. The latest [behavior repair and AUTO validation report](docs/carla-behavior-repair-20260922.md)
records three parked-car avoidance runs, three pedestrian-stop runs, and two
regression runs with no collisions. These results use the separate CARLA
`ground_truth` object mode; they do not demonstrate improved LiDAR detection.
Temporary stops and stronger-than-commanded braking remain documented limitations.
See the [object input mode guide](docs/carla-ground-truth.md),
[lane-change map repair](docs/carla-town05-lane-changes.md), and
[raw validation evidence](docs/validation/carla-behavior-repair-20260922/README.md).

For dense background vehicles and roaming pedestrians without restarting
Autoware, see [Town05 city traffic settings](docs/carla-city-traffic.md).
For a desktop map editor with vehicle/pedestrian placement, save/load, and
traffic count controls, see [Town05 scenario GUI](docs/carla-scenario-gui.md).
The [crosswalk stop-line update](docs/carla-stop-lines.md) places matched existing
stop lines 3 m before crosswalk centers. The [current route-block diagnosis](docs/carla-route-block-20260923.md)
identifies a lane-start projection error on long Town05 lanelets. The
[fix and VTD comparison](docs/carla-lane-start-fix-20260923.md) records the repaired
candidates, live avoidance/return, and a right-turn continuation at half simulation speed;
normal-speed planning-rate warnings remain a validation limitation.

See the earlier [Town05 repair and validation record](docs/carla-repair-20260922.md)
for the Prius calibration, CARLA planning preset, native traffic signals/crosswalks,
map preparation and runtime commands. Custom CARLA bridge changes are maintained
in `carla_overlay/src/` and rebuilt with `./scripts/carla/build_carla_image`.

This repository also contains the CARLA runtime layer developed from the final
VTD-tested image. The earlier Town01 setup starts bridge-managed NPC
vehicles, adds continuously roaming pedestrians, and follows the ego vehicle
with the CARLA spectator.

Build the runtime image after building `selfcar-2026-vtd:local`:

```bash
docker build \
  --build-arg AUTOWARE_IMAGE=selfcar-2026-vtd:local \
  -t selfcar-2026-carla:local \
  -f docker/carla/Dockerfile .
```

CARLA itself, point-cloud/Lanelet2 map assets, ML models, and generated
TensorRT engines are external runtime assets. The tested bus Lanelet2 map and
signal bundle are included under `maps/Town05Bus`; large point clouds, model
weights and generated engines are not stored in Git. With CARLA 0.9.16 installed under `/home/a/CARLA/0.9.16` and the map
and models under `/home/a/autoware_data`, prepare the Town05 map as described
above and run:

```bash
./scripts/carla/carla_run -quality-level=Low
CARLA_PERCEPTION_MODE=ground_truth ./scripts/carla/carla_autoware start-town05
./scripts/carla/carla_autoware status
```

For separate terminal commands, full restart instructions, the scenario GUI, and
the `start-town05: command not found` fix, see [the operations guide](docs/carla-operations.md).

The default object mode is `sensor`. Stop the existing runtime before switching
object modes; the launcher does not engage AUTO on startup.

The original VTD scenario adapter and matching OpenDRIVE are retained under
`tools/carla_vtd_scenario.py` and `config/carla/`. The native Town05 setup does
not use that converted scenario. See
[docs/carla-autoware-town01.md](docs/carla-autoware-town01.md) and
[docs/carla-vtd-scenario.md](docs/carla-vtd-scenario.md).

Town01's supplied Lanelet2 map has no traffic-light, stop-line, or crosswalk
regulatory elements. Vehicle and pedestrian perception and obstacle planning
are available, while traffic-rule behavior needs a CARLA-specific regulatory
map and traffic-signal bridge.

## Documentation

To learn more about using or developing Autoware, refer to the [Autoware documentation site](https://autowarefoundation.github.io/autoware-documentation/main/). You can find the source for the documentation in [autowarefoundation/autoware-documentation](https://github.com/autowarefoundation/autoware-documentation).

## Contributing

<p align="left">
    <a href="https://github.com/autowarefoundation/autoware_universe/graphs/contributors">
        <img src="https://img.shields.io/github/contributors/autowarefoundation/autoware_universe?style=flat&label=Autoware%20Universe%20Contributors"
            alt="Autoware Universe Contributors" /></a>
    <a href="https://github.com/autowarefoundation/autoware/graphs/contributors">
        <img src="https://img.shields.io/github/contributors/autowarefoundation/autoware?style=flat&label=Autoware%20Contributors"
            alt="Autoware Contributors" /></a>
</p>

<!--- Commit Activity -->
<p align="left">
    <a href="https://github.com/autowarefoundation/autoware_universe/pulse">
        <img src="https://img.shields.io/github/commit-activity/m/autowarefoundation/autoware_universe?style=flat&label=Autoware%20Universe%20Commit%20Activity"
            alt="Autoware Universe Activity" /></a>
    <a href="https://github.com/autowarefoundation/autoware/pulse">
        <img src="https://img.shields.io/github/commit-activity/m/autowarefoundation/autoware?style=flat&label=Autoware%20Commit%20Activity"
            alt="Autoware Activity" /></a>
</p>

- Make sure to follow the [Contribution Guidelines](https://autowarefoundation.github.io/autoware-documentation/main/contributing/).
- Take a look at Autoware's [various working groups](https://github.com/autowarefoundation/autoware-projects/wiki#working-group-list) to gain an understanding of any work in progress and to see how projects are managed.
- If you have any technical questions, you can start a discussion in the [Q&A category](https://github.com/autowarefoundation/autoware/discussions/categories/q-a) to request help and confirm if a potential issue is a bug or not.

## Useful resources

<!--- CI Reports -->
<p align="left">
    <a href="https://github.com/autowarefoundation/autoware/actions/workflows/health-check.yaml?query=branch%3Amain">
        <img src="https://img.shields.io/github/actions/workflow/status/autowarefoundation/autoware/health-check.yaml?style=flat&label=health-check"
            alt="health-check CI" /></a>
    <a href="https://app.codecov.io/gh/autowarefoundation/autoware_universe">
        <img src="https://img.shields.io/codecov/c/gh/autowarefoundation/autoware_universe?style=flat&label=Coverage&logo=codecov&logoColor=white"
            alt="Code Coverage" /></a>
</p>

- [Autoware Foundation homepage](https://www.autoware.org/)
- [Support guidelines](https://autowarefoundation.github.io/autoware-documentation/main/community/support/)
- [CI metrics](https://autowarefoundation.github.io/autoware-ci-metrics/)
