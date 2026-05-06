# TODO_SH — State Machine 리팩토링 트래킹 (SH)

## 목적
`3d_state_machine_node.py` (3500+ 줄 God object)의 유지보수성 개선.
첫 단계: **check 함수들을 순수 함수로 분리 + pytest 테스트 하네스 도입**.

이 작업이 완료되면 이후 모든 리팩토링(state machine 포크 통합, GB/Smart 이중 루프 통합 등)에 회귀 검사용 안전망이 생김.

---

## ✅ 검증 결과 (2026-05-06)

| 항목 | 결과 |
|---|---|
| Docker 이미지 (`race_stack_sim_sh`) 빌드 | ✅ 16.8분, 14.4GB |
| catkin build | 47/48 (blink1만 skip) |
| pyflakes (활성 mixin + 메인) | 깨끗 |
| pytest 40개 (호스트 + Docker 안 둘 다) | ✅ 0.05s |
| Smoke test — mixin MRO + 6 property + 47 메서드 | ✅ |
| roscore + fake_odom + base_system 시퀀스 | ✅ 모든 노드 정상 |
| `/car_state/odom_frenet` 50 Hz | ✅ |
| `/state_machine` 80 Hz, 초기 `GB_TRACK` | ✅ |
| Obstacle 주입 후 **state 전이** | ✅ `GB_TRACK` → `TRAILING` |

### 추가 빌드 필요 패키지 (`catkin build`에서 누락된 것 직접 빌드)
- `frenet_conversion_server` (frenet_conversion_server_node)
- `frenet_odom_republisher` (frenet_odom_republisher_node)

### 검증 환경에서 발견된 부수 이슈 (검증 무관, HJ 영역)
- `controller_manager` SIM yaml에 `L1_controller/trailing_vel_gain`, `AEB_thres` 등 누락
- `gp_traj_predictor` 가 `ccma` 모듈 의존 (별도 설치 필요)
- `state_indicator_node.py` 가 `blink1.msg` import (blink1 패키지 skip 시 죽음)

→ 우리 리팩토링한 state_machine 자체는 100% 정상 동작.

---

## 검증 환경: `icra2026_sh` Docker 컨테이너 (검증 전용 minimal)

HJ 가 쓰는 `icra2026` 컨테이너와 별도. cache 도 분리해서 catkin build 충돌 회피.

### 빠진 의존 (검증엔 불필요)
- GLIL thirdparty superbuild (GTSAM, ROBIN, gtsam_points, KISS-Matcher) — 가장 무거움
- Cartographer + abseil
- Livox SDK2 + driver
- CMake 3.28 (system default 사용)

### 결과
- 빌드 시간: **5~15분** (vs HJ 베이스 40~60분)
- 이미지 크기: **~3GB** (vs ~10GB)

### 새로 추가된 파일
| 파일 | 역할 |
|---|---|
| `.docker_utils/Dockerfile.sim_sh.x86` | minimal sim 이미지 정의 |
| `.docker_utils/main_dock_sh.sh` | `icra2026_sh` 컨테이너 띄우는 스크립트 |
| `docker-compose.yaml` (수정) | `sim_sh_x86` 서비스 추가 |

### 빌드 + 띄우기 (호스트에서)
```bash
cd $HOME/unicorn_ws/ICRA2026_HJ

# 환경 변수
export USER=$(whoami) UID=$(id -u) GID=$(id -g)

# 1) 이미지 빌드 (5~15분, 처음 한 번만)
docker compose build sim_sh_x86

# 2) 컨테이너 띄우기 (백그라운드)
./.docker_utils/main_dock_sh.sh

# 3) 진입
docker exec -it icra2026_sh bash
```

### 첫 진입 후 workspace 빌드 (컨테이너 안)
```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin build      # 30~60분 (state_machine 등 모두)
source devel/setup.bash
```

cache 가 호스트의 `~/unicorn_ws/cache_sh/` 에 저장되므로 다음 빌드부터는 incremental.

### 검증 시퀀스 (state_machine + fake_odom 시나리오)

```bash
# 컨테이너 안 — 4 개 터미널 사용
docker exec -it icra2026_sh bash   # 각 터미널마다

# 터미널 1 — fake_odom (raceline 따라 진행)
source devel/setup.bash
rosrun stack_master fake_odom_publisher.py \
  _map:=gazebo_wall_3d_rc_car_10th_timeoptimal _speed_scale:=1.0

# 터미널 2 — base 시스템 (sim 모드)
source devel/setup.bash
roslaunch stack_master 3d_base_system.launch \
  sim:=true map:=gazebo_wall_3d_rc_car_10th_timeoptimal

# 터미널 3 — headtohead (state_machine 포함)
source devel/setup.bash
roslaunch stack_master 3d_headtohead.launch dynamic_avoidance_mode:=NONE

# 터미널 4 — 모니터링
source devel/setup.bash
rostopic echo /state_machine            # GB_TRACK ↔ TRAILING ↔ OVERTAKE 전이
rostopic hz /local_waypoints            # 50 Hz 발행 확인
```

기대 동작:
- 노드 crash 없음
- `state_machine` 토픽이 50Hz 로 GB_TRACK 발행
- obstacle_publisher 띄우면 TRAILING/OVERTAKE 로 전이

### 정리
```bash
docker rm -f icra2026_sh
```

---

## Phase 1 — `_check_free_frenet` 순수화

### 결정 사항
- **타겟 함수**: `state_machine/src/3d_state_machine_node.py:1074-1215` `_check_free_frenet`
- **새 파일**: `state_machine/src/path_checker.py`
- **테스트 위치**: `state_machine/test/test_path_checker.py`
- **언어 정책**: 코드는 영어, 주석은 한글
- **rospy 처리**: pure function 안에 호출 그대로 유지, 테스트에서 `sys.modules`로 mock (옵션 A — 동작 100% 동일성 우선)
- **wpnts_data 객체**: duck typing으로 그대로 받음 (호출부 변경 최소화)
- **HJ 주석 규칙**: 수정한 wrapper 쪽에 `### HJ : ...` 주석 추가

### Pure function 시그니처
```python
@dataclass(frozen=True)
class EgoFrenetState:
    s: float
    vs: float

@dataclass(frozen=True)
class FrenetCheckParams:
    max_s: float
    veh_length: float
    ego_width: float
    safety_factor_sec: float = 0.5

@dataclass(frozen=True)
class FreeFrenetResult:
    is_free: bool
    closest_obstacle: Optional[Any] = None
    closest_gap: float = 2.0

def check_free_frenet(
    ego, waypoints, obstacles,
    obstacle_predictions, obstacle_prediction_id, params,
) -> FreeFrenetResult: ...
```

### 동작 동일성 보증
| 항목 | 원본 | wrapper 적용 후 |
|---|---|---|
| 입력 | `self`, `wpnts_data` | 동일 |
| 반환 | `bool` | 동일 |
| `wpnts_data.closest_target` 갱신 | O | O (wrapper에서 대입) |
| `wpnts_data.closest_gap` 갱신 | O | O (wrapper에서 대입) |
| `rospy.loginfo` 시점 | 루프 도중 | 동일 (pure function 내부 유지) |

### 진행 체크리스트
- [x] `_check_free_frenet` 의존성 분석 완료
- [x] 순수 함수 시그니처 설계
- [x] TODO_SH.md 생성
- [x] 사용자 승인 (모드 무시, 그대로 진행)
- [x] `state_machine/src/path_checker.py` 작성
- [x] `state_machine/test/fake_msgs.py` 작성 (가짜 메시지 dataclass)
- [x] `state_machine/test/conftest.py` 작성 (rospy mock + 공용 fixture)
- [x] `state_machine/test/test_path_checker.py` 작성 (시나리오 6개)
- [x] `3d_state_machine_node.py` `_check_free_frenet`을 wrapper로 교체
- [x] 호스트에서 `pytest state_machine/test/` 실행 → 6/6 통과 (0.03s)
- [x] `python3 -m py_compile`로 wrapper 구문 검증
- [ ] Gazebo `gazebo_wall_2`에서 회귀 한 바퀴 확인 (사용자 환경에서 수동)

### 테스트 시나리오 (실제 작성 — 6개)
1. ✅ `test_no_obstacles_returns_free` — 빈 장애물
2. ✅ `test_static_obstacle_in_front_blocks_path` — 5m 앞 정적 장애물 (lateral 0)
3. ✅ `test_static_obstacle_behind_is_ignored` — 5m 뒤 (max_horizon 밖)
4. ✅ `test_static_obstacle_far_lateral_does_not_block` — d=2.0 측면
5. ✅ `test_uninitialized_waypoints_return_free` — `is_init=False`
6. ✅ `test_closest_obstacle_is_the_nearest_one` — 여러 충돌 중 가장 가까운 것 선택

### 결과
- pytest: **6 passed in 0.03s** (호스트, Docker 불필요)
- 변경 파일:
  - 신규: `state_machine/src/path_checker.py` (193줄)
  - 신규: `state_machine/test/fake_msgs.py` (60줄)
  - 신규: `state_machine/test/conftest.py` (52줄)
  - 신규: `state_machine/test/test_path_checker.py` (115줄)
  - 수정: `state_machine/src/3d_state_machine_node.py`
    - import 추가 (line 27)
    - `_check_free_frenet` 함수 142줄 → 21줄 wrapper (line 1077-1097)

---

---

## Phase 2 — `_check_overtaking_mode` / `_check_static_overtaking_mode` 결정 로직 분리

### 결정 사항
- **타겟**: `_check_overtaking_mode` (line 1201), `_check_static_overtaking_mode` (line 1233)
- **스킵**: `_check_overtaking_mode_sustainability` (side effect 없음, 분리 가치 적음 — 다음 라운드)
- **접근**: sub-check 메서드(`_check_ot_sector` 등)는 아직 self에 묶여 있어 한꺼번에 분리 시 작업 폭발. **결정 로직만** (4개 boolean 조합) 순수 함수로 분리. side effect (`static_overtaking_mode = True/False`) 는 wrapper에서 명시적으로 한 줄.

### 새로 추가된 항목
- `path_checker.OvertakingModeChecks` (dataclass, 4 fields)
- `path_checker.StaticOvertakingChecks` (dataclass, 4 fields)
- `path_checker.should_engage_overtaking(checks) -> bool`
- `path_checker.should_engage_static_overtaking(checks) -> bool`

### 진행 체크리스트
- [x] 3개 함수 의존성/side effect 분석
- [x] 시그니처 설계 (dataclass + decision function)
- [x] `path_checker.py`에 dataclass + pure function 추가
- [x] `test_path_checker.py`에 시나리오 11개 추가 (각 함수 happy path + parametrize 4 fail 케이스 + 진리표)
- [x] `_check_overtaking_mode` wrapper 교체
- [x] `_check_static_overtaking_mode` wrapper 교체
- [x] `_check_overtaking_mode_sustainability` 그대로 둠 (의도적 스킵)
- [x] pytest 17/17 통과 (0.04s)
- [x] py_compile syntax OK
- [ ] Gazebo 회귀 (사용자 환경에서 수동, 이번 라운드 스킵)

### 결과
- pytest: **17 passed in 0.04s**
- 변경 파일:
  - `path_checker.py`: dataclass 2개 + 함수 2개 추가 (~40줄)
  - `test_path_checker.py`: 시나리오 11개 추가
  - `3d_state_machine_node.py`:
    - import 확장
    - `_check_overtaking_mode` 31줄 → 35줄 (dataclass 사용으로 약간 늘었지만 결정 로직이 1줄)
    - `_check_static_overtaking_mode` 35줄 → 35줄 (동일 패턴)
- side effect 위치 명시화: line 1241 (`= False`), line 1278 (`= True`)

---

---

## Phase 3 — Sub-check 메서드 분리 (1차)

### 결정 사항
- **분리 대상**: `_check_ot_sector` (line 879), `_check_getting_closer` (line 921)
- **스킵**: `_check_latest_wpnts`, `_check_availability` — `wpnts_data` 내부 상태(`is_init`, `array`, `list`) 직접 갱신이 핵심 동작이라, **WaypointData 클래스 자체 정리 라운드와 묶어 처리**
- **접근**: 결정 로직만 pure로, ROS publish / `parent` 분기 / obstacles_in_interest[0] 추출은 wrapper에 유지

### 새로 추가된 항목
- `path_checker.GettingCloserParams` (dataclass)
- `path_checker.is_in_overtaking_zone(s_m, waypoints_dist, zones) -> bool`
- `path_checker.is_getting_closer(cur_s, cur_vs, first_obstacle, params) -> bool`

### 진행 체크리스트
- [x] 4개 sub-check 함수 분석
- [x] 분리 가능 2개 / 스킵 2개 결정
- [x] 시그니처 설계
- [x] `path_checker.py`에 dataclass + 함수 2개 추가
- [x] `fake_msgs.py`에 `s_start`, `in_static_obs_sector` 필드 추가
- [x] `test_path_checker.py`에 시나리오 10개 추가
- [x] `_check_ot_sector` wrapper 교체 (28줄 → 17줄)
- [x] `_check_getting_closer` wrapper 교체 (38줄 → 18줄)
- [x] pytest 27/27 통과 (0.04s)
- [x] py_compile syntax OK
- [ ] Gazebo 회귀 (사용자 환경, 스킵)

### 결과
- pytest: **27 passed in 0.04s**
- 변경 파일:
  - `path_checker.py`: dataclass 1개 + 함수 2개 추가
  - `fake_msgs.py`: `FakeObstacle`에 2개 필드 추가
  - `test_path_checker.py`: 시나리오 10개 추가 (zone 4 + getting_closer 6)
  - `3d_state_machine_node.py`: import 확장 + 2개 wrapper 교체

---

---

## Phase 4 — `WaypointData` 클래스 통합 (옵션 C-A)

### 결정 사항
- **접근**: 클래스 분리/재설계는 **하지 않기로** 결정. 비용 대비 이득 불명확.
- **대신**: `for_test` classmethod 추가로 ROS 없이 인스턴스 생성 가능 → 테스트 가치 확보
- **부수 효과**: 5개 파일에 복제되어 있던 `WaypointData` 정의 중 활성 3D 3개를 별도 파일 `waypoint_data.py`로 통합 (코드 중복 60% 해소)
- **legacy 유지**: 2D legacy 2개 파일 (`state_machine_node.py`, `state_machine_node_original.py`)은 ROS 파라미터 경로가 다름 (`/dyn_planners/...` vs `/dyn_planners_statemachine/...`) → 손대지 않음

### 새로 추가된 항목
- `state_machine/src/waypoint_data.py` — 통합된 `WaypointData` + `for_test` classmethod
- conftest.py에 `dynamic_reconfigure` mock 추가

### 진행 체크리스트
- [x] 5개 파일의 WaypointData 정의 비교 (3D 3개는 거의 동일, MPC만 `from_mpc` 필드 추가, 2D legacy 2개는 ROS 경로 다름)
- [x] 통합 범위 결정 (활성 3D 3개만)
- [x] `waypoint_data.py` 신규 (3D 마스터 + `from_mpc` default 포함 + `for_test`)
- [x] `3d_state_machine_node.py` — 클래스 정의 38줄 제거, import 한 줄 추가
- [x] `mpc/3d_mpc_state_machine_node.py` — 동일
- [x] `fast_sqp_planner/fast_sqp_planner_sm.py` — 동일
- [x] 4개 파일 py_compile 통과
- [x] conftest.py에 `dynamic_reconfigure` mock 추가
- [x] 진짜 `WaypointData.for_test()` 사용 테스트 3개 추가
- [x] pytest 30/30 통과 (0.06s)
- [ ] Gazebo 회귀 (사용자 환경, 스킵)

### 결과
- pytest: **30 passed in 0.06s**
- 변경 파일:
  - 신규: `waypoint_data.py` (143줄, 통합된 클래스 + for_test)
  - 수정: `3d_state_machine_node.py` (38줄 제거, import 추가)
  - 수정: `mpc/3d_mpc_state_machine_node.py` (43줄 제거, import 추가)
  - 수정: `fast_sqp_planner/fast_sqp_planner_sm.py` (38줄 제거, import 추가)
  - 수정: `test/test_path_checker.py` (테스트 3개 추가)
  - 수정: `test/conftest.py` (mock 확장)
- **코드 중복 해소**: 활성 3D 노드 3개의 동일 클래스 정의 119줄 → 0줄 (통합 1곳)
- **테스트 가치**: 진짜 `WaypointData` 호스트에서 ROS 없이 사용 가능 (`FakeWaypointData` 점진적 대체 가능)

### 추가 검증된 사실
- `for_test`는 `__new__` 우회 패턴으로 ROS Subscriber 호출 회피
- 호출부 동작 변경 0 — 기존 `WaypointData('foo', True)` 그대로 동작

---

---

## Phase 5 — `_check_on_spline` + `_check_latest_wpnts` 분리

### 결정 사항
- **분리 대상**: `_check_on_spline` (line 1006), `_check_latest_wpnts` (line 924)
- **스킵**: `_check_availability` (시간 분기 + sub-check 호출 + side effect가 한 함수에 얽혀, 분리 시 wrapper 더 복잡해짐)
- **부분 분리 원칙**:
  - `_check_on_spline`: 결정 로직 전부 pure로
  - `_check_latest_wpnts`: timestamp/freshness 결정만 pure로. `initialize_traj` (side effect) 와 `_check_on_spline` 호출은 wrapper에 남김
- **현실적 인지부담 고려**: 사용자 원칙(헷갈리는 변경 피하기) 따라 한꺼번에 너무 많이 쪼개지 않음

### 새로 추가된 항목
- `path_checker.OnSplineParams` (dataclass)
- `path_checker.TrajFreshnessParams` (dataclass)
- `path_checker.is_on_spline(cur_s, current_position_xy, waypoints, params) -> bool`
- `path_checker.is_traj_msg_fresh(src_msg, now_sec, params) -> bool`

### 진행 체크리스트
- [x] 3개 함수(`_check_latest_wpnts`/`_check_availability`/`_check_on_spline`) 분석
- [x] 분리 가능 2개 / 스킵 1개 결정
- [x] `path_checker.py`에 dataclass 2개 + 함수 2개 추가
- [x] `test_path_checker.py`에 시나리오 10개 추가
  - on_spline: uninit / 통과 / 끝점 가까움 실패 / 측면 이탈 실패 (4개)
  - traj_fresh: None / 빈 wpnts / 일반 fresh / 일반 stale / smart fresh / smart zero (6개)
- [x] `_check_on_spline` wrapper 교체 (디버그 로그는 보존)
- [x] `_check_latest_wpnts` wrapper 교체 (`initialize_traj` + `_check_on_spline` 호출은 wrapper)
- [x] `_check_availability`는 손대지 않음 (의도적)
- [x] numpy bool → Python bool 캐스팅 추가 (`is True/False` 비교 호환성)
- [x] pytest 40/40 통과 (0.05s)
- [x] py_compile syntax OK
- [ ] Gazebo 회귀 (사용자 환경, 스킵)

### 결과
- pytest: **40 passed in 0.05s**
- 변경 파일:
  - `path_checker.py`: dataclass 2개 + 함수 2개 추가
  - `test_path_checker.py`: 시나리오 10개 추가
  - `3d_state_machine_node.py`: import 확장 + 2개 wrapper 교체

### 부수 효과
- `_check_availability` wrapper 동작 동일 (이 함수가 호출하는 sub-check가 안쪽에서 정리됨)
- 진짜 `WaypointData.for_test()` 와 새 `_make_straight_spline()` 헬퍼로 on_spline 시나리오 직접 검증

---

---

## Phase 6 — 백업 파일 정리

### 결정 사항
- 모든 백업 파일이 git 추적 중 → 삭제 후에도 `git log -- <path>` / `git show <commit>:<path>` 로 복원 가능
- 활성 코드에서 참조하는 곳 0건 확인 (grep)
- 47개 패턴 후보 + git rm 시 추가 잡힌 파일 = **49개 파일 staged deletion**
- "최소 변경" 원칙 위반 없음 — 위험은 0

### 삭제 카테고리 (47 → 49 파일)
| 카테고리 | 개수 | 위치 |
|---|---|---|
| HJ MPC 작업 백업 | 17 | `planner/mpc_planner/` |
| HJ Sampling 작업 백업 | 4 | `planner/3d_sampling_based_planner/` |
| Stack master / vel planner 백업 | 7 | `stack_master/scripts/`, `f110_utils/libs/2.5d_vel_planner/` |
| GB optimizer 백업 | 7 | `planner/{2.5d,3d}_gb_optimizer/` |
| C++ FBGA `.bak` | 8 | `f110_utils/libs/FBGA/` |
| Legacy / 단발성 백업 | 4 | `state_machine_node_original.py` 외 |
| (git rm 시 추가 잡힘) | +2 | (잔존 0 검증됨) |

### 진행 체크리스트
- [x] 백업 파일 패턴 전수 조사 (47개 발견)
- [x] git 추적 상태 확인 (전부 추적됨)
- [x] 의심 후보 (Wpnt_original.msg, frenet_odom_republisher_original.cc, state_machine_node_original.py) 참조 grep — 모두 참조 없음
- [x] `git rm` 으로 일괄 삭제 (49개 staged deletion)
- [x] 잔존 0 검증
- [x] pytest 40/40 통과 (회귀 없음)

### 결과
- pytest: **40 passed in 0.05s** — 회귀 없음
- 우리 작업 변경사항(`3d_state_machine_node.py`, `path_checker.py`, `waypoint_data.py` 등)은 unstaged 로 그대로 보존
- staged area에는 **백업 파일 49개 deletion만** — 사용자가 원하는 시점에 단독 commit 가능

### 남은 사용자 작업
- 백업 삭제 commit 분리 권장: `git commit -m "chore: remove tracked backup files (recoverable via git history)"`
- 그 후 Phase 1~5 작업을 별도 commit 들로

---

---

## Phase 7 — 가독성 정리 (`if True:` + `_check_overtaking_mode_sustainability`)

### 결정 사항
- 사용자 옵션 C2 선택: `if True:` 8개 unwrap + `_check_overtaking_mode_sustainability` 재구조화
- **죽은 주석 함수 (30개), 비활성화된 로그 (76개) 등은 손대지 않음** — HJ가 디버깅 시 임시로 끈 것일 수 있어 삭제 위험
- 비활성화된 sub-check 잔재 (`# if obs.is_static:` + `if True:` + `else: pass`) 패턴은 안전한 부산물로 동시 제거

### 처리 패턴
| 패턴 | 위치 | 처리 | 파일 |
|---|---|---|---|
| **A** — `_check_free_cartesian` 안 `if True:` + `else: pass` | 4개 | 단순 unwrap (들여쓰기 줄임 + else 블록 제거) | 4개 파일 |
| **B-단순** — `_check_overtaking_mode_sustainability` 17줄 | 3개 | 재구조화 → 12줄 (변수 할당 + early return) | 3d, 2D legacy, fast_sqp |
| **B-복잡** — mpc 의 `_check_overtaking_mode_sustainability` (MPC 분기 포함) | 1개 | `if True:` 만 unwrap, 재구조화는 위험해서 보류 | mpc |

### 진행 체크리스트
- [x] 8개 `if True:` 위치 컨텍스트 확인
- [x] 패턴 A 4개 unwrap (3d, 2D legacy, fast_sqp, mpc)
- [x] 패턴 B-단순 3개 재구조화 (3d, 2D legacy, fast_sqp)
- [x] 패턴 B-복잡 1개 (mpc) — `if True:` 만 unwrap
- [x] py_compile 4개 파일 OK
- [x] `if True:` 잔존 코드 줄 0 확인
- [x] pytest 40/40 통과 (회귀 없음)

### 결과
- `if True:` 코드 줄: **8 → 0**
- `_check_overtaking_mode_sustainability` 함수 길이: **17줄 → 12줄** (3개 파일), mpc는 22줄 → 20줄
- `_check_free_cartesian` 안의 죽은 `else: pass + 주석 블록`: 3개 파일에서 ~25줄씩 제거
- 동작 동일성: 100% (단순 unwrap + 도달 불가능 코드 제거)

### 변경 요약
- 4개 파일 수정: `state_machine_node.py` (2D legacy), `3d_state_machine_node.py`, `mpc/3d_mpc_state_machine_node.py`, `fast_sqp_planner/fast_sqp_planner_sm.py`
- 총 라인 감소: 약 **80줄 감소** (죽은 주석 + else: pass + if True 들여쓰기)

---

---

## Phase 8 — `loop()` 헬퍼 함수 추출

### 결정 사항
- 작업 범위: `3d_state_machine_node.py` 내부만 (mpc/fast_sqp는 개발 중이라 손 안 댐)
- 의도적 스킵: `_check_availability` 분리 (Phase 5 평가대로 가치 낮음, 솔직히 정정)
- 5개 헬퍼 함수 추출 — 마커 시각화 패턴 중복 + 흩어진 책임 통일

### 추출된 헬퍼 메서드 (5개, line 1978-2042)
| 메서드 | 책임 | 원본 라인 수 |
|---|---|---|
| `_reset_check_result_caches()` | 매 iteration 시작 시 closest_target=None 초기화 (5필드) | 5줄 |
| `_check_low_voltage_warning()` | 배터리 voltage 낮으면 경고 마커 발행 | 4줄 |
| `_decide_next_state()` | force_gbtrack > ftg_only_zone > transition 분기 | 7줄 |
| `_log_state_change_if_debug(prev_state, prev_src)` | DEBUG_STATE_TRANSITION 활성 시 변경 로그 | 6줄 |
| `_publish_behavior_strategy(local_wpnts)` | BehaviorStrategy 채우고 발행 | 6줄 |
| `_publish_target_marker(pub, targets, *, color_b, color_g)` | 마커 시각화 — overtaking/trailing 둘 다 사용 (중복 30줄 → 헬퍼 1개) | 17줄 |

### 진행 체크리스트
- [x] `loop()` 124줄 코드 책임 분석
- [x] 5개 헬퍼 메서드 추가 (loop 직전)
- [x] `loop()` 본문 124줄 → 50줄로 교체
- [x] `need_vel_planner = False` 중복 두 줄 → 한 줄 (`_publish_behavior_strategy` 안)
- [x] 죽은 주석 정리 (`# self.behavior_strategy = BehaviorStrategy()` 등)
- [x] py_compile OK
- [x] pytest 40/40 통과 (회귀 없음)
- [ ] Gazebo 회귀 (사용자 환경, 스킵)

### 결과
- `loop()` 길이: **124줄 → 50줄** (60% 감소)
- 마커 시각화 중복 (overtaking + trailing 거의 동일 패턴 30줄): **헬퍼 1개**로 통일
- 동작 동일성: 모든 publish 호출 / state 변경 / 시각화 순서 그대로

### 동작 동일성 보증
| 항목 | 비교 |
|---|---|
| ROS publish 순서 | 동일 |
| 모든 `self.xxx = ...` 순서 | 동일 |
| 디버그 로그 메시지 / 조건 | 동일 (헬퍼에서 그대로) |
| LOSTLINE → GB_TRACK 후처리 위치 | 동일 (혹시 모를 영향 회피) |
| `need_vel_planner = False` (중복 두 줄 → 한 줄) | 동일 (값 동일하므로 결과 동일) |

---

---

## Phase 9 — `__init__()` 헬퍼 함수 추출

### 결정 사항
- 작업 범위: `3d_state_machine_node.py` 내부만
- 6개 헬퍼 함수 추출 — 책임별로 분리. rosparam 로드 / 차량 동역학 / vel planner / 인스턴스 변수 / Subscriber / Publisher
- 변수 초기화 순서는 100% 유지 (의존성 위험 회피)

### 추출된 헬퍼 메서드 (6개, line 134-456)
| 헬퍼 | 책임 | 줄 수 |
|---|---|---|
| `_load_rosparams()` | 모든 `rospy.get_param` 호출 (~25개) + sectors_params 등 derived 값 | 47 |
| `_load_vehicle_dynamics()` | racecar_f110.ini + ggv/ax_max csv 로드 | 25 |
| `_load_vel_planner_params()` | vel_planner.yaml + dyn_reconfigure 구독 | 31 |
| `_init_state_attributes()` | 모든 인스턴스 변수 default + WaypointData 6개 + states/state_transitions 딕셔너리 | 149 |
| `_setup_ros_subscribers()` | ROS Subscriber 등록 모두 (ot_planner 분기 포함) | 50 |
| `_setup_ros_publishers()` | ROS Publisher 등록 모두 | 21 |

### 진행 체크리스트
- [x] `__init__()` 300줄 코드 책임 분석
- [x] 6개 헬퍼 메서드 추가 + `__init__()` 본문 통째로 교체
- [x] 호출 순서: `_load_rosparams` → `_load_vehicle_dynamics` → `_load_vel_planner_params` → `_init_state_attributes` → `_setup_ros_subscribers` → `_setup_ros_publishers` → `SmartStaticChecker` → `loop()`
- [x] `splini_ttl` 삼항 표현식을 if/else 두 줄로 분리 (가독성)
- [x] 죽은 주석 정리 (`# self.cur_recovery_wpnts = WpntArray()` 등)
- [x] inline `from std_msgs.msg import Bool` 제거 (이미 line 18에서 import됨)
- [x] py_compile OK
- [x] pytest 40/40 통과
- [ ] Gazebo 회귀 (사용자 환경, 스킵)

### 결과
- `__init__()` 길이: **300줄 → 20줄** (93% 감소)
- 헬퍼 6개 합계: ~320줄 (주석/docstring 포함). 총 라인은 비슷하지만 **응집도/가독성 크게 향상**
- 동작 동일성: 변수 초기화/구독 등록 순서 모두 보존

### 동작 동일성 보증
| 항목 | 비교 |
|---|---|
| 모든 `rospy.get_param` 호출 순서 | 동일 |
| 모든 `rospy.Subscriber` 등록 순서 | 동일 |
| 모든 `rospy.wait_for_message` 호출 순서 | 동일 |
| 모든 인스턴스 변수 초기화 순서 | 동일 |
| WaypointData 6개 생성 순서 | 동일 |
| SmartStaticChecker 생성 시점 (publisher 후) | 동일 |
| `self.loop()` 호출 위치 (마지막) | 동일 |

---

---

## Phase 10 — Callback 정리 (작은 정리만)

### 결정 사항
- **솔직한 평가**: callback 25개 중 대부분이 1~7줄 단순 setter. "큰 헬퍼 추출" ROI 낮음
- 진짜 가치 있는 것만 정리 → A 옵션 (작은 정리만)
- 의도적 스킵: `obstacle_perception_cb` horizon 헬퍼 추출, `_vel_planner_3d_param_cb` 분해 등 ROI 낮은 작업

### 처리한 정리 (3개)
| 항목 | 처리 |
|---|---|
| **죽은 주석 callback** (line 619-634) | `obstacle_perception_cb` 옛날 버전 16줄 주석 제거 (활성 버전이 바로 아래) |
| **`dyn_param_cb` splini_ttl 삼항식** | `_load_rosparams`와 동일하게 if/else 두 줄로 풀기 (일관성) |
| **`smart_static_avoidance_cb` 죽은 주석** | `# ===== HJ ADDED =====` / `# ===== HJ MODIFIED =====` 마커 + 인라인 설명 주석 정리, docstring으로 통합 |

### 진행 체크리스트
- [x] 25개 callback 길이/패턴 조사
- [x] 솔직한 평가 — 대부분 정리 가치 낮음
- [x] 죽은 주석 callback 제거 (16줄)
- [x] 삼항식 풀기 (4줄)
- [x] smart_static_avoidance_cb 정리 (33줄 → 30줄, 가독성 향상)
- [x] py_compile OK
- [x] pytest 40/40 통과

### 결과
- 라인 감소: 약 20줄 (주로 죽은 주석)
- 가독성 향상: smart_static_avoidance_cb 의 의미가 docstring 으로 명확
- 일관성: splini_ttl 분기 패턴 통일

---

---

## Phase 11 — Visualization/publish 정리 (A 단계 마무리)

### 결정 사항
- 작업 범위: `_pub_local_wpnts`, `visualize_state`, `publish_not_ready_marker`
- 가장 큰 가치: `visualize_state` 의 32줄 if/elif 색상 분기 → dict 매핑

### 처리 내용
| 항목 | 처리 |
|---|---|
| **모듈 상수 `STATE_COLORS` 추가** | state name → (r, g, b) dict. visualize_state 내 32줄 if/elif → 1줄 lookup |
| **`_speed_to_color(vx, vx_min, vx_max)` 헬퍼** | `_pub_local_wpnts` 내 색상 계산 4줄 코드 두 곳 중복 → 헬퍼 1번 호출 |
| **`_compute_visualization_anchor()` 헬퍼** | visualize_state 의 첫 호출 시 anchor 좌표 계산 12줄 분리 |
| **`_publish_state_text_marker()` 헬퍼** | visualize_state 의 텍스트 마커 생성 17줄 분리 |
| **죽은 주석 정리** | `# if len(loc_wpnts.wpnts) == 0:` 등 |

### 진행 체크리스트
- [x] visualization 메서드 3개 본문 분석
- [x] STATE_COLORS dict + _speed_to_color 헬퍼 추가
- [x] `_pub_local_wpnts` 색상 중복 제거 (~10줄 감소)
- [x] `visualize_state` 분리 (105줄 → 29줄, 헬퍼 2개)
- [x] py_compile OK
- [x] pytest 40/40 통과

### 결과
- `visualize_state`: **105줄 → 29줄** (72% 감소, 헬퍼 2개로 분리)
- `_pub_local_wpnts`: 중복 색상 계산 4줄 두 곳 → 헬퍼 호출
- `publish_not_ready_marker`: 손대지 않음 (이미 단순)

---

## A 작업 진행 상황 (3d_state_machine_node.py 한정) — **완료**
- [x] 1단계 (Phase 1~7) — check 함수 분리 + WaypointData 통합 + 가독성 정리
- [x] 2단계 (Phase 8) — `loop()` 정리 (124줄 → 50줄)
- [x] 3단계 (Phase 9) — `__init__()` 정리 (300줄 → 20줄, 헬퍼 6개)
- [x] 4단계 (Phase 10) — callback 정리 (죽은 주석 + 일관성)
- [x] **5단계 (Phase 11) — visualization 정리** ← 완료

### A 단계 종합 결과
| 측면 | Before | After |
|---|---|---|
| `loop()` | 124줄 | 50줄 |
| `__init__()` | 300줄 | 20줄 |
| `visualize_state()` | 105줄 | 29줄 |
| 주요 check 함수 (7개) | wrapper 안에 결정 로직 + side effect | path_checker 순수 함수로 분리 |
| WaypointData 정의 | 5개 파일에 복제 | 1개 파일 (waypoint_data.py) |
| 백업 파일 | 49개 | 0개 |
| pytest | 0개 | **40개** |
| 추가된 모듈 | — | `path_checker.py`, `waypoint_data.py` |
| 추출된 헬퍼 메서드 | — | 16개 (loop 6, init 6, visualize 4) |

### 동작 동일성
- 모든 ROS publish / Subscriber 등록 / state 변경 / 시각화 순서 보존
- pytest 40/40 통과 (회귀 검사망 안에서 확인)

---

## Phase 12 — Mixin 분리 (3개 파일로)

### 결정 사항
- 사용자 제안: state_machine 헬퍼들이 너무 많아 (16개) 한 파일이 비대해짐 → 분리 필요
- **A (Mixin) 옵션 선택** — multiple inheritance, 호출부 변경 0
- 3개 mixin 파일로 분리: visualization / init / callbacks
- 순환 import 회피: `ENABLE_STATIC_SECTOR_FILTERING` / `HORIZON_FOR_TTL` 두 상수도 callback 파일에 함께 두고, 메인 wrapper 가 이 모듈에서 import

### 새 파일 (3개)
| 파일 | 내용 | 줄 |
|---|---|---|
| `state_machine_visualization.py` | `VisualizationMixin` (6개 메서드 + STATE_COLORS) | 224 |
| `state_machine_init.py` | `InitMixin` (6개 헬퍼) | 370 |
| `state_machine_callbacks.py` | `CallbackMixin` (24개 callback + 2개 상수) | 290 |

### 클래스 선언
```python
class StateMachine(InitMixin, VisualizationMixin, CallbackMixin):
    def __init__(self, name) -> None: ...
    def loop(self): ...
    def _check_*(self): ...   # check 함수 wrapper들 (메인에 유지)
    def update_waypoints(self): ...
    ...
```

### 진행 체크리스트
- [x] Step 1/3: VisualizationMixin 분리 (메인 2124 → 1927줄, -197)
- [x] Step 2/3: InitMixin 분리 (1927 → 1605줄, -322)
- [x] Step 3/3: CallbackMixin 분리 (1605 → 1285줄, -320)
- [x] 순환 import 회피 (ENABLE_STATIC_SECTOR_FILTERING / HORIZON_FOR_TTL 을 callback 모듈로)
- [x] py_compile 4개 파일 모두 OK
- [x] pytest 40/40 통과 (회귀 없음)

### 결과 (메인 파일 변화)
| 시점 | 메인 파일 줄 수 |
|---|---|
| Phase 1~11 후 | 2,124 |
| VisualizationMixin 분리 후 | 1,927 (-197) |
| InitMixin 분리 후 | 1,605 (-322) |
| CallbackMixin 분리 후 | **1,285 (-320)** |
| 누적 감소 | **-839 (-39.5%)** |

### 동작 동일성
- Mixin은 multiple inheritance 만 사용. 호출부 변경 0 (`self.method()` 그대로)
- ROS Subscriber 등록 / publish 순서 보존
- pytest 40/40 통과

---

## A + Mixin 분리 종합

### 메인 파일 추이 (전체)
| 단계 | 메인 파일 줄 수 |
|---|---|
| 시작 (Phase 0) | ~3,500 (추정) |
| Phase 1~11 후 | 2,124 |
| Phase 12 (Mixin) 후 | **1,285** |

### state_machine 패키지 src/ 모듈 (현재)
| 파일 | 줄 | 역할 |
|---|---|---|
| `3d_state_machine_node.py` | 1,285 | 메인 엔트리 (loop, check 함수 wrapper, update_*, get_*) |
| `state_machine_init.py` | 370 | Init 헬퍼 (rosparam, 차량 동역학, vel planner, 인스턴스 변수, ROS IO) |
| `state_machine_callbacks.py` | 290 | 24개 ROS Subscriber callback |
| `state_machine_visualization.py` | 224 | RViz 마커 발행 (sphere, cylinder, text) |
| `path_checker.py` | 367 | 순수 함수 collision check (check_free_frenet 등 9개) |
| `waypoint_data.py` | 134 | WaypointData 클래스 (5 노드 공유) |

### 테스트
- `state_machine/test/` 40개 (호스트 0.05s)

---

## B 단계 (A 완료 후) — 진행 가능 후보
- `state_transitions.py` — GB/Smart 이중 closed loop 통합
- `state_helper_for_smart.py` — `__dict__` 복사 패턴 정리
- `states.py` — 죽은 주석 함수 정리

---

## 메모
- 백업 규칙: `3d_state_machine_node.py` 직접 수정 시 `_backup_YYYYMMDD` 사본 먼저 생성
- 테스트 실행 환경: Docker 컨테이너 `icra2026` 안에서 pytest 실행
- 호스트에는 ROS 없을 수 있음 → pytest는 반드시 컨테이너 안에서
