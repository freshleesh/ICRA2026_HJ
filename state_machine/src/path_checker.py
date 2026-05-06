"""Pure-function path validation for the state machine.

이 모듈은 state machine의 collision check 로직을 ROS 의존성에서 분리한 순수 함수 모음이다.
모든 함수는 명시적 인자만 받으며, 입력 객체를 변경하지 않는다 (rospy 로깅 제외).
호출자(wrapper)는 결과 dataclass를 받아 필요한 side effect를 처리한다.
"""

from dataclasses import dataclass
from typing import Any, List, Optional, Tuple

import numpy as np
import rospy  # 로깅용. 테스트에서는 sys.modules로 mock 처리한다.


# =============================================================================
# 입출력 dataclass
# =============================================================================

@dataclass(frozen=True)
class EgoFrenetState:
    """자차의 Frenet 상태 (체크 함수가 의존하는 최소 필드만)."""
    s: float    # 진행 거리 [m]
    vs: float   # 종방향 속도 [m/s]


@dataclass(frozen=True)
class FrenetCheckParams:
    """충돌 체크에 필요한 차량/트랙 파라미터."""
    max_s: float                    # 트랙 한 바퀴 길이 [m]
    veh_length: float               # 자차 길이 [m]
    ego_width: float                # 자차 폭 [m]
    safety_factor_sec: float = 0.5  # 안전 시간 마진 [s] (현재 미사용, 향후 확장용)


@dataclass
class FreeFrenetResult:
    """check_free_frenet 결과.

    - is_free=True 이면 경로에 충돌 없음
    - is_free=False 이면 closest_obstacle / closest_gap 에 가장 가까운 충돌 정보가 담김
    """
    is_free: bool
    closest_obstacle: Optional[Any] = None
    closest_gap: float = 2.0


@dataclass(frozen=True)
class OvertakingModeChecks:
    """동적 OT 모드 진입 조건들 (모두 충족 시 진입)."""
    in_ot_sector: bool          # 현재 위치가 OT sector 안에 있는가
    is_getting_closer: bool     # 장애물이 다가오고 있는가 (10m threshold)
    wpnts_are_latest: bool      # avoidance_wpnts가 최근 (timestamp 내)인가
    path_is_free: bool          # avoidance 경로가 충돌 없는가


@dataclass(frozen=True)
class StaticOvertakingChecks:
    """정적 OT 모드 진입 조건들 (모두 충족 시 진입)."""
    velocity_safe: bool         # 자차 속도가 정적 OT 진입 가능한 임계 이하인가
    is_getting_closer: bool     # 장애물이 다가오고 있는가 (7m threshold)
    wpnts_are_latest: bool      # static_avoidance_wpnts가 최근인가
    path_is_free: bool          # static_avoidance 경로가 충돌 없는가


@dataclass(frozen=True)
class GettingCloserParams:
    """is_getting_closer가 의존하는 트랙/필터 파라미터."""
    horizon_for_ttl: float          # 정적 sector 장애물의 거리 임계 [m]
    static_sector_filtering: bool   # 정적 sector 필터링 활성 여부 (전역 플래그)
    track_length: float             # 트랙 한 바퀴 길이 [m]


@dataclass(frozen=True)
class OnSplineParams:
    """is_on_spline 가 의존하는 트랙/임계 파라미터."""
    track_length: float                 # 트랙 한 바퀴 길이 [m] (wrap-around용)
    front_horizon_thres_m: float        # wpnt 끝까지 남은 진행거리 최소 임계 [m]
    min_dist_thres_m: float             # 자차 ↔ 가장 가까운 wpnt 거리 최대 임계 [m]


@dataclass(frozen=True)
class TrajFreshnessParams:
    """is_traj_msg_fresh 가 의존하는 timestamp 임계."""
    is_smart_static: bool               # smart static 플래너인가 (특수 처리)
    latest_threshold_sec: float         # 일반 플래너의 최대 stale 시간 [s]


# =============================================================================
# Pure functions
# =============================================================================

def check_free_frenet(
    ego: EgoFrenetState,
    waypoints: Any,                   # WaypointData (duck typed: 아래 필드만 읽음)
    obstacles: List[Any],             # 현재 관심 장애물 목록 (Obstacle 메시지)
    obstacle_predictions: List[Any],  # 동적 장애물 예측 시계열 (PredictionStep)
    obstacle_prediction_id: int,      # 위 예측이 적용되는 장애물 id
    params: FrenetCheckParams,
) -> FreeFrenetResult:
    """경로(waypoints) 위에 충돌 가능한 장애물이 있는지 Frenet 좌표계에서 체크.

    `3d_state_machine_node.StateMachine._check_free_frenet`에서 추출한 순수 함수.
    원본과 동일한 로직을 유지하며, side effect(`wpnts_data.closest_target/closest_gap` 수정)만
    제거하여 결과 dataclass로 반환한다.

    waypoints 객체에서 읽는 필드 (duck typing):
        - is_init, is_closed, is_gb_track_wpnts, is_ot_wpnts: bool
        - max_horizon, lateral_width_m, free_scaling_reference_distance_m: float
        - array: np.ndarray, columns = [x_m, y_m, s_m, d_m]
        - list: List[Wpnt-like], 각 원소는 .d_m 을 가짐

    obstacle 객체에서 읽는 필드:
        - s_center, d_center: float
        - is_static: bool
        - size: float
        - vs: float
        - id: int

    obstacle_predictions 원소에서 읽는 필드:
        - pred_s, pred_d: float
    """
    # 웨이포인트가 아직 채워지지 않았으면 자유로 간주 (원본과 동일 동작)
    if not waypoints.is_init:
        return FreeFrenetResult(is_free=True)

    is_free = True
    closest_obs: Optional[Any] = None
    min_gap: float = 2.0

    max_horizon = waypoints.max_horizon
    is_gb_track_wpnts = waypoints.is_gb_track_wpnts
    is_ot_wpnts = waypoints.is_ot_wpnts
    free_scaling_reference_distance_m = waypoints.free_scaling_reference_distance_m
    lateral_width_m = waypoints.lateral_width_m

    # 비폐곡선 wpnt를 위한 끝점까지의 진행 거리
    max_gap = (waypoints.array[-1, 2] - ego.s) % params.max_s

    for obs in obstacles:
        obs_s = obs.s_center
        # wrap-around: 자차에서 장애물까지의 진행 거리 (트랙 둘레 기준 모듈로)
        gap = (obs_s - ego.s) % params.max_s
        relative_vs = ego.vs - obs.vs
        clip_vs = max(relative_vs, 0.5)
        # ttc / tt0: 장애물 진입/이탈 시점 (자차 기준 충돌 시간 윈도우)
        ttc = (gap - params.veh_length) / clip_vs
        tt0 = (gap + 0.3 * params.veh_length) / clip_vs

        if obs.is_static:
            # 비폐곡선 + 장애물이 wpnt 끝 너머에 있으면 충돌로 처리
            # (Closed Wpnts is Short!! 경고용)
            if not waypoints.is_closed and gap > max_gap:
                is_free = False
                if closest_obs is None or min_gap > gap:
                    closest_obs = obs
                    min_gap = gap

            elif gap < max_horizon:
                # 장애물의 d 좌표를 wpnt 기준 d로 변환
                ot_d = 0
                if not is_gb_track_wpnts:
                    avoid_wpnt_idx = np.argmin(abs(waypoints.array[:, 2] - obs_s))
                    ot_d = waypoints.list[avoid_wpnt_idx].d_m
                min_dist = abs(ot_d - obs.d_center)

                free_dist = min_dist - obs.size / 2 - params.ego_width / 2

                # 거리가 멀수록 lateral 안전 마진 작아지도록 스케일링
                scaling_factor = np.clip(gap / free_scaling_reference_distance_m, 0.0, 1.0)
                if free_dist < lateral_width_m * scaling_factor:
                    is_free = False
                    rospy.loginfo(
                        "[State Machine] FREE False, obs dist to ot lane: {} m".format(free_dist)
                    )
                    if closest_obs is None or min_gap > gap:
                        closest_obs = obs
                        min_gap = gap
        else:
            # 동적 장애물: 예측 궤적이 있으면 그것을 따라 충돌 윈도우만 체크
            if len(obstacle_predictions) != 0 and obstacle_prediction_id == obs.id:
                start_idx = 0
                end_idx = len(obstacle_predictions)

                # OT wpnt면 ttc/tt0 시점 사이의 예측만 체크 (예측 dt 50ms 가정)
                if is_ot_wpnts:
                    if ttc > 0:
                        start_idx = min(int(ttc / 0.05), len(obstacle_predictions))
                    if tt0 > 0:
                        end_idx = min(int(tt0 / 0.05), len(obstacle_predictions))

                for obs_pred in obstacle_predictions[start_idx:end_idx]:
                    wpnt_idx = np.argmin(abs(waypoints.array[:, 2] - obs_pred.pred_s))
                    wpnt_d = waypoints.list[wpnt_idx].d_m
                    min_dist = abs(wpnt_d - obs_pred.pred_d)
                    free_dist = min_dist - obs.size / 2 - params.ego_width / 2
                    scaling_factor = np.clip(gap / free_scaling_reference_distance_m, 0.0, 1.0)
                    if is_ot_wpnts:
                        rospy.logwarn(
                            f"free_dist: {free_dist}, lateral_width_m: {lateral_width_m}, "
                            f"scaling_factor: {scaling_factor}, obs.size: {obs.size}, "
                            f"wpnt_d:{wpnt_d}, obs_pred.pred_d: {obs_pred.pred_d} "
                        )
                    if free_dist < lateral_width_m * scaling_factor:
                        is_free = False
                        if closest_obs is None or min_gap > gap:
                            closest_obs = obs
                            min_gap = gap
            else:
                # 예측이 없는 동적 장애물: 정적 장애물처럼 현재 위치만으로 체크
                if not waypoints.is_closed and gap > max_gap:
                    is_free = False
                    if closest_obs is None or min_gap > gap:
                        closest_obs = obs
                        min_gap = gap
                elif gap < max_horizon:
                    ot_d = 0
                    if not is_gb_track_wpnts:
                        avoid_wpnt_idx = np.argmin(abs(waypoints.array[:, 2] - obs.s_center))
                        ot_d = waypoints.list[avoid_wpnt_idx].d_m
                    min_dist = abs(ot_d - obs.d_center)

                    free_dist = min_dist - obs.size / 2 - params.ego_width / 2

                    scaling_factor = np.clip(gap / free_scaling_reference_distance_m, 0.0, 1.0)
                    if free_dist < lateral_width_m * scaling_factor:
                        is_free = False
                        if closest_obs is None or min_gap > gap:
                            closest_obs = obs
                            min_gap = gap

    return FreeFrenetResult(
        is_free=is_free,
        closest_obstacle=closest_obs,
        closest_gap=min_gap,
    )


def should_engage_overtaking(checks: OvertakingModeChecks) -> bool:
    """동적 OT 진입 결정. 4개 조건 전부 참이어야 진입.

    원본 `_check_overtaking_mode`의 결정부 추출. side effect (static_overtaking_mode 갱신)는
    호출자(wrapper)가 처리한다.
    """
    return (
        checks.in_ot_sector
        and checks.is_getting_closer
        and checks.wpnts_are_latest
        and checks.path_is_free
    )


def should_engage_static_overtaking(checks: StaticOvertakingChecks) -> bool:
    """정적 OT 진입 결정. 4개 조건 전부 참이어야 진입.

    원본 `_check_static_overtaking_mode`의 결정부 추출. side effect (static_overtaking_mode
    갱신)는 호출자(wrapper)가 처리한다.
    """
    return (
        checks.velocity_safe
        and checks.is_getting_closer
        and checks.wpnts_are_latest
        and checks.path_is_free
    )


def is_in_overtaking_zone(
    s_m: float,
    waypoints_dist: float,
    zones: List[Tuple[float, float]],
) -> bool:
    """현재 진행거리(s_m)가 OT 허용 zone 안에 있는지 확인.

    원본 `_check_ot_sector`의 zone 매칭 로직 추출. zone은 (start_idx, end_idx) 튜플 리스트로,
    waypoint 인덱스 단위(s_m / waypoints_dist)로 비교한다. ROS publish 등 side effect는 wrapper에서.
    """
    idx = s_m / waypoints_dist
    return any(start <= idx <= end for start, end in zones)


def is_getting_closer(
    cur_s: float,
    cur_vs: float,
    first_obstacle: Optional[Any],
    params: GettingCloserParams,
) -> bool:
    """관심 장애물(첫 번째)이 자차에 가까워지고 있는지 판정.

    원본 `_check_getting_closer`의 결정 로직 추출. obstacle 리스트의 첫 번째 원소만 의존하므로
    호출자가 None 또는 obstacle 객체를 명시적으로 넘긴다.

    obstacle 객체에서 읽는 필드:
        - in_static_obs_sector: bool
        - is_static: bool
        - s_start: float
        - vs: float
    """
    if first_obstacle is None:
        return False

    obs = first_obstacle
    is_static_in_static_sector = obs.in_static_obs_sector and obs.is_static

    # 정적 sector + 필터링 활성: 거리 임계 안일 때만 "가까워짐"으로 본다
    if is_static_in_static_sector and params.static_sector_filtering:
        distance = (obs.s_start - cur_s) % params.track_length
        if distance > params.horizon_for_ttl:
            return False

    # 그 외 케이스: 상대속도 기준
    return cur_vs - obs.vs > -0.5


def is_on_spline(
    cur_s: float,
    current_position_xy: np.ndarray,    # shape (2,) — [x, y]
    waypoints: Any,                     # WaypointData (duck typed)
    params: OnSplineParams,
) -> bool:
    """자차가 waypoint spline 위에 있는지 (시작점 충분 + 가까이 붙어있는지) 판정.

    원본 `_check_on_spline`의 결정 로직 추출. ROS 로깅은 wrapper에 남는다.

    waypoints 객체에서 읽는 필드:
        - is_init: bool
        - list[-1].s_m: float       (마지막 wpnt의 진행거리)
        - array[:, 0:2]: np.ndarray  (모든 wpnt의 x, y)

    조건 두 가지 모두 충족해야 spline 위로 본다:
        1. 마지막 wpnt 까지의 남은 진행거리(gap)가 front_horizon_thres_m 보다 클 것
        2. 가장 가까운 wpnt 까지의 cartesian 거리가 min_dist_thres_m 보다 작을 것
    """
    if not waypoints.is_init:
        return False
    gap = (waypoints.list[-1].s_m - cur_s) % params.track_length
    min_dist = np.min(np.linalg.norm(waypoints.array[:, 0:2] - current_position_xy, axis=1))
    # numpy scalar 와 Python float 비교 결과를 명시적으로 Python bool 로
    return bool(gap > params.front_horizon_thres_m and min_dist < params.min_dist_thres_m)


def is_traj_msg_fresh(
    src_msg: Optional[Any],
    now_sec: float,
    params: TrajFreshnessParams,
) -> bool:
    """들어온 trajectory 메시지가 fresh 한지 (사용 가능한지) 판정.

    원본 `_check_latest_wpnts`의 timestamp 검사 부분만 추출. `initialize_traj` 호출과
    `_check_on_spline` 호출은 wrapper에 남는다 (side effect / 다른 sub-check).

    src_msg 객체에서 읽는 필드:
        - wpnts: list (비어있는지 확인)
        - header.stamp.is_zero(): bool
        - header.stamp.to_sec(): float

    Smart Static 플래너는 stamp 초기화만 됐으면 영구 유효.
    그 외 플래너는 (now - stamp) <= latest_threshold_sec 이어야 fresh.
    """
    if src_msg is None or len(src_msg.wpnts) == 0:
        return False

    if params.is_smart_static:
        # Smart Static: stamp가 0이 아니면 항상 valid (path는 한번 만들어지면 변하지 않음)
        return not src_msg.header.stamp.is_zero()

    # 일반 플래너: timestamp 임계 체크
    time_diff = now_sec - src_msg.header.stamp.to_sec()
    return time_diff <= params.latest_threshold_sec
