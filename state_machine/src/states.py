"""각 state 별로 local waypoint 리스트를 만드는 함수 모음.

모든 함수는 state_machine 객체를 받아 List[Wpnt] (또는 빈 리스트)를 반환한다.
state_machine_node 의 `self.states` 딕셔너리가 StateType -> 이 함수들로 매핑된다.
"""
from __future__ import annotations

from typing import TYPE_CHECKING, List
from f110_msgs.msg import Wpnt

if TYPE_CHECKING:
    from state_machine_node import StateMachine


def GlobalTracking(state_machine: StateMachine) -> List[Wpnt]:
    s = int(state_machine.cur_s / state_machine.waypoints_dist + 0.5)
    return [
        state_machine.cur_gb_wpnts.list[(s + i) % state_machine.num_glb_wpnts]
        for i in range(state_machine.n_loc_wpnts)
    ]


def Overtaking(state_machine: StateMachine) -> List[Wpnt]:
    """Overtaking waypoint 생성. Priority:

    1. 정적 장애물 회피 (`static_overtaking_mode`) — ot_planner 무관
    2. spliner / predictive_spliner 동적 회피
    3. 사전 계산된 overtake_wpnts (다른 planner fallback)
    """
    if state_machine.static_overtaking_mode:
        return state_machine.get_splini_wpts()  # cur_static_avoidance_wpnts 사용

    if state_machine.ot_planner in ("spliner", "predictive_spliner"):
        return state_machine.get_splini_wpts()  # cur_avoidance_wpnts 사용

    # 다른 planner (graph_based 등) — 사전 계산된 OT 라인 사용
    s = state_machine.cur_id_ot
    return [
        state_machine.overtake_wpnts[(s + i) % state_machine.num_ot_points]
        for i in range(state_machine.n_loc_wpnts)
    ]


def RECOVERY(state_machine: StateMachine):
    return state_machine.get_recovery_wpts()


def START(state_machine: StateMachine):
    return state_machine.get_start_wpts()


def FTGOnly(state_machine: StateMachine):
    """FTG-only state — 제어 입력은 control 노드가 직접 생성, wpnts 없음."""
    return []


def SmartStatic(state_machine: StateMachine) -> List[Wpnt]:
    """Smart Static — GB optimizer fixed path 사용."""
    return state_machine.get_smart_static_wpts()
