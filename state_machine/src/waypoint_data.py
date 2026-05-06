"""WaypointData — state_machine 노드들이 공유하는 plannner-별 wpnt 컨테이너.

이 모듈은 5개 state_machine 노드 파일에 복제되어 있던 동일 클래스를 한 곳으로 모은 것이다.
- 활성 3D 노드들 (3d_state_machine_node, mpc/3d_mpc_state_machine_node, fast_sqp_planner_sm)
  은 모두 이 파일을 import 한다.
- 2D legacy 파일들 (state_machine_node, state_machine_node_original)은 ROS 파라미터 경로가
  다르므로(`/dyn_planners/...`) 자기 안의 정의를 그대로 유지한다.

`for_test` classmethod 는 ROS 환경 없이 테스트용 인스턴스를 만들기 위한 편의 진입점이다.
"""

import numpy as np
import rospy
from dynamic_reconfigure.msg import Config


class WaypointData:
    """플래너 한 개의 wpnt 데이터 + 동적 파라미터 + 결과 캐시 컨테이너.

    필드 묶음 (역사적 이유로 한 클래스에 모여있음):
        - wpnt 데이터: list, array, stamp, is_init
        - 트랙 메타: is_closed, is_gb_track_wpnts, is_ot_wpnts
        - 외부 함수가 적어두는 결과 캐시: closest_target, closest_gap
        - ROS 동적 파라미터: min_horizon, max_horizon, lateral_width_m, ...
        - MPC 출처 표식 (MPC 노드에서만 사용): from_mpc

    생성자는 ROS Subscriber/get_param에 의존하므로, 테스트 환경에서는 `for_test` 사용.
    """

    def __init__(self, planner_name, is_closed):
        self.name = planner_name
        self.node_name = "/dyn_planners_statemachine/" + self.name
        self.list = []
        self.array = None
        self.stamp = None
        self.is_init = False
        self.is_gb_track_wpnts = False
        self.is_ot_wpnts = False
        self.closest_target = None
        self.closest_gap = None
        self.is_closed = is_closed
        self.vel_planner_safety_factor = 1.0
        # ### HJ : Phase X (refactored) — provenance tag. When the last
        # initialize_traj came from the unified MPC topic, set this True
        # so get_splini_wpts / get_recovery_wpts can choose NOT to pad
        # with GB waypoints (user directive 2026-04-24 "mpc 출력 그대로").
        self.from_mpc = False
        self.dyn_sub = rospy.Subscriber(
            self.node_name + "/parameter_updates", Config, self.dyn_param_cb
        )
        self.update_param()

    def dyn_param_cb(self, config):
        self.update_param()

    def update_param(self):
        self.min_horizon = rospy.get_param(self.node_name + "/min_horizon")
        self.max_horizon = rospy.get_param(self.node_name + "/max_horizon")
        self.lateral_width_m = rospy.get_param(self.node_name + "/lateral_width_m")
        self.free_scaling_reference_distance_m = rospy.get_param(
            self.node_name + "/free_scaling_reference_distance_m"
        )
        self.latest_threshold = rospy.get_param(self.node_name + "/latest_threshold")
        self.on_spline_front_horizon_thres_m = rospy.get_param(
            self.node_name + "/on_spline_front_horizon_thres_m"
        )
        self.on_spline_min_dist_thres_m = rospy.get_param(
            self.node_name + "/on_spline_min_dist_thres_m"
        )
        self.hyst_timer_sec = rospy.get_param(self.node_name + "/hyst_timer_sec")
        self.killing_timer_sec = rospy.get_param(self.node_name + "/killing_timer_sec")

    def initialize_traj(self, wpnt):
        if len(wpnt.wpnts) != 0:
            self.stamp = wpnt.header.stamp
            self.list = wpnt.wpnts
            self.array = np.array(
                [[wpnt.x_m, wpnt.y_m, wpnt.s_m, wpnt.d_m] for wpnt in wpnt.wpnts]
            )
            self.is_init = True

    # =========================================================================
    # Test-only entry point
    # =========================================================================

    @classmethod
    def for_test(cls, planner_name="test_planner", is_closed=True, **param_overrides):
        """ROS 의존성 없이 인스턴스 생성. pytest용.

        __init__ 우회로 ROS Subscriber / get_param 호출을 피한다. 모든 동적 파라미터는
        합리적 default 로 채우고, **param_overrides 로 개별 덮어쓰기 가능.

        예:
            wd = WaypointData.for_test(
                planner_name="dynamic_avoidance_planner",
                max_horizon=20.0,
                lateral_width_m=0.5,
            )
        """
        obj = cls.__new__(cls)  # __init__ 우회 (ROS 호출 없음)
        # 인스턴스 식별자
        obj.name = planner_name
        obj.node_name = "/dyn_planners_statemachine/" + planner_name
        # wpnt 데이터 (비어있는 상태로 시작)
        obj.list = []
        obj.array = None
        obj.stamp = None
        obj.is_init = False
        # 트랙 메타
        obj.is_gb_track_wpnts = False
        obj.is_ot_wpnts = False
        obj.is_closed = is_closed
        # 결과 캐시
        obj.closest_target = None
        obj.closest_gap = None
        # 안전 계수
        obj.vel_planner_safety_factor = 1.0
        obj.from_mpc = False
        # ROS 관련 핸들 — 테스트에서는 None
        obj.dyn_sub = None
        # 동적 파라미터 default (실제 운용값과 비슷한 합리적 값)
        obj.min_horizon = 1.0
        obj.max_horizon = 30.0
        obj.lateral_width_m = 0.4
        obj.free_scaling_reference_distance_m = 5.0
        obj.latest_threshold = 1.0
        obj.on_spline_front_horizon_thres_m = 5.0
        obj.on_spline_min_dist_thres_m = 0.5
        obj.hyst_timer_sec = 1.0
        obj.killing_timer_sec = 3.0
        # 호출자가 명시적으로 덮어쓴 값 적용
        for key, val in param_overrides.items():
            setattr(obj, key, val)
        return obj
