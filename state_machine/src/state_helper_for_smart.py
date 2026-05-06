#!/usr/bin/env python3
"""SmartStaticChecker — Fixed Frenet 좌표계 기반 state checker.

상속 + composition 혼합 패턴:
- StateMachine 을 상속해 모든 check 함수 (`_check_close_to_raceline`, `_check_free_frenet`,
  `_check_overtaking_mode_sustainability` 등) 를 그대로 사용한다.
- 단 self.cur_s / cur_d / obstacles / cur_gb_wpnts 등 일부 attribute 만 Fixed Frenet
  데이터로 override 한다 (생성 시 + `update()` 매 iteration 에서).
- 그 외 attribute (rate_hz, n_loc_wpnts, pars, ftg_*, splini_*, ...) 는 모두
  `__getattr__` 으로 parent 에서 동적으로 fallback. parent 에 새 attribute 가 추가돼도
  자동으로 보임 — 옛 `self.__dict__.update(parent.__dict__)` 패턴이 가진 시한폭탄
  (init 후 parent attribute 추가 시 동기화 깨짐) 을 제거.

Usage:
    checker = SmartStaticChecker(state_machine)
    checker.update()                        # 매 iteration parent.update_waypoints() 가 호출
    close = checker._check_close_to_raceline()  # Fixed Frenet 기반 check
"""

import copy

import rospy
from nav_msgs.msg import Odometry

from state_machine_node import StateMachine


class SmartStaticChecker(StateMachine):
    """StateMachine 의 check 함수를 Fixed Frenet 좌표계로 재사용한다."""

    def __init__(self, parent_state_machine):
        """parent (메인 StateMachine) 를 참조로 들고 Fixed Frenet override 만 set.

        주의: 부모 `StateMachine.__init__` 은 호출하지 않는다. ROS 노드 중복 init 방지.
        대신 부모의 모든 attribute / method 는 `__getattr__` 또는 상속으로 자동 사용.
        """
        # parent 는 __getattr__ 가 무한재귀 없이 접근할 수 있도록 __dict__ 에 직접 set
        self.__dict__['parent'] = parent_state_machine

        # ── Fixed Frenet 전용 override (parent 의 GB Frenet 값을 가리지 않도록 helper 자체에 보유) ──
        self.cur_s = 0.0
        self.cur_d = 0.0
        self.cur_vs = 0.0
        self.cur_vd = 0.0

        # Fixed Frenet 으로 변환된 obstacle 리스트 (update() 가 매번 채움)
        self.obstacles = []
        self.obstacles_in_interest = []
        self.cur_obstacles_in_interest = []

        # 모드 플래그 — parent 와 분리 (Smart helper 는 자체 OT 결정 가짐)
        self.static_overtaking_mode = False

        # check 함수가 cur_gb_wpnts 로 base path 접근 → Smart helper 에선 Smart Static path
        # cur_gb_wpnts 자체를 alias 로 — update() 가 매번 갱신
        self.cur_gb_wpnts = parent_state_machine.cur_smart_static_avoidance_wpnts
        self.num_glb_wpnts = 0
        self.waypoints_dist = 0.0
        # max_s, track_length 는 update() 에서 Fixed path 길이로 설정

        # Fixed Frenet odom — parent 의 GB odom 과 분리된 토픽
        rospy.Subscriber('/car_state/odom_frenet_fixed', Odometry, self._odom_fixed_cb)
        rospy.loginfo("[SmartStaticChecker] Initialized with Fixed Frenet odom subscription")

    def __getattr__(self, name):
        """self / 클래스에 없는 attribute 는 parent 에서 자동 fallback.

        효과: parent (StateMachine) 가 가진 모든 attribute (rate_hz, pars, ftg_*,
        splini_*, recovery_wpnts, avoidance_wpnts, current_position, only_ftg_zones,
        overtake_zones, obstacles_prediction, ego_prediction, ...) 를 helper 가
        자동으로 사용 가능. parent 가 attribute 를 새로 추가해도 즉시 반영.

        주의: __getattr__ 은 __dict__ + class hierarchy 모두 miss 일 때만 호출되므로
        - cur_s / cur_d 등 helper 가 직접 set 한 attribute 는 helper 의 것 사용 (override)
        - StateMachine 의 메서드들은 클래스 attribute 로 상속됨 (이 fallback 안 거침)
        - 'parent' 자체 접근은 __dict__ hit 라 fallback 트리거 안 됨 (무한재귀 X)
        """
        # name='parent' 가 __dict__ 에서 못 찾는 비정상 상황 방어 (e.g. pickling)
        if name == 'parent':
            raise AttributeError("SmartStaticChecker has no 'parent' yet")
        parent = object.__getattribute__(self, 'parent')
        return getattr(parent, name)

    def _odom_fixed_cb(self, data):
        """Fixed Frenet odom callback — helper 자체의 cur_s/d/vs/vd override."""
        self.cur_s = data.pose.pose.position.x
        self.cur_d = data.pose.pose.position.y
        self.cur_vs = data.twist.twist.linear.x
        self.cur_vd = data.twist.twist.linear.y

    def update(self):
        """매 iteration parent.update_waypoints() 가 동기적으로 호출.

        (1) Smart Static path 메타데이터 갱신 (waypoint 수, 트랙 길이 등)
        (2) parent 의 obstacles 를 Fixed Frenet 좌표로 변환
        (3) interest_horizon_m 안의 obstacles 만 추려 obstacles_in_interest 에 저장
        """
        if len(self.parent.cur_smart_static_avoidance_wpnts.list) == 0:
            # Smart Static path 아직 없음
            self.num_glb_wpnts = 0
            self.obstacles = []
            self.obstacles_in_interest = []
            self.cur_obstacles_in_interest = []
            return

        # (1) waypoint 메타 갱신
        self.cur_gb_wpnts = self.parent.cur_smart_static_avoidance_wpnts
        self.num_glb_wpnts = len(self.cur_gb_wpnts.list)
        self.track_length = self.cur_gb_wpnts.list[-1].s_m
        self.waypoints_dist = self.track_length / self.num_glb_wpnts
        # CRITICAL: max_s 도 Fixed path 길이여야 _check_free_frenet 의 wrap-around 가 맞음
        self.max_s = self.track_length

        # (2) parent 의 obstacles → Fixed Frenet 좌표로 변환 (얕은 복사 + _fixed 필드 대체)
        self.obstacles = []
        for obs in self.parent.obstacles:
            obs_copy = copy.copy(obs)
            obs_copy.s_start = obs.s_start_fixed
            obs_copy.s_end = obs.s_end_fixed
            obs_copy.s_center = obs.s_center_fixed
            obs_copy.d_center = obs.d_center_fixed
            obs_copy.d_right = obs.d_right_fixed
            obs_copy.d_left = obs.d_left_fixed
            obs_copy.vs = obs.vs_fixed
            obs_copy.vd = obs.vd_fixed
            obs_copy.s_var = obs.s_var_fixed
            obs_copy.d_var = obs.d_var_fixed
            obs_copy.vs_var = obs.vs_var_fixed
            obs_copy.vd_var = obs.vd_var_fixed
            self.obstacles.append(obs_copy)

        # (3) interest_horizon_m 안의 obstacle 만 추출
        self._update_obstacles_in_interest()

    def _update_obstacles_in_interest(self):
        """Fixed Frenet 기반 obstacles_in_interest 필터링."""
        obstacles_in_interest = []
        for obs in self.obstacles:
            gap = (obs.s_start - self.cur_s) % self.track_length
            if gap < self.interest_horizon_m:
                obstacles_in_interest.append(obs)
        self.obstacles_in_interest = obstacles_in_interest
        self.cur_obstacles_in_interest = obstacles_in_interest
