"""Init mixin — state_machine 노드 초기화 헬퍼 모음.

이 mixin은 6개 헬퍼를 제공한다 — `_load_rosparams`, `_load_vehicle_dynamics`,
`_load_vel_planner_params`, `_init_state_attributes`, `_setup_ros_subscribers`,
`_setup_ros_publishers`. StateMachine `__init__` 가 이 순서로 호출하므로 의존성은
호출 순서로 보장된다 (rosparam → vehicle dynamics → vel_planner → state attrs → IO).

이 mixin이 사용하는 외부 모듈 / 본체 attribute:
    - `WaypointData`, `states`, `state_transitions` (모듈) — 본체 import 그대로 의존
    - 본체 callback 메서드들 (`odom_cb`, `glb_wpnts_cb` 등) — Subscriber 등록 시 참조
    - 본체 helper 메서드 `_apply_vel_planner_params`, `_vel_planner_3d_param_cb`
"""
import configparser
import json
import os
import threading

import rospy
from rospkg import RosPack
import trajectory_planning_helpers as tph
from dynamic_reconfigure.msg import Config
from f110_msgs.msg import (
    BehaviorStrategy,
    ObstacleArray,
    OTWpntArray,
    PredictionArray,
    WpntArray,
)
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

import states
import state_transitions
from states_types import StateType
from waypoint_data import WaypointData

# VESC msg는 sim 이 아닐 때만 필요 — import 자체는 통과해야 하므로 fallback
try:
    from vesc_msgs.msg import VescStateStamped
except ImportError:
    VescStateStamped = None


class InitMixin:
    """StateMachine `__init__` 의 책임을 6개 헬퍼로 분리한 mixin."""

    def _load_rosparams(self):
        """모든 ROS 파라미터 로드 + sectors_params 등 derived 값 계산.

        다른 헬퍼들이 self.racecar_version / self.ot_planner 등에 의존하므로 가장 먼저 호출.
        """
        # 노드 기본
        self.rate_hz = rospy.get_param("state_machine/rate")
        self.n_loc_wpnts = rospy.get_param("state_machine/n_loc_wpnts")
        self.measuring = rospy.get_param("/measure", default=False)

        # Racecar / sectors
        self.racecar_version = rospy.get_param("/racecar_version")
        self.sectors_params = rospy.get_param("/map_params")
        self.timetrials_only = rospy.get_param("state_machine/timetrials_only", False)
        self.n_sectors = self.sectors_params["n_sectors"]

        # OT sectors / planner
        self.ot_sectors_params = rospy.get_param("/ot_map_params")
        self.n_ot_sectors = self.ot_sectors_params["n_sectors"]
        self.volt_threshold = rospy.get_param("state_machine/volt_threshold", default=10)
        self.ot_planner = rospy.get_param("state_machine/ot_planner", default="predictive_spliner")

        # Waypoint dimensions
        self.gb_ego_width_m = rospy.get_param("state_machine/gb_ego_width_m")
        self.lateral_width_gb_m = rospy.get_param("state_machine/lateral_width_gb_m", 0.3)
        self.gb_horizon_m = rospy.get_param("state_machine/gb_horizon_m")
        self.interest_horizon_m = rospy.get_param("state_machine/interest_horizon_m", 20.0)

        # Spliner / overtaking
        self.use_force_trailing = not rospy.get_param("state_machine/use_force_trailing", False)
        if self.ot_planner == "spliner":
            self.splini_ttl = rospy.get_param("state_machine/splini_ttl", 2.0)
        else:
            self.splini_ttl = rospy.get_param("state_machine/pred_splini_ttl", 0.2)
        self.overtaking_horizon_m = rospy.get_param("state_machine/overtaking_horizon_m", 6.9)
        self.lateral_width_ot_m = rospy.get_param("state_machine/lateral_width_ot_m", 0.3)
        self.splini_hyst_timer_sec = rospy.get_param("state_machine/splini_hyst_timer_sec", 0.75)
        self.emergency_break_horizon = rospy.get_param("state_machine/emergency_break_horizon", 1.1)

        # Track / FTG / force GBTRACK / overtaking TTL
        self.track_length = rospy.get_param("/global_republisher/track_length")
        self.ftg_speed_mps = rospy.get_param("state_machine/ftg_speed_mps", 1.0)
        self.ftg_timer_sec = rospy.get_param("state_machine/ftg_timer_sec", 3.0)
        self.ftg_disabled = not rospy.get_param("state_machine/ftg_active", False)
        self.force_gbtrack_state = rospy.get_param("state_machine/force_GBTRACK", False)
        self.overtaking_ttl_sec = rospy.get_param("state_machine/overtaking_ttl_sec", 3.0)

    def _load_vehicle_dynamics(self):
        """racecar_f110.ini 의 차량 파라미터 + GGV / ax_max / b_ax_max csv 로드."""
        config_dir = os.path.join(
            RosPack().get_path('stack_master'), 'config', self.racecar_version
        )
        ini_path = os.path.join(config_dir, 'racecar_f110.ini')

        parser = configparser.ConfigParser()
        self.pars = {}
        if not parser.read(ini_path):
            raise ValueError('Specified config file does not exist or is empty!')
        self.pars["veh_params"] = json.loads(parser.get('GENERAL_OPTIONS', 'veh_params'))
        self.pars["vel_calc_opts"] = json.loads(parser.get('GENERAL_OPTIONS', 'vel_calc_opts'))

        veh_dyn_dir = os.path.join(config_dir, "veh_dyn_info")
        ggv_path = os.path.join(veh_dyn_dir, "ggv.csv")
        ax_max_path = os.path.join(veh_dyn_dir, "ax_max_machines.csv")
        b_ax_max_path = os.path.join(veh_dyn_dir, "b_ax_max_machines.csv")
        self.ggv, self.ax_max_machines = tph.import_veh_dyn_info.import_veh_dyn_info(
            ggv_import_path=ggv_path, ax_max_machines_import_path=ax_max_path,
        )
        _, self.b_ax_max_machines = tph.import_veh_dyn_info.import_veh_dyn_info(
            ggv_import_path=ggv_path, ax_max_machines_import_path=b_ax_max_path,
        )

    def _load_vel_planner_params(self):
        """3D vel planner 파라미터 로드 (vel_planner.yaml) + dyn_reconfigure 구독.

        (1) default 값 → (2) yaml 에서 덮어쓰기 → (3) rqt 실시간 변경 구독
        """
        import yaml as _yaml

        # (1) default
        self._h_cog = self.pars["veh_params"].get("cog_z", 0.074)
        self._slope_correction = 1.0
        self._slope_brake_margin = 0.0
        self._slope_brake_vmax = 5.0
        self._grip_scale_exp = 0.7

        # (2) yaml 에서 덮어쓰기
        yaml_path = os.path.join(
            RosPack().get_path('stack_master'), 'config', self.racecar_version, 'vel_planner.yaml'
        )
        try:
            with open(yaml_path) as f:
                params = _yaml.safe_load(f)
            self._apply_vel_planner_params(params)
            rospy.loginfo("[StateMachine3D] vel_planner.yaml loaded")
        except Exception as e:
            rospy.logwarn(f"[StateMachine3D] vel_planner.yaml not found ({e}), using defaults")

        # (3) rqt 실시간 변경 구독
        rospy.Subscriber(
            "/global_velplanner_3d/parameter_updates", Config, self._vel_planner_3d_param_cb
        )

    def _init_state_attributes(self):
        """모든 인스턴스 변수 default + WaypointData 6개 + states/state_transitions 딕셔너리.

        rosparam이 모두 로드된 후 호출 (일부 변수가 rate_hz / overtaking_ttl_sec 등에 의존).
        """
        # 노드 기본
        self.local_wpnts = WpntArray()
        self.waypoints_dist = 0.1  # [m]
        self.lock = threading.Lock()

        # FTG
        self.only_ftg_zones = []
        self.ftg_counter = 0

        # 자차 위치
        self.cur_s = 0.0
        self.cur_d = 0.0
        self.cur_vs = 0.0

        # Overtaking 상태
        self.overtake_wpnts = None
        self.overtake_zones = []
        self.ot_begin_margin = 0.5
        self.cur_volt = 11.69  # default value for sim
        self.static_overtaking_mode = False

        # Waypoint 메타 / 카운터
        self.cur_id_ot = 1
        self.max_speed = -1
        self.max_s = 0
        self.current_position = None
        self.gb_wpnts = None
        self.recovery_wpnts = None
        self.smart_static_wpnts = None  # Smart static avoidance waypoints from spliner
        self.smart_static_active = False  # Flag from spliner — is smart static mode active?
        self.gb_max_idx = None
        self.wpnt_dist = self.waypoints_dist
        self.num_glb_wpnts = 0
        self.num_ot_points = 0
        self.previous_index = 0
        self.last_recovery_update_time = None

        # WaypointData 인스턴스 6개 + 메타
        self.cur_gb_wpnts = WaypointData('global_tracking', True)
        self.cur_recovery_wpnts = WaypointData('recovery_planner', False)
        self.cur_avoidance_wpnts = WaypointData('dynamic_avoidance_planner', False)
        self.cur_static_avoidance_wpnts = WaypointData('static_avoidance_planner', False)
        self.cur_start_wpnts = WaypointData('start_planner', False)
        # smart_static은 static_avoidance_planner 파라미터를 공유하되 closed=True
        self.cur_smart_static_avoidance_wpnts = WaypointData('static_avoidance_planner', True)
        self.smart_track_length = None
        self.smart_wpnt_dist = None

        # WaypointData 속성 설정
        self.cur_avoidance_wpnts.is_ot_wpnts = True
        self.cur_static_avoidance_wpnts.is_ot_wpnts = True
        self.cur_gb_wpnts.is_gb_track_wpnts = True
        self.cur_recovery_wpnts.vel_planner_safety_factor = 0.5

        # closest target / gap (visualization 캐시)
        self.gb_closest_target = None
        self.gb_closest_gap = None
        self.recovery_closest_target = None
        self.recovery_closest_gap = None
        self.ot_closest_target = None
        self.ot_closest_gap = None

        # behavior strategy 메시지
        self.behavior_strategy = BehaviorStrategy()

        # Splines (mincurv + ot)
        self.mincurv_spline_x = None
        self.mincurv_spline_y = None
        self.ot_spline_x = None
        self.ot_spline_y = None
        self.ot_spline_d = None
        self.recompute_ot_spline = True

        # 장애물 회피 변수
        self.obstacles = []
        self.obstacles_in_interest = []
        self.cur_obstacles_in_interest = []
        self.obstacles_perception = []
        self.obstacles_prediction_id = None
        self.obstacles_prediction = []
        self.ego_prediction = []
        self.obstacle_was_here = True
        self.side_by_side_threshold = 0.6
        self.merger = None
        self.force_trailing = False

        # Spliner 변수
        self.splini_ttl_counter = int(self.splini_ttl * self.rate_hz)
        self.avoidance_wpnts = None
        self.static_avoidance_wpnts = None
        self.start_wpnts = None
        self.start_wpnts_array = None
        self.last_valid_avoidance_wpnts = None
        self.last_valid_avoidance_array = None
        self.last_valid_static_avoidance_wpnts = None
        self.emergency_break_d = 0.12  # [m]

        # Graph based + Frenet
        self.graph_based_wpts = None
        self.gb_wpnts_arr = None
        self.frenet_wpnts = WpntArray()

        # Overtaking TTL counter
        self.overtaking_ttl_count = 0
        self.overtaking_ttl_count_threshold = int(self.overtaking_ttl_sec * self.rate_hz)

        # Start trajectory 상태
        self.save_start_traj = False
        self.cur_start_wpnts_candidate = OTWpntArray()
        self.need_start_traj = False

        # Visualization 보조
        self.first_visualization = True
        self.x_viz = 0
        self.y_viz = 0

        # State 변수
        self.cur_state = StateType.GB_TRACK
        self.local_wpnts_src = StateType.GB_TRACK
        self.static_avoid = False
        self.fail_trailing = False

        # State -> wpnt 생성 함수 매핑
        self.states = {
            StateType.GB_TRACK: states.GlobalTracking,
            StateType.OVERTAKE: states.Overtaking,
            StateType.FTGONLY: states.FTGOnly,
            StateType.RECOVERY: states.RECOVERY,
            StateType.START: states.START,
            StateType.SMART_STATIC: states.SmartStatic,
        }

        # State -> 다음 state 결정 함수 매핑
        self.state_transitions = {
            StateType.GB_TRACK: state_transitions.GlobalTrackingTransition,
            StateType.RECOVERY: state_transitions.RecoveryTransition,
            StateType.TRAILING: state_transitions.TrailingTransition,
            StateType.ATTACK: state_transitions.TrailingTransition,
            StateType.OVERTAKE: state_transitions.OvertakingTransition,
            StateType.FTGONLY: state_transitions.FTGOnlyTransition,
            StateType.START: state_transitions.StartTransition,
            StateType.SMART_STATIC: state_transitions.SmartStaticTransition,
        }

    def _setup_ros_subscribers(self):
        """모든 ROS Subscriber 등록 + 필수 메시지 대기.

        ot_planner 종류에 따라 일부 토픽만 선택적으로 구독.
        """
        self.opponent = ObstacleArray()

        # Localization / global track
        rospy.Subscriber("/car_state/odom", Odometry, self.odom_cb)
        rospy.wait_for_message("/car_state/odom", Odometry)
        rospy.Subscriber("/global_waypoints_scaled", WpntArray, self.glb_wpnts_cb)
        rospy.Subscriber("/planner/recovery/wpnts", WpntArray, self.recovery_wpnts_cb)
        rospy.Subscriber("/global_waypoints/overtaking", WpntArray, self.overtake_cb)
        rospy.wait_for_message("/global_waypoints_scaled", WpntArray)
        rospy.wait_for_message("/global_waypoints/overtaking", WpntArray)
        rospy.Subscriber("/car_state/odom_frenet", Odometry, self.frenet_pose_cb)
        rospy.wait_for_message("/car_state/odom_frenet", Odometry)
        rospy.Subscriber("/global_waypoints", WpntArray, self.glb_wpnts_og_cb)

        # Dynamic reconfigure
        rospy.Subscriber("/dyn_statemachine/parameter_updates", Config, self.dyn_param_cb)
        rospy.Subscriber("/dyn_sector_tuner/speed/parameter_updates", Config, self.sector_dyn_param_cb)
        rospy.Subscriber("/dyn_sector_tuner/overtake/parameter_updates", Config, self.ot_dyn_param_cb)

        # Perception / prediction
        rospy.Subscriber("/tracking/obstacles", ObstacleArray, self.obstacle_perception_cb)
        rospy.Subscriber("/opponent_prediction/obstacles_pred", PredictionArray, self.obstacle_prediction_cb)
        rospy.Subscriber("/mpc_controller/ego_prediction", PredictionArray, self.ego_prediction_cb)

        # Planner-specific (ot_planner 에 따라)
        if self.ot_planner in ("spliner", "predictive_spliner"):
            rospy.Subscriber("/planner/avoidance/otwpnts", OTWpntArray, self.avoidance_cb)
            # Smart Static 모드 (HJ 추가)
            rospy.Subscriber("/planner/avoidance/smart_static_otwpnts", OTWpntArray, self.smart_static_avoidance_cb)
            rospy.Subscriber("/planner/avoidance/smart_static_active", Bool, self.smart_static_active_cb)
            if self.ot_planner == "predictive_spliner":
                rospy.Subscriber("/planner/avoidance/static_otwpnts", OTWpntArray, self.static_avoidance_cb)
        if self.ot_planner == "predictive_spliner":
            rospy.Subscriber("/planner/avoidance/merger", Float32MultiArray, self.merger_cb)
            rospy.Subscriber("collision_prediction/force_trailing", Bool, self.force_trailing_cb)
            rospy.Subscriber("planner/avoidance/fail_trailing", Bool, self.fail_trailing_cb)

        # 하드웨어 (sim 이 아닐 때만 — 배터리 voltage)
        if not rospy.get_param("/sim"):
            rospy.Subscriber("/vesc/sensors/core", VescStateStamped, self.vesc_state_cb)

        # Start trajectory 저장 트리거
        rospy.Subscriber("/planner/start_wpnts", OTWpntArray, self.start_wpnts_cb)
        rospy.Subscriber("/save_start_traj", Bool, self.save_start_traj_cb)

    def _setup_ros_publishers(self):
        """모든 ROS Publisher 등록."""
        self.behavior_strategy_pub = rospy.Publisher("behavior_strategy", BehaviorStrategy, queue_size=1)
        self.trailing_marker_pub = rospy.Publisher("/state_machine/trailing_target", Marker, queue_size=10)
        self.overtaking_marker_pub = rospy.Publisher("/state_machine/overtaking_target", Marker, queue_size=10)
        self.obstacles_in_interest_marker_pub = rospy.Publisher(
            "/state_machine/obstacles_in_interest", MarkerArray, queue_size=10
        )

        self.loc_wpnt_pub = rospy.Publisher("local_waypoints", WpntArray, queue_size=1)
        self.vis_loc_wpnt_pub = rospy.Publisher("local_waypoints/markers", MarkerArray, queue_size=10)
        self.vis_loc_vel_pub = rospy.Publisher("local_waypoints/vel_markers", MarkerArray, queue_size=10)
        self.state_pub = rospy.Publisher("state_machine", String, queue_size=1)
        self.state_mrk = rospy.Publisher("/state_marker", Marker, queue_size=10)
        self.state_wpnts_src_marker = rospy.Publisher("/state_wpnts_src_marker", Marker, queue_size=10)
        self.emergency_pub = rospy.Publisher("/emergency_marker", Marker, queue_size=5)  # for low voltage
        self.ot_section_check_pub = rospy.Publisher("/ot_section_check", Bool, queue_size=1)

        if self.measuring:
            self.latency_pub = rospy.Publisher("/state_machine/latency", Float32, queue_size=10)
