"""Shared pytest fixtures and environment setup for state_machine tests.

ROS 환경 없이 순수 함수만 테스트하기 위해 rospy를 mock으로 대체하고,
state_machine/src 를 import path에 추가한다.
"""
import os
import sys
from unittest.mock import MagicMock

import pytest

# ---------------------------------------------------------------------------
# ROS 의존성 mock
#   - path_checker.py / waypoint_data.py 가 import 시점에 rospy / dynamic_reconfigure 로드
#   - 호스트 환경에 ROS가 없거나 있어도 진짜 통신 일어나면 안 되므로 가짜로 대체
# ---------------------------------------------------------------------------
sys.modules.setdefault("rospy", MagicMock())
sys.modules.setdefault("dynamic_reconfigure", MagicMock())
sys.modules.setdefault("dynamic_reconfigure.msg", MagicMock())

# state_machine/src 디렉터리를 import path에 추가
_SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

# 테스트 디렉터리도 추가 (fake_msgs import 용)
_TEST_DIR = os.path.abspath(os.path.dirname(__file__))
if _TEST_DIR not in sys.path:
    sys.path.insert(0, _TEST_DIR)


# ---------------------------------------------------------------------------
# 공용 fixture
# ---------------------------------------------------------------------------

@pytest.fixture
def straight_track_wpnts():
    """0~50m 직선 트랙 (closed=True 가정)."""
    from fake_msgs import make_straight_track
    return make_straight_track(length_m=50, ds=1.0)


@pytest.fixture
def default_params():
    """일반적인 f1tenth 차량 파라미터."""
    from path_checker import FrenetCheckParams
    return FrenetCheckParams(
        max_s=200.0,    # 트랙 둘레 (충분히 큼)
        veh_length=0.5,
        ego_width=0.3,
    )


@pytest.fixture
def ego_at_10m():
    """진행거리 10m, 속도 5 m/s 인 자차."""
    from path_checker import EgoFrenetState
    return EgoFrenetState(s=10.0, vs=5.0)
