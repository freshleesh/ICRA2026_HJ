#!/bin/bash
# validate_state_machine.sh — SH refactor 회귀 검증용 자동 sequencing 스크립트.
#
# 한 명령으로 fake_odom + 3d_base_system + 3d_headtohead 띄움. 노드 간 rosparam
# 순서 보장이 필요해 roslaunch include 만으로는 안 되고 sleep 으로 sequencing 한다.
# 호스트 GLIM/Gazebo 없이 (icra2026_sh 컨테이너 안에서) 실행.
#
# Usage:
#   ./validate_state_machine.sh                # GB_TRACK 만 검증
#   ./validate_state_machine.sh --obstacle     # 장애물 주입까지 (TRAILING/OVERTAKE 전이)
#   ./validate_state_machine.sh --map gazebo_wall_2 --obstacle
#
# 종료: Ctrl+C 한 번 또는 ./validate_state_machine.sh --kill

set -e

# 기본값
MAP="gazebo_wall_2"
RACECAR="SIM"
SPEED_SCALE="1.0"
INJECT_OBSTACLE=0
KILL_ONLY=0

# 인자 파싱
while [[ $# -gt 0 ]]; do
  case "$1" in
    --obstacle)   INJECT_OBSTACLE=1; shift ;;
    --map)        MAP="$2"; shift 2 ;;
    --racecar)    RACECAR="$2"; shift 2 ;;
    --speed)      SPEED_SCALE="$2"; shift 2 ;;
    --kill)       KILL_ONLY=1; shift ;;
    -h|--help)
      echo "Usage: $0 [--obstacle] [--map <name>] [--racecar <SIM>] [--speed <1.0>] [--kill]"
      exit 0 ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

# 정리만 하고 종료
if [[ "$KILL_ONLY" -eq 1 ]]; then
  echo "[validate] killing all ROS processes..."
  pkill -9 -f "python3|roscore|rosmaster|roslaunch" 2>/dev/null || true
  sleep 1
  echo "[validate] done"
  exit 0
fi

export CAR_NAME="$RACECAR"

# 0. 기존 ROS 프로세스 정리
echo "[validate] cleaning previous ROS processes..."
pkill -9 -f "python3|roscore|rosmaster|roslaunch" 2>/dev/null || true
sleep 2

# 1. roscore (background)
echo "[validate] 1/4 starting roscore..."
roscore > /tmp/validate_roscore.log 2>&1 &
sleep 3

# 2. fake_odom — base_system 의 wait_for_message 가 받을 수 있도록 먼저 띄움
echo "[validate] 2/4 starting fake_odom_publisher (map=$MAP, speed=$SPEED_SCALE)..."
rosrun stack_master fake_odom_publisher.py _map:="$MAP" _speed_scale:="$SPEED_SCALE" \
  > /tmp/validate_fake_odom.log 2>&1 &
sleep 2

# 3. base_system — frenet, sector_tuner, global_republisher 등.
#    global_republisher 가 rosparam set 까지 시간 필요해 충분히 대기.
echo "[validate] 3/4 starting 3d_base_system (sim=true, map=$MAP)..."
roslaunch stack_master 3d_base_system.launch \
  sim:=true map:="$MAP" racecar_version:="$RACECAR" \
  > /tmp/validate_base_system.log 2>&1 &
sleep 15

# 4. headtohead — state_machine 노드 등장. global_republisher 의 track_length param
#    이 set 된 후 시작해야 _load_rosparams 가 통과한다.
echo "[validate] 4/4 starting 3d_headtohead (state_machine 포함)..."
roslaunch stack_master 3d_headtohead.launch \
  dynamic_avoidance_mode:=NONE racecar_version:="$RACECAR" \
  > /tmp/validate_headtohead.log 2>&1 &
sleep 10

# 5. (옵션) obstacle 주입
if [[ "$INJECT_OBSTACLE" -eq 1 ]]; then
  echo "[validate] +obstacle injection..."
  roslaunch obstacle_publisher 3d_obstacle_publisher.launch \
    start_s:=20 speed_scaler:=0.3 \
    > /tmp/validate_obstacle.log 2>&1 &
  sleep 5
fi

echo ""
echo "[validate] ✅ all running. monitoring suggestions:"
echo "  rostopic echo /state_machine"
echo "  rostopic hz /local_waypoints"
echo "  rostopic echo /behavior_strategy"
echo ""
echo "[validate] press Ctrl+C to stop, or run: $0 --kill"
echo ""

# 자식 프로세스들 wait
wait
