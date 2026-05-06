#!/bin/bash
# icra2026_sh — state_machine 검증 전용 컨테이너 (SH).
# HJ 컨테이너 (icra2026) 와 별도. cache 도 분리하여 catkin build 충돌 회피.
#
# 사전 조건:
#   - 이미지 빌드 완료: `docker compose build sim_sh_x86`
#   - 환경 변수: USER, UID, GID, DISPLAY, XAUTH_LOC

IMAGE=race_stack_sim_sh
WS_ROOT=${RACE_STACK_ROOT:-$HOME/unicorn_ws/ICRA2026_HJ}
CACHE_ROOT=${SH_CACHE_ROOT:-$HOME/unicorn_ws/cache_sh}
XAUTH=${XAUTH_LOC:-$HOME/.Xauthority}

# cache 디렉터리 준비 (HJ 와 분리)
mkdir -p $CACHE_ROOT/noetic/{build,devel,logs}

# 기존 동명 컨테이너 있으면 정리
docker rm -f icra2026_sh 2>/dev/null

docker run --tty --interactive \
    --network=host \
    --env DISPLAY=$DISPLAY \
    --env USER=$USER \
    --env XAUTHORITY=/home/$USER/.Xauthority \
    --env ROS_HOSTNAME=${ROS_HOSTNAME:-localhost} \
    --volume $XAUTH:/home/$USER/.Xauthority \
    --volume /dev:/dev \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume $WS_ROOT:/home/$USER/catkin_ws/src/race_stack \
    --volume $CACHE_ROOT/noetic/build:/home/$USER/catkin_ws/build \
    --volume $CACHE_ROOT/noetic/devel:/home/$USER/catkin_ws/devel \
    --volume $CACHE_ROOT/noetic/logs:/home/$USER/catkin_ws/logs \
    --privileged \
    --name icra2026_sh \
    --entrypoint /bin/bash \
    -d \
    ${IMAGE}:latest

echo ""
echo "✅ 컨테이너 'icra2026_sh' 가 백그라운드에서 동작 중."
echo ""
echo "진입:"
echo "  docker exec -it icra2026_sh bash"
echo ""
echo "처음 한 번만 — workspace 빌드:"
echo "  docker exec -it icra2026_sh bash -c \\"
echo "    'source /opt/ros/noetic/setup.bash && cd ~/catkin_ws && catkin build'"
echo ""
echo "정리:"
echo "  docker rm -f icra2026_sh"
