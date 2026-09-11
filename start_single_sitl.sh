set -e

PX4_DIR="/home/uav/sushil/de-swarm/PX4-Autopilot"
BUILD="$PX4_DIR/build/px4_sitl_default"

source "$BUILD/rootfs/gz_env.sh"

MicroXRCEAgent udp4 -p 8888 >/tmp/agent1.log 2>&1 &
sleep 2

mkdir -p "$BUILD/instance_1"
(
    cd "$BUILD/instance_1"

    PX4_UXRCE_DDS_PORT=8888 \
    PX4_SYS_AUTOSTART=4001 \
    PX4_SIM_MODEL=gz_x500 \
    PX4_GZ_WORLD=baylands \
    "$BUILD/bin/px4" -d -i 1 "$BUILD/etc"
)