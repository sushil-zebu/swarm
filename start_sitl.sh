set -e

PX4_DIR="/home/uav/sushil/de-swarm/PX4-Autopilot"
BUILD="$PX4_DIR/build/px4_sitl_default"

source "$BUILD/rootfs/gz_env.sh"

wait_for_gazebo_vehicle() {
    local model_name="$1"
    local motor_topic="/$model_name/command/motor_speed"

    for _ in $(seq 1 30); do
        if gz model --list 2>/dev/null | grep -Fxq "$model_name" &&
           gz topic -l 2>/dev/null | grep -Fxq "$motor_topic"; then
            echo "Ready: $model_name (motor topic $motor_topic)"
            return 0
        fi
        sleep 1
    done

    echo "ERROR: Gazebo/PX4 bridge for $model_name did not become ready."
    echo "Expected motor topic: $motor_topic"
    return 1
}

MicroXRCEAgent udp4 -p 8888 >/tmp/agent1.log 2>&1 &
MicroXRCEAgent udp4 -p 8889 >/tmp/agent2.log 2>&1 &
MicroXRCEAgent udp4 -p 8890 >/tmp/agent3.log 2>&1 &
MicroXRCEAgent udp4 -p 8891 >/tmp/agent4.log 2>&1 &

rm -f "$BUILD/rootfs/parameters"*.bson
rm -rf "$BUILD"/instance_{1,2,3,4}

mkdir -p "$BUILD/instance_1"
(
cd "$BUILD/instance_1"

PX4_UXRCE_DDS_PORT=8888 \
PX4_UXRCE_DDS_NS=drone_1 \
PX4_SYS_AUTOSTART=4001 \
PX4_SIM_MODEL=gz_x500 \
"$BUILD/bin/px4" -d -i 1 "$BUILD/etc"
)

wait_for_gazebo_vehicle "x500_1"

mkdir -p "$BUILD/instance_2"
(
cd "$BUILD/instance_2"

PX4_UXRCE_DDS_PORT=8889 \
PX4_UXRCE_DDS_NS=drone_2 \
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="0,7,0,0,0,0" \
PX4_SIM_MODEL=gz_x500 \
"$BUILD/bin/px4" -d -i 2 "$BUILD/etc"
)

wait_for_gazebo_vehicle "x500_2"

mkdir -p "$BUILD/instance_3"
(
cd "$BUILD/instance_3"

PX4_UXRCE_DDS_PORT=8890 \
PX4_UXRCE_DDS_NS=drone_3 \
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="0,14,0,0,0,0" \
PX4_SIM_MODEL=gz_x500 \
"$BUILD/bin/px4" -d -i 3 "$BUILD/etc"
)

wait_for_gazebo_vehicle "x500_3"

mkdir -p "$BUILD/instance_4"
(
    cd "$BUILD/instance_4"

    PX4_UXRCE_DDS_PORT=8891 \
    PX4_UXRCE_DDS_NS=drone_4 \
    PX4_GZ_STANDALONE=1 \
    PX4_SYS_AUTOSTART=4001 \
    PX4_GZ_MODEL_POSE="0,21,0,0,0,0" \
    PX4_SIM_MODEL=gz_x500 \
    "$BUILD/bin/px4" -d -i 4 "$BUILD/etc"
)

wait_for_gazebo_vehicle "x500_4"

echo
echo "Swarm started: x500_1, x500_2, x500_3, and x500_4 are ready."
