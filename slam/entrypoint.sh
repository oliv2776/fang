#!/bin/bash
set -e

echo "=========================================="
echo "  Neato Brainslug - SLAM Container"
echo "  Robot: Neato D7 (gen3, LiDAR 360)"
echo "=========================================="
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo "MQTT_BROKER=${MQTT_BROKER:-192.168.10.126}:${MQTT_PORT:-1883}"
echo "SLAM_MODE=${SLAM_MODE:-mapping}"
echo "ROBOT_TYPE=${ROBOT_TYPE:-d7_gen3}"
echo "REST_PORT=${REST_PORT:-2000}"
echo "WS_PORT=${WS_PORT:-2001}"
echo "ROSBRIDGE_PORT=${ROSBRIDGE_PORT:-2002}"
echo "=========================================="

mkdir -p /app/maps /app/logs

source /opt/ros/humble/setup.bash

# --- 1. robot_state_publisher (TF base_link -> laser_link) ---
ros2 run robot_state_publisher robot_state_publisher \
       --ros-args --params-file /app/config/rsp_params.yaml \
       > /app/logs/rsp.log 2>&1 &
RSP_PID=$!
echo "[OK] robot_state_publisher (PID=$RSP_PID)"

# --- 2. ekf_localization (fusion odom + scan) ---
ros2 run robot_localization ekf_node \
       --ros-args --params-file /app/config/robot_params.yaml \
       > /app/logs/ekf.log 2>&1 &
EKF_PID=$!
echo "[OK] ekf_node (PID=$EKF_PID)"

# --- 3. slam_toolbox (génère /map) ---
SLAM_MODE="${SLAM_MODE:-mapping}"
if [ "$SLAM_MODE" = "localize" ]; then
    ros2 launch slam_toolbox online_localization_launch.py \
           > /app/logs/slam_toolbox.log 2>&1 &
else
    ros2 launch slam_toolbox online_async_launch.py \
           > /app/logs/slam_toolbox.log 2>&1 &
fi
SLAM_PID=$!
echo "[OK] slam_toolbox mode=$SLAM_MODE (PID=$SLAM_PID)"

# --- 4. slam_bridge (MQTT/WS -> ROS2 topics /odom, /scan) ---
python3 /app/src/slam_bridge.py \
       --ros-args \
       -p mqtt_broker:="${MQTT_BROKER:-192.168.10.126}" \
       -p mqtt_port:="${MQTT_PORT:-1883}" \
       -p mqtt_prefix:="${MQTT_PREFIX:-neato/robot}" \
       -p ws_port:="${WS_IN_PORT:-2003}" \
       > /app/logs/bridge.log 2>&1 &
BRIDGE_PID=$!
echo "[OK] slam_bridge (PID=$BRIDGE_PID)"

# --- 5. slam_server (REST + WS pour l'interface web) ---
python3 /app/src/slam_server.py \
       --ros-args \
       -p rest_port:="${REST_PORT:-2000}" \
       -p ws_port:="${WS_PORT:-2001}" \
       > /app/logs/server.log 2>&1 &
SERVER_PID=$!
echo "[OK] slam_server (PID=$SERVER_PID)"

# --- 6. rosbridge_server (WebSocket ROS2 -> JSON pour le front) ---
ros2 run rosbridge_server rosbridge_web \
       --ros-args -p port:="${ROSBRIDGE_PORT:-2002}" \
       > /app/logs/rosbridge.log 2>&1 &
ROSBRIDGE_PID=$!
echo "[OK] rosbridge_server (PID=$ROSBRIDGE_PID)"

echo ""
echo "Tous les processus lancés."
echo "  REST API        : http://0.0.0.0:${REST_PORT:-2000}"
echo "  WebSocket       : ws://0.0.0.0:${WS_PORT:-2001}"
echo "  rosbridge       : ws://0.0.0.0:${ROSBRIDGE_PORT:-2002}"
echo "  WS robot in     : ws://0.0.0.0:${WS_IN_PORT:-2003}"
echo "  Logs            : /app/logs/"
echo ""

trap "echo 'Arret...'; kill $RSP_PID $EKF_PID $SLAM_PID $BRIDGE_PID $SERVER_PID $ROSBRIDGE_PID 2>/dev/null; wait; exit 0" SIGINT SIGTERM
wait
