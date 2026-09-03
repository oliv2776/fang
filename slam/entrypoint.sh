#!/bin/bash
# Neato Brainslug - SLAM Container entrypoint
# ROS2 Humble + slam_toolbox + bridge MQTT/WS + API REST/WS

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
echo "WS_IN_PORT=${WS_IN_PORT:-2003}"
echo "=========================================="

# On ne met PAS set -e : les processus en background peuvent mourir
# et on veut que le container continue de tourner
set +e

# S'assurer que les répertoires existent
mkdir -p /app/maps /app/logs

# Sourcing ROS2
source /opt/ros/humble/setup.bash

# Fichier de config slam_toolbox (frames odom/map/base_link, scan_topic, etc.)
# BUGFIX: ce fichier n'était jamais passé à slam_toolbox -> params par défaut du package
SLAM_PARAMS_FILE="/app/config/slam_toolbox.yaml"

# Fichier de config Nav2 (costmaps, controller, planner...)
NAV2_PARAMS_FILE="/app/config/nav2_params.yaml"

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
            slam_params_file:="${SLAM_PARAMS_FILE}" \
            > /app/logs/slam_toolbox.log 2>&1 &
else
    ros2 launch slam_toolbox online_async_launch.py \
            slam_params_file:="${SLAM_PARAMS_FILE}" \
            > /app/logs/slam_toolbox.log 2>&1 &
fi
SLAM_PID=$!
echo "[OK] slam_toolbox mode=$SLAM_MODE (PID=$SLAM_PID)"

# --- 3bis. Nav2 (navigation) ---
# NAV2_ENABLED permet de désactiver complètement cet étage (ex: tant que
# tu valides juste le mapping) sans toucher au reste du script.
if [ "${NAV2_ENABLED:-false}" = "true" ]; then
    ros2 run nav2_controller controller_server \
            --ros-args --params-file "${NAV2_PARAMS_FILE}" \
            > /app/logs/nav2_controller.log 2>&1 &
    NAV2_CONTROLLER_PID=$!
    echo "[OK] nav2 controller_server (PID=$NAV2_CONTROLLER_PID)"

    ros2 run nav2_planner planner_server \
            --ros-args --params-file "${NAV2_PARAMS_FILE}" \
            > /app/logs/nav2_planner.log 2>&1 &
    NAV2_PLANNER_PID=$!
    echo "[OK] nav2 planner_server (PID=$NAV2_PLANNER_PID)"

    ros2 run nav2_behaviors behavior_server \
            --ros-args --params-file "${NAV2_PARAMS_FILE}" \
            > /app/logs/nav2_behavior.log 2>&1 &
    NAV2_BEHAVIOR_PID=$!
    echo "[OK] nav2 behavior_server (PID=$NAV2_BEHAVIOR_PID)"

    ros2 run nav2_bt_navigator bt_navigator \
            --ros-args --params-file "${NAV2_PARAMS_FILE}" \
            > /app/logs/nav2_bt.log 2>&1 &
    NAV2_BT_PID=$!
    echo "[OK] nav2 bt_navigator (PID=$NAV2_BT_PID)"

    ros2 run nav2_waypoint_follower waypoint_follower \
            --ros-args --params-file "${NAV2_PARAMS_FILE}" \
            > /app/logs/nav2_waypoint.log 2>&1 &
    NAV2_WAYPOINT_PID=$!
    echo "[OK] nav2 waypoint_follower (PID=$NAV2_WAYPOINT_PID)"

    ros2 run nav2_lifecycle_manager lifecycle_manager \
            --ros-args --params-file "${NAV2_PARAMS_FILE}" \
            > /app/logs/nav2_lifecycle.log 2>&1 &
    NAV2_LIFECYCLE_PID=$!
    echo "[OK] nav2 lifecycle_manager (PID=$NAV2_LIFECYCLE_PID)"
fi

# --- 4. slam_bridge (MQTT/WS -> ROS2 topics /odom, /scan, /cmd_vel_manual ;
#         ROS2 /cmd_vel (Nav2) -> MQTT vers l'ESP32) ---
python3 /app/src/slam_bridge.py \
        --ros-args \
        -p mqtt_broker:="${MQTT_BROKER:-192.168.10.126}" \
        -p mqtt_port:="${MQTT_PORT:-1883}" \
        -p mqtt_prefix:="${MQTT_PREFIX:-neato/robot}" \
        -p ws_port:="${WS_IN_PORT:-2003}" \
        -p cmd_vel_out_topic:="${CMD_VEL_OUT_TOPIC:-neato/robot/cmd_vel_out}" \
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
echo "  REST API         : http://0.0.0.0:${REST_PORT:-2000}"
echo "  WebSocket        : ws://0.0.0.0:${WS_PORT:-2001}"
echo "  rosbridge        : ws://0.0.0.0:${ROSBRIDGE_PORT:-2002}"
echo "  WS robot in      : ws://0.0.0.0:${WS_IN_PORT:-2003}"
echo "  Nav2             : ${NAV2_ENABLED:-false}"
echo "  Logs             : /app/logs/"
echo ""

# Attendre la fin (Ctrl+C ou SIGTERM pour arrêter proprement)
# On utilise un loop qui vérifie que les processus sont encore vivants
# et relance ceux qui sont morts (auto-restart)
cleanup() {
    echo "Arret des processus..."
    kill $RSP_PID $EKF_PID $SLAM_PID $BRIDGE_PID $SERVER_PID $ROSBRIDGE_PID \
         $NAV2_CONTROLLER_PID $NAV2_PLANNER_PID $NAV2_BEHAVIOR_PID \
         $NAV2_BT_PID $NAV2_WAYPOINT_PID $NAV2_LIFECYCLE_PID 2>/dev/null
    wait 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

# Watchdog : relance les processus morts toutes les 10s
while true; do
    # Vérifier robot_state_publisher
    if ! kill -0 $RSP_PID 2>/dev/null; then
        echo "[WARN] robot_state_publisher mort, relance..."
        ros2 run robot_state_publisher robot_state_publisher \
                --ros-args --params-file /app/config/rsp_params.yaml \
                > /app/logs/rsp.log 2>&1 &
        RSP_PID=$!
    fi

    # Vérifier ekf_node
    if ! kill -0 $EKF_PID 2>/dev/null; then
        echo "[WARN] ekf_node mort, relance..."
        ros2 run robot_localization ekf_node \
                --ros-args --params-file /app/config/robot_params.yaml \
                > /app/logs/ekf.log 2>&1 &
        EKF_PID=$!
    fi

    # Vérifier slam_toolbox
    if ! kill -0 $SLAM_PID 2>/dev/null; then
        echo "[WARN] slam_toolbox mort, relance..."
        if [ "$SLAM_MODE" = "localize" ]; then
            ros2 launch slam_toolbox online_localization_launch.py \
                    slam_params_file:="${SLAM_PARAMS_FILE}" \
                    > /app/logs/slam_toolbox.log 2>&1 &
        else
            ros2 launch slam_toolbox online_async_launch.py \
                    slam_params_file:="${SLAM_PARAMS_FILE}" \
                    > /app/logs/slam_toolbox.log 2>&1 &
        fi
        SLAM_PID=$!
    fi

    # Vérifier slam_bridge
    if ! kill -0 $BRIDGE_PID 2>/dev/null; then
        echo "[WARN] slam_bridge mort, relance..."
        python3 /app/src/slam_bridge.py \
                --ros-args \
                -p mqtt_broker:="${MQTT_BROKER:-192.168.10.126}" \
                -p mqtt_port:="${MQTT_PORT:-1883}" \
                -p mqtt_prefix:="${MQTT_PREFIX:-neato/robot}" \
                -p ws_port:="${WS_IN_PORT:-2003}" \
                -p cmd_vel_out_topic:="${CMD_VEL_OUT_TOPIC:-neato/robot/cmd_vel_out}" \
                > /app/logs/bridge.log 2>&1 &
        BRIDGE_PID=$!
    fi

    # Vérifier slam_server
    if ! kill -0 $SERVER_PID 2>/dev/null; then
        echo "[WARN] slam_server mort, relance..."
        python3 /app/src/slam_server.py \
                --ros-args \
                -p rest_port:="${REST_PORT:-2000}" \
                -p ws_port:="${WS_PORT:-2001}" \
                > /app/logs/server.log 2>&1 &
        SERVER_PID=$!
    fi

    # Vérifier rosbridge
    if ! kill -0 $ROSBRIDGE_PID 2>/dev/null; then
        echo "[WARN] rosbridge mort, relance..."
        ros2 run rosbridge_server rosbridge_web \
                --ros-args -p port:="${ROSBRIDGE_PORT:-2002}" \
                > /app/logs/rosbridge.log 2>&1 &
        ROSBRIDGE_PID=$!
    fi

    # Vérifier les processus Nav2 (uniquement si NAV2_ENABLED=true)
    if [ "${NAV2_ENABLED:-false}" = "true" ]; then
        if ! kill -0 $NAV2_CONTROLLER_PID 2>/dev/null; then
            echo "[WARN] nav2 controller_server mort, relance..."
            ros2 run nav2_controller controller_server \
                    --ros-args --params-file "${NAV2_PARAMS_FILE}" \
                    > /app/logs/nav2_controller.log 2>&1 &
            NAV2_CONTROLLER_PID=$!
        fi
        if ! kill -0 $NAV2_PLANNER_PID 2>/dev/null; then
            echo "[WARN] nav2 planner_server mort, relance..."
            ros2 run nav2_planner planner_server \
                    --ros-args --params-file "${NAV2_PARAMS_FILE}" \
                    > /app/logs/nav2_planner.log 2>&1 &
            NAV2_PLANNER_PID=$!
        fi
        if ! kill -0 $NAV2_BEHAVIOR_PID 2>/dev/null; then
            echo "[WARN] nav2 behavior_server mort, relance..."
            ros2 run nav2_behaviors behavior_server \
                    --ros-args --params-file "${NAV2_PARAMS_FILE}" \
                    > /app/logs/nav2_behavior.log 2>&1 &
            NAV2_BEHAVIOR_PID=$!
        fi
        if ! kill -0 $NAV2_BT_PID 2>/dev/null; then
            echo "[WARN] nav2 bt_navigator mort, relance..."
            ros2 run nav2_bt_navigator bt_navigator \
                    --ros-args --params-file "${NAV2_PARAMS_FILE}" \
                    > /app/logs/nav2_bt.log 2>&1 &
            NAV2_BT_PID=$!
        fi
        if ! kill -0 $NAV2_WAYPOINT_PID 2>/dev/null; then
            echo "[WARN] nav2 waypoint_follower mort, relance..."
            ros2 run nav2_waypoint_follower waypoint_follower \
                    --ros-args --params-file "${NAV2_PARAMS_FILE}" \
                    > /app/logs/nav2_waypoint.log 2>&1 &
            NAV2_WAYPOINT_PID=$!
        fi
        if ! kill -0 $NAV2_LIFECYCLE_PID 2>/dev/null; then
            echo "[WARN] nav2 lifecycle_manager mort, relance..."
            ros2 run nav2_lifecycle_manager lifecycle_manager \
                    --ros-args --params-file "${NAV2_PARAMS_FILE}" \
                    > /app/logs/nav2_lifecycle.log 2>&1 &
            NAV2_LIFECYCLE_PID=$!
        fi
    fi

    sleep 10
done
