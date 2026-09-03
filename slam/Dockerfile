# Neato Brainslug - SLAM Container
# ROS2 Humble + slam_toolbox + bridge MQTT/WS + API REST/WS
# Adapté pour Neato D7 gen3 avec LiDAR 360°
# Base : Ubuntu 22.04 (jammy)

FROM ros:humble-ros-base-ubuntu-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# --- Dépendances système ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-venv \
        python3-dev \
        build-essential \
        curl \
        ca-certificates \
        ros-humble-slam-toolbox \
        ros-humble-robot-state-publisher \
        ros-humble-ros2-launch \
        ros-humble-ros2-topic \
        ros-humble-ros2-service \
        ros-humble-std-msgs \
        ros-humble-geometry-msgs \
        ros-humble-nav-msgs \
        ros-humble-map-msgs \
        ros-humble-tf2-ros \
        ros-humble-tf2-geometry-msgs \
        ros-humble-robot-localization \
        ros-humble-rosbridge-protocol \
        ros-humble-rosbridge-server \
        ros-humble-std-srvs \
        ros-humble-visualization-msgs \
        ros-humble-map-server \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-nav2-regulated-pure-pursuit-controller \
        && rm -rf /var/lib/apt/lists/*

# --- Dépendances Python ---
COPY slam/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# --- Copie du code ---
WORKDIR /app
COPY slam/src/ /app/src/
COPY slam/config/ /app/config/
COPY slam/entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

# --- Ports exposés (ignorés avec network_mode: host, mais utiles pour la doc) ---
# 2000   REST API (carte, statut, pose)
# 2001   WebSocket (flux temps réel vers l'interface web)
# 2002   rosbridge (WebSocket ROS2 -> JSON)
# 2003   WebSocket (données robot entrantes)
EXPOSE 2000 2001 2002 2003

ENTRYPOINT ["/app/entrypoint.sh"]
