#!/usr/bin/env python3
"""
slam_bridge.py
Reçoit les données du robot Neato D7 gen3 (via MQTT ou WebSocket) et les publie
en topics ROS2 pour slam_toolbox.

--------------------------------------------------------------------------
LiDAR (format BINAIRE réel publié par le composant ESPHome neato_lidar) :
--------------------------------------------------------------------------
Topic MQTT (mqtt_topic du composant neato_lidar, ex: "neato/scan" ou
"neato/<device_id>/scan") -> payload binaire :

    Offset 0-1   : magic bytes 0x4C 0x44 ("LD")
    Offset 2     : nombre de blocs de 1080 points dans ce message (normalement 1)
    Offset 3     : version du format (0x01)
    Offset 4..   : N x 1080 points x 5 octets, pour chaque point :
        [0-1] angle   uint16 LE (index 0..1079, résolution 0.33°, PAS des radians)
        [2-3] distance uint16 LE, en mm (0 = pas de retour valide)
        [4]   intensité uint8 (0..255)

Voir fang-custom/components/neato_lidar/neato_lidar.h pour la source de vérité
de ce format (constantes LIDAR_MAGIC_0/1, LIDAR_FORMAT_VERSION, etc.).

--------------------------------------------------------------------------
Odométrie (dead-reckoning différentiel à partir des encodeurs de roues) :
--------------------------------------------------------------------------
Topic MQTT : <mqtt_prefix>/wheels, payload JSON :
    {"left_mm": float, "right_mm": float, "timestamp": float}

  - left_mm / right_mm : distance CUMULÉE parcourue par chaque roue, en mm
    (valeur croissante/décroissante monotone, typiquement issue de la
    commande série Neato "GetMotors" -> champs LeftWheel_PositionInMM /
    RightWheel_PositionInMM. Ce n'est PAS encore publié par le firmware
    actuel du repo : il faut ajouter la lecture de GetMotors côté ESPHome
    et publier ce message MQTT pour que l'odométrie fonctionne.)

Le bridge calcule x/y/theta par intégration différentielle à partir des
DELTAS de left_mm/right_mm entre deux messages (donc peu importe si les
compteurs sont remis à zéro de temps en temps, tant qu'il n'y a pas de
coupure de connexion au mauvais moment).

Topic MQTT alternatif : <mqtt_prefix>/odom, payload JSON (pose déjà calculée) :
    {"x": float, "y": float, "theta": float, "timestamp": float}
  -> conservé pour compatibilité / si une autre source calcule déjà la pose.

Topic MQTT : <mqtt_prefix>/cmd_vel, payload JSON :
    {"linear_x": float, "angular_z": float}

--------------------------------------------------------------------------
Topics ROS2 publiés :
    /odom           -> nav_msgs/Odometry
    /scan           -> sensor_msgs/LaserScan
    /cmd_vel        -> geometry_msgs/Twist
    TF: odom -> base_link
"""

import json
import math
import struct
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, TransformStamped

try:
    import paho.mqtt.client as mqtt
    HAS_MQTT = True
except ImportError:
    HAS_MQTT = False

try:
    import websockets
    import asyncio
    HAS_WS = True
except ImportError:
    HAS_WS = False


# --- Constantes du protocole binaire LiDAR (doivent matcher neato_lidar.h) ---
LIDAR_MAGIC_0 = 0x4C
LIDAR_MAGIC_1 = 0x44
LIDAR_FORMAT_VERSION = 0x01
LIDAR_POINTS_PER_SCAN = 1080
LIDAR_BYTES_PER_POINT = 5  # uint16 angle + uint16 distance + uint8 intensité
LIDAR_HEADER_SIZE = 4
LIDAR_BLOCK_SIZE = LIDAR_POINTS_PER_SCAN * LIDAR_BYTES_PER_POINT  # 5400
LIDAR_POINT_STRUCT = struct.Struct('<HHB')  # angle(u16 LE), distance(u16 LE), intensity(u8)

RANGE_MIN_M = 0.1
RANGE_MAX_M = 8.0


class SlamBridge(Node):
    def __init__(self):
        super().__init__('slam_bridge')

        # --- Déclarations de paramètres ---
        self.mqtt_broker = self.declare_parameter('mqtt_broker', '192.168.10.126').value
        self.mqtt_port = int(self.declare_parameter('mqtt_port', 1883).value)
        self.mqtt_prefix = self.declare_parameter('mqtt_prefix', 'neato/robot').value
        self.lidar_mqtt_topic = self.declare_parameter('lidar_mqtt_topic', 'neato/scan').value
        self.ws_port = int(self.declare_parameter('ws_port', 2003).value)

        # Empattement (distance entre les deux roues motrices), en mètres.
        # Valeur par défaut approximative pour un Neato D-series (~0.248 m,
        # cf. drivers Neato ROS1 open-source) : À VÉRIFIER / mesurer sur ton D7,
        # une erreur ici fausse directement l'estimation de rotation (theta).
        self.wheel_base_m = float(self.declare_parameter('wheel_base_m', 0.248).value)

        # --- Publishers ROS2 ---
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- État odométrie (dead-reckoning) ---
        self._odom_lock = threading.Lock()
        self._odo_x = 0.0
        self._odo_y = 0.0
        self._odo_theta = 0.0
        self._prev_left_mm = None
        self._prev_right_mm = None
        self._prev_odom_ts = None

        # --- MQTT ---
        self._mqtt_client = None
        self._mqtt_connected = False
        self._mqtt_reconnect_timer = None
        if HAS_MQTT:
            self._init_mqtt()
        else:
            self.get_logger().error("paho-mqtt non installé, pas de source MQTT !")

        # --- WebSocket (pour les clients qui préfèrent WS) ---
        if HAS_WS:
            self._init_websocket()

        self.get_logger().info(
            f"slam_bridge initialisé | MQTT={self.mqtt_broker}:{self.mqtt_port} "
            f"WS=:{self.ws_port} prefix={self.mqtt_prefix} "
            f"lidar_topic={self.lidar_mqtt_topic} wheel_base={self.wheel_base_m}m"
        )

    # ------------------------------------------------------------------ MQTT
    def _init_mqtt(self):
        try:
            self._mqtt_client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1
            )
        except AttributeError:
            # paho-mqtt < 2.0 : API classique
            self._mqtt_client = mqtt.Client()

        self._mqtt_client.on_connect = self._on_mqtt_connect
        self._mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self._mqtt_client.on_message = self._on_mqtt_message

        self._connect_mqtt()

    def _connect_mqtt(self):
        try:
            self._mqtt_client.connect(
                self.mqtt_broker, self.mqtt_port, keepalive=60
            )
            self._mqtt_client.loop_start()
            self.get_logger().info(
                f"MQTT connecté à {self.mqtt_broker}:{self.mqtt_port}"
            )
        except Exception as e:
            self.get_logger().error(
                f"MQTT connect failed: {e}, retry in 5s..."
            )
            self._mqtt_reconnect_timer = threading.Timer(5.0, self._connect_mqtt)
            self._mqtt_reconnect_timer.daemon = True
            self._mqtt_reconnect_timer.start()

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._mqtt_connected = True
            prefix = self.mqtt_prefix
            # Données "métier" du robot, en JSON
            client.subscribe(f"{prefix}/odom", qos=1)
            client.subscribe(f"{prefix}/wheels", qos=1)
            client.subscribe(f"{prefix}/cmd_vel", qos=1)
            # Flux LiDAR binaire, topic dédié (celui configuré côté ESPHome
            # dans le composant neato_lidar : mqtt_topic:)
            client.subscribe(self.lidar_mqtt_topic, qos=0)
            self.get_logger().info(
                f"MQTT subscribed to {prefix}/{{odom,wheels,cmd_vel}} "
                f"and {self.lidar_mqtt_topic}"
            )
        else:
            self.get_logger().error(f"MQTT connect rc={rc}")

    def _on_mqtt_disconnect(self, client, userdata, rc, *args):
        self._mqtt_connected = False
        if rc != 0:
            self.get_logger().warn(
                f"MQTT déconnecté (rc={rc}), reconnexion dans 5s..."
            )
            self._mqtt_reconnect_timer = threading.Timer(5.0, self._connect_mqtt)
            self._mqtt_reconnect_timer.daemon = True
            self._mqtt_reconnect_timer.start()

    def _on_mqtt_message(self, client, userdata, msg):
        topic = msg.topic

        # --- Flux LiDAR : binaire, pas du JSON ---
        if topic == self.lidar_mqtt_topic:
            self._handle_lidar_binary(msg.payload)
            return

        # --- Tout le reste : JSON ---
        try:
            data = json.loads(msg.payload.decode('utf-8'))
        except (json.JSONDecodeError, UnicodeDecodeError):
            self.get_logger().warn(f"Payload JSON invalide sur {topic}, ignoré")
            return

        if topic.endswith('/wheels'):
            self._handle_wheels(data)
        elif topic.endswith('/odom'):
            self._handle_odom_precomputed(data)
        elif topic.endswith('/cmd_vel'):
            self._handle_cmd_vel(data)

    # ------------------------------------------------------------- WebSocket
    def _init_websocket(self):
        self._ws_loop = asyncio.new_event_loop()
        self._ws_thread = threading.Thread(target=self._run_ws, daemon=True)
        self._ws_thread.start()

    def _run_ws(self):
        asyncio.set_event_loop(self._ws_loop)
        self._ws_loop.run_until_complete(self._ws_server())

    async def _ws_server(self):
        async def handler(websocket):
            # websockets >= 11.0 : le handler ne reçoit plus "path"
            self.get_logger().info("WS client connecté")
            try:
                async for raw in websocket:
                    # Un scan LiDAR envoyé en binaire par WS suit le même
                    # format que sur MQTT.
                    if isinstance(raw, (bytes, bytearray)):
                        self._handle_lidar_binary(bytes(raw))
                        continue
                    try:
                        data = json.loads(raw)
                    except json.JSONDecodeError:
                        continue
                    msg_type = data.get('type', '')
                    if msg_type == 'wheels':
                        self._handle_wheels(data)
                    elif msg_type == 'odom':
                        self._handle_odom_precomputed(data)
                    elif msg_type == 'cmd_vel':
                        self._handle_cmd_vel(data)
            except Exception:
                pass
            finally:
                self.get_logger().info("WS client déconnecté")

        try:
            async with websockets.serve(handler, '0.0.0.0', self.ws_port):
                self.get_logger().info(f"WS server écoutant sur :{self.ws_port}")
                await asyncio.Future()
        except Exception as e:
            self.get_logger().error(f"WS server error: {e}")

    # --------------------------------------------------------- LiDAR (binaire)
    def _handle_lidar_binary(self, payload: bytes):
        if len(payload) < LIDAR_HEADER_SIZE:
            self.get_logger().warn(f"Trame LiDAR trop courte ({len(payload)} octets)")
            return

        if payload[0] != LIDAR_MAGIC_0 or payload[1] != LIDAR_MAGIC_1:
            self.get_logger().warn(
                f"Magic bytes invalides (0x{payload[0]:02X} 0x{payload[1]:02X}), "
                f"trame ignorée"
            )
            return

        block_count = payload[2]
        version = payload[3]
        if version != LIDAR_FORMAT_VERSION:
            self.get_logger().warn(
                f"Version de format LiDAR inattendue: 0x{version:02X} "
                f"(attendu 0x{LIDAR_FORMAT_VERSION:02X}), on tente quand même"
            )

        expected_len = LIDAR_HEADER_SIZE + block_count * LIDAR_BLOCK_SIZE
        if len(payload) < expected_len:
            self.get_logger().warn(
                f"Trame LiDAR tronquée: {len(payload)} octets reçus, "
                f"{expected_len} attendus pour {block_count} bloc(s)"
            )
            return

        now = self.get_clock().now().to_msg()
        offset = LIDAR_HEADER_SIZE
        for _ in range(max(block_count, 1)):
            ranges, intensities = self._parse_scan_block(payload, offset)
            offset += LIDAR_BLOCK_SIZE
            self._publish_scan(ranges, intensities, now)

    @staticmethod
    def _parse_scan_block(payload: bytes, block_offset: int):
        """Parse un bloc de 1080 points et renvoie (ranges, intensities)
        indexés par angle (donc robustes à un décalage de phase du scan)."""
        ranges = [math.inf] * LIDAR_POINTS_PER_SCAN
        intensities = [0.0] * LIDAR_POINTS_PER_SCAN

        for i in range(LIDAR_POINTS_PER_SCAN):
            point_offset = block_offset + i * LIDAR_BYTES_PER_POINT
            angle_idx, dist_mm, intensity = LIDAR_POINT_STRUCT.unpack_from(
                payload, point_offset
            )

            if angle_idx >= LIDAR_POINTS_PER_SCAN:
                # Point corrompu / hors plage, on l'ignore
                continue

            if dist_mm == 0:
                # 0 = pas de retour valide (convention Neato)
                continue

            dist_m = dist_mm / 1000.0
            if dist_m < RANGE_MIN_M or dist_m > RANGE_MAX_M:
                continue

            ranges[angle_idx] = dist_m
            intensities[angle_idx] = float(intensity)

        return ranges, intensities

    def _publish_scan(self, ranges, intensities, stamp):
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'laser_link'
        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi * (LIDAR_POINTS_PER_SCAN - 1) / LIDAR_POINTS_PER_SCAN
        scan.angle_increment = 2.0 * math.pi / LIDAR_POINTS_PER_SCAN
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = RANGE_MIN_M
        scan.range_max = RANGE_MAX_M
        scan.ranges = ranges
        scan.intensities = intensities
        self.scan_pub.publish(scan)

    # --------------------------------------------------------- Odométrie
    def _handle_wheels(self, data: dict):
        """Dead-reckoning différentiel à partir des positions cumulées
        (en mm) des deux roues motrices."""
        try:
            left_mm = float(data['left_mm'])
            right_mm = float(data['right_mm'])
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn(f"Message /wheels invalide: {data}")
            return
        ts = float(data.get('timestamp', time.time()))

        with self._odom_lock:
            if self._prev_left_mm is None:
                # Premier message : on initialise juste la référence,
                # pas de delta calculable encore.
                self._prev_left_mm = left_mm
                self._prev_right_mm = right_mm
                self._prev_odom_ts = ts
                return

            delta_left_m = (left_mm - self._prev_left_mm) / 1000.0
            delta_right_m = (right_mm - self._prev_right_mm) / 1000.0
            dt = max(ts - self._prev_odom_ts, 1e-3)

            self._prev_left_mm = left_mm
            self._prev_right_mm = right_mm
            self._prev_odom_ts = ts

            # Filet de sécurité : un saut énorme (reset compteur ESP,
            # overflow, glitch) ne doit pas téléporter la pose.
            if abs(delta_left_m) > 1.0 or abs(delta_right_m) > 1.0:
                self.get_logger().warn(
                    "Delta roue aberrant (>1m entre deux messages), "
                    "ignoré (reset de compteur côté robot ?)"
                )
                return

            d_center = (delta_left_m + delta_right_m) / 2.0
            d_theta = (delta_right_m - delta_left_m) / self.wheel_base_m

            # Intégration au point milieu (plus précis qu'un simple Euler)
            mid_theta = self._odo_theta + d_theta / 2.0
            self._odo_x += d_center * math.cos(mid_theta)
            self._odo_y += d_center * math.sin(mid_theta)
            self._odo_theta = self._normalize_angle(self._odo_theta + d_theta)

            lin_vel = d_center / dt
            ang_vel = d_theta / dt

            x, y, theta = self._odo_x, self._odo_y, self._odo_theta

        self._publish_odom(x, y, theta, lin_vel, ang_vel)

    def _handle_odom_precomputed(self, data: dict):
        """Chemin alternatif : une source externe envoie déjà x/y/theta
        calculés (conservé pour compatibilité)."""
        x = float(data.get('x', 0.0))
        y = float(data.get('y', 0.0))
        theta = float(data.get('theta', 0.0))

        with self._odom_lock:
            self._odo_x, self._odo_y, self._odo_theta = x, y, theta

        self._publish_odom(x, y, theta, 0.0, 0.0)

    def _publish_odom(self, x, y, theta, lin_vel, ang_vel):
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self._euler_to_quat(theta)
        odom.pose.covariance = [
            0.001, 0, 0, 0, 0, 0,
            0, 0.001, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0.001, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
        ]

        odom.twist.twist.linear.x = lin_vel
        odom.twist.twist.angular.z = ang_vel

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def _handle_cmd_vel(self, data: dict):
        twist = Twist()
        twist.linear.x = float(data.get('linear_x', 0.0))
        twist.angular.z = float(data.get('angular_z', 0.0))
        self.cmd_vel_pub.publish(twist)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _euler_to_quat(theta: float) -> Quaternion:
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(theta / 2.0),
            w=math.cos(theta / 2.0),
        )


def main(args=None):
    rclpy.init(args=args)
    node = SlamBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
