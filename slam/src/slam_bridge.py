#!/usr/bin/env python3
"""
slam_bridge.py
Reçoit les données du robot Neato D7 gen3 (via MQTT ou WebSocket) et les publie
en topics ROS2 pour slam_toolbox.

Le D7 gen3 a un LiDAR 360° (RPLiDAR A1 ou équivalent) :
  - ~720 points par scan (résolution 0.5°)
  - Range : 0.1m à 8m
  - Fréquence : ~10-15 Hz

Topics MQTT attendus (payload JSON) :
  neato/robot/odom      -> {"x": float, "y": float, "theta": float, "timestamp": float}
  neato/robot/ranges    -> {"ranges": [float,...], "angle_min": float, "angle_max": float,
                             "angle_increment": float, "timestamp": float}
  neato/robot/cmd_vel   -> {"linear_x": float, "angular_z": float}

Topics ROS2 publiés :
   /odom         -> nav_msgs/Odometry
   /scan         -> sensor_msgs/LaserScan
   /cmd_vel      -> geometry_msgs/Twist
  TF: odom -> base_link
"""

import json
import math
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


class SlamBridge(Node):
    def __init__(self):
        super().__init__('slam_bridge')

        # --- Déclarations de paramètres ---
        self.mqtt_broker = self.declare_parameter('mqtt_broker', '192.168.10.126').value
        self.mqtt_port = int(self.declare_parameter('mqtt_port', 1883).value)
        self.mqtt_prefix = self.declare_parameter('mqtt_prefix', 'neato/robot').value
        self.ws_port = int(self.declare_parameter('ws_port', 2003).value)

        # --- Publishers ROS2 ---
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- État ---
        self._odom_lock = threading.Lock()
        self._last_odom = None

        # --- MQTT ---
        self._mqtt_client = None
        if HAS_MQTT:
            self._init_mqtt()

        # --- WebSocket (pour les clients qui préfèrent WS) ---
        if HAS_WS:
            self._init_websocket()

        self.get_logger().info(
            f"slam_bridge initialisé | MQTT={self.mqtt_broker}:{self.mqtt_port} "
            f"WS=:{self.ws_port} prefix={self.mqtt_prefix}"
        )

    # ------------------------------------------------------------------ MQTT
    def _init_mqtt(self):
        # paho-mqtt 2.x : il faut préciser la version de l'API de callback
        try:
            self._mqtt_client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1
            )
        except AttributeError:
            # paho-mqtt < 2.0 : API classique
            self._mqtt_client = mqtt.Client()

        self._mqtt_client.on_connect = self._on_mqtt_connect
        self._mqtt_client.on_message = self._on_mqtt_message

        try:
            self._mqtt_client.connect(self.mqtt_broker, self.mqtt_port, keepalive=60)
            self._mqtt_client.loop_start()
            self.get_logger().info(
                f"MQTT connecté à {self.mqtt_broker}:{self.mqtt_port}"
            )
        except Exception as e:
            self.get_logger().error(f"MQTT connect failed: {e}")

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        prefix = self.mqtt_prefix
        client.subscribe(f"{prefix}/odom", qos=1)
        client.subscribe(f"{prefix}/ranges", qos=1)
        client.subscribe(f"{prefix}/cmd_vel", qos=1)
        self.get_logger().info(f"MQTT subscribed to {prefix}/#")

    def _on_mqtt_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return

        topic = msg.topic
        if topic.endswith('/odom'):
            self._handle_odom(data)
        elif topic.endswith('/ranges'):
            self._handle_ranges(data)
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
        async def handler(websocket, path=None):
            self.get_logger().info("WS client connecté")
            try:
                async for raw in websocket:
                    try:
                        data = json.loads(raw)
                    except json.JSONDecodeError:
                        continue
                    msg_type = data.get('type', '')
                    if msg_type == 'odom':
                        self._handle_odom(data)
                    elif msg_type == 'ranges':
                        self._handle_ranges(data)
                    elif msg_type == 'cmd_vel':
                        self._handle_cmd_vel(data)
            except Exception:
                pass
            self.get_logger().info("WS client déconnecté")

        async with websockets.serve(handler, '0.0.0.0', self.ws_port):
            self.get_logger().info(f"WS server écoutant sur :{self.ws_port}")
            await asyncio.Future()

    # --------------------------------------------------------- Handlers
    def _handle_odom(self, data: dict):
        x = float(data.get('x', 0.0))
        y = float(data.get('y', 0.0))
        theta = float(data.get('theta', 0.0))
        ts = float(data.get('timestamp', time.time()))

        with self._odom_lock:
            self._last_odom = (x, y, theta, ts)

        # Publier /odom
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self._euler_to_quat(theta)

        # Covariance (valeur par défaut, à affiner selon le capteur)
        odom.pose.covariance = [
            0.001, 0, 0, 0, 0, 0,
            0, 0.001, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0.001, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
        ]
        self.odom_pub.publish(odom)

        # Publier TF odom -> base_link
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def _handle_ranges(self, data: dict):
        ranges = [float(r) for r in data.get('ranges', [])]
        angle_min = float(data.get('angle_min', 0.0))
        angle_max = float(data.get('angle_max', 2 * math.pi))
        angle_inc = float(data.get('angle_increment', 0.0))

        # Pour le D7 gen3 avec LiDAR 360° :
        # Si angle_increment n'est pas fourni ou est 0, on le calcule
        # à partir du nombre de points
        if angle_inc <= 0.0 and len(ranges) > 1:
            angle_inc = (angle_max - angle_min) / len(ranges)

        # Si on a exactement 720 points (résolution 0.5°), on force
        if len(ranges) == 720 and abs(angle_inc - 0.0) < 0.001:
            angle_inc = (2 * math.pi) / 720.0

        # Si on a 360 points (résolution 1°), on force
        if len(ranges) == 360 and abs(angle_inc - 0.0) < 0.001:
            angle_inc = (2 * math.pi) / 360.0

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_link'
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_inc
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = 0.1
        scan.range_max = 8.0
        scan.ranges = ranges
        self.scan_pub.publish(scan)

    def _handle_cmd_vel(self, data: dict):
        twist = Twist()
        twist.linear.x = float(data.get('linear_x', 0.0))
        twist.angular.z = float(data.get('angular_z', 0.0))
        self.cmd_vel_pub.publish(twist)

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
