#!/usr/bin/env python3
"""
slam_server.py
Expose la carte SLAM et le statut du robot via REST + WebSocket.

Endpoints :
  GET     /api/status          -> {slam_running, map_size, last_update}
  GET     /api/robot/pose      -> {x, y, theta}
  GET     /api/map             -> JSON de la carte (OccupancyGrid)
  POST    /api/slam/start      -> démarre le mode mapping
  POST    /api/slam/stop       -> arrête le mapping
  POST    /api/slam/save       -> sauvegarde la carte dans /app/maps/
  WS      /ws                  -> flux temps réel (pose, map updates)

BUGFIX vs version précédente :
  - slam_toolbox ne publie pas de topic pose dédié ("/slam_toolbox/pose"
    n'existe pas) et rien dans cette stack ne publie sur "/amcl_pose"
    (pas d'AMCL) -> /api/robot/pose restait bloqué à {0,0,0}.
    La pose du robot est maintenant lue directement depuis le TF tree
    (map -> base_link), qui est la source de vérité que slam_toolbox
    met à jour en continu.
"""

import json
import math
import os
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

try:
    from flask import Flask, jsonify, request
    from flask_cors import CORS
    HAS_FLASK = True
except ImportError:
    HAS_FLASK = False

try:
    import websockets
    import asyncio
    HAS_WS = True
except ImportError:
    HAS_WS = False


class SlamServer(Node):
    def __init__(self):
        super().__init__('slam_server')

        self.rest_port = int(self.declare_parameter('rest_port', 2000).value)
        self.ws_port = int(self.declare_parameter('ws_port', 2001).value)
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.pose_poll_rate = float(self.declare_parameter('pose_poll_rate', 10.0).value)

        # --- État (protégé par _lock) ---
        self._map = None
        self._robot_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self._slam_running = True
        self._last_update = 0.0
        self._lock = threading.Lock()

        # --- WebSocket clients (protégé par _ws_lock) ---
        self._ws_clients = set()
        self._ws_lock = threading.Lock()

        # --- TF (source de la pose robot) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0 / self.pose_poll_rate, self._update_pose_from_tf)

        # --- Abonnements ROS2 ---
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.create_subscription(Bool, '/safety_stop', self._on_safety_stop, 10)
        self.start_cleaning_pub = self.create_publisher(Bool, '/start_cleaning', 10)
        self.zone_pub = self.create_publisher(String, '/clean_zone_request', 10)
        self._safety_stop = False

        # --- Démarrer les serveurs ---
        if HAS_FLASK:
            self._start_flask()
        if HAS_WS:
            self._start_ws()

        self.get_logger().info(
            f"slam_server initialisé | REST=:{self.rest_port} WS=:{self.ws_port} "
            f"| pose via TF {self.map_frame}->{self.base_frame}"
        )

    # --------------------------------------------------------- ROS callbacks
    def _on_map(self, msg: OccupancyGrid):
        with self._lock:
            self._map = msg
            self._last_update = time.time()
        self._broadcast({"type": "map", "data": self._map_to_json(msg)})

    def _on_safety_stop(self, msg: Bool):
        with self._lock:
            self._safety_stop = msg.data
        self._broadcast({"type": "safety_stop", "data": {"stop": msg.data}})

    def _update_pose_from_tf(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            # Normal au tout début, tant que slam_toolbox n'a pas encore
            # publié la transform map->odom (pas d'erreur à logguer en boucle)
            return

        q = t.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        new_pose = {
            "x": t.transform.translation.x,
            "y": t.transform.translation.y,
            "theta": yaw,
        }
        with self._lock:
            self._robot_pose = new_pose
        self._broadcast({"type": "pose", "data": new_pose})

    @staticmethod
    def _map_to_json(msg: OccupancyGrid) -> dict:
        return {
            "resolution": msg.info.resolution,
            "width": msg.info.width,
            "height": msg.info.height,
            "origin": {
                "x": msg.info.origin.position.x,
                "y": msg.info.origin.position.y,
                "theta": msg.info.origin.orientation.z,
            },
            "data": list(msg.data),
        }

    # --------------------------------------------------------- Flask REST
    def _start_flask(self):
        app = Flask(__name__)
        CORS(app)

        @app.route('/api/status')
        def status():
            with self._lock:
                map_size = None
                if self._map is not None:
                    map_size = [self._map.info.width, self._map.info.height]
                return jsonify({
                    "slam_running": self._slam_running,
                    "map_size": map_size,
                    "last_update": self._last_update,
                })

        @app.route('/api/robot/pose')
        def pose():
            with self._lock:
                return jsonify(self._robot_pose)

        @app.route('/api/map')
        def get_map():
            with self._lock:
                if self._map is None:
                    return jsonify({"error": "no map yet"}), 404
                return jsonify(self._map_to_json(self._map))

        @app.route('/api/slam/start', methods=['POST'])
        def start_slam():
            with self._lock:
                self._slam_running = True
            return jsonify({"status": "started"})

        @app.route('/api/slam/stop', methods=['POST'])
        def stop_slam():
            with self._lock:
                self._slam_running = False
            return jsonify({"status": "stopped"})

        @app.route('/api/slam/save', methods=['POST'])
        def save_map():
            with self._lock:
                if self._map is None:
                    return jsonify({"error": "no map to save"}), 404
                filename = f"/app/maps/map_{int(time.time())}.yaml"
                os.makedirs(os.path.dirname(filename), exist_ok=True)
                with open(filename, 'w') as f:
                    json.dump(self._map_to_json(self._map), f, indent=2)
            return jsonify({"status": "saved", "file": filename})

        @app.route('/api/safety')
        def safety():
            with self._lock:
                return jsonify({"stop": self._safety_stop})

        @app.route('/api/clean/start', methods=['POST'])
        def start_cleaning():
            with self._lock:
                if self._safety_stop:
                    return jsonify({
                        "error": "safety_stop actif, vérifie le robot avant de démarrer"
                    }), 409
            msg = Bool()
            msg.data = True
            self.start_cleaning_pub.publish(msg)
            return jsonify({"status": "cleaning_requested"})

        @app.route('/api/clean/stop', methods=['POST'])
        def stop_cleaning():
            msg = Bool()
            msg.data = False
            self.start_cleaning_pub.publish(msg)
            return jsonify({"status": "cleaning_stop_requested"})

        @app.route('/api/clean/zone', methods=['POST'])
        def clean_zone():
            with self._lock:
                if self._safety_stop:
                    return jsonify({
                        "error": "safety_stop actif, vérifie le robot avant de démarrer"
                    }), 409

            body = request.get_json(silent=True)
            if not body or "polygon" not in body:
                return jsonify({"error": "corps attendu: {\"polygon\": [[x,y],...]}"}), 400

            polygon = body["polygon"]
            if not isinstance(polygon, list) or len(polygon) < 3:
                return jsonify({"error": "polygon doit contenir au moins 3 points [x,y]"}), 400
            try:
                # Validation : chaque point doit être une paire de nombres.
                [(float(p[0]), float(p[1])) for p in polygon]
            except (TypeError, ValueError, IndexError):
                return jsonify({"error": "points de polygon invalides, attendu [x,y] numériques"}), 400

            msg = String()
            msg.data = json.dumps({"polygon": polygon})
            self.zone_pub.publish(msg)
            return jsonify({"status": "zone_cleaning_requested", "points": len(polygon)})

        t = threading.Thread(
            target=lambda: app.run(host='0.0.0.0', port=self.rest_port, debug=False),
            daemon=True,
        )
        t.start()
        self.get_logger().info(f"Flask REST sur :{self.rest_port}")

    # --------------------------------------------------------- WebSocket
    def _start_ws(self):
        self._ws_loop = asyncio.new_event_loop()
        t = threading.Thread(target=self._run_ws, daemon=True)
        t.start()

    def _run_ws(self):
        asyncio.set_event_loop(self._ws_loop)
        self._ws_loop.run_until_complete(self._ws_serve())

    async def _ws_serve(self):
        async def handler(websocket):
            # websockets >= 11.0 : le handler ne reçoit plus "path"
            with self._ws_lock:
                self._ws_clients.add(websocket)
            self.get_logger().info(
                f"WS client connecté ({len(self._ws_clients)})"
            )
            try:
                async for _ in websocket:
                    pass
            except Exception:
                pass
            finally:
                with self._ws_lock:
                    self._ws_clients.discard(websocket)

        try:
            async with websockets.serve(handler, '0.0.0.0', self.ws_port):
                self.get_logger().info(f"WS server sur :{self.ws_port}")
                await asyncio.Future()
        except Exception as e:
            self.get_logger().error(f"WS server error: {e}")

    def _broadcast(self, msg: dict):
        if not HAS_WS:
            return
        with self._ws_lock:
            if not self._ws_clients:
                return
            clients = list(self._ws_clients)
        payload = json.dumps(msg)
        self._ws_loop.call_soon_threadsafe(self._do_broadcast, payload, clients)

    async def _do_broadcast(self, payload: str, clients: list):
        if clients:
            await asyncio.gather(
                *[c.send(payload) for c in clients],
                return_exceptions=True,
            )


def main(args=None):
    rclpy.init(args=args)
    node = SlamServer()
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
