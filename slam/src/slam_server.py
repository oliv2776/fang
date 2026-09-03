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
import uuid
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import diagnostics

try:
    from flask import Flask, jsonify, request, Response
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
        self.mqtt_broker = self.declare_parameter('mqtt_broker', '192.168.10.108').value
        self.mqtt_port = int(self.declare_parameter('mqtt_port', 1883).value)
        self.mqtt_prefix = self.declare_parameter('mqtt_prefix', 'neato/robot').value

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
        self.start_scan_pub = self.create_publisher(Bool, '/start_scan', 10)
        self.start_explore_pub = self.create_publisher(Bool, '/start_explore', 10)
        self.return_to_dock_pub = self.create_publisher(Bool, '/return_to_dock', 10)
        self.set_home_pub = self.create_publisher(Bool, '/set_home_position', 10)
        self.dock_native_pub = self.create_publisher(Bool, '/dock_native_request', 10)
        # /cmd_vel : MÊME topic que Nav2 (voir slam_bridge.py::_on_nav2_cmd_vel,
        # déjà relié jusqu'à SetMotor côté ESP). La téléopération réutilise
        # tel quel tout le pipeline existant, rien de nouveau côté ESP.
        self.teleop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.zone_pub = self.create_publisher(String, '/clean_zone_request', 10)
        self._safety_stop = False

        # --- Zones nommées (persistées sur disque, survivent au redémarrage) ---
        self._zones_file = "/app/maps/zones.json"
        self._zones = self._load_zones()

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

    # -------------------------------------------------------- Zones nommées
    # Palette et icônes volontairement limitées à un ensemble fixe (pas de
    # champ libre non validé) - évite de stocker n'importe quelle chaîne
    # arbitraire envoyée par le frontend.
    ZONE_COLORS = {"red", "blue", "green", "amber", "purple", "teal", "pink", "gray"}
    ZONE_ICONS = {"sofa", "cooking", "bed", "bath", "door", "box", "plant", "tv", "stairs", "washing-machine"}

    def _load_zones(self) -> dict:
        try:
            with open(self._zones_file) as f:
                data = json.load(f)
            if isinstance(data, dict):
                return data
        except (FileNotFoundError, json.JSONDecodeError):
            pass
        return {}

    def _save_zones(self):
        os.makedirs(os.path.dirname(self._zones_file), exist_ok=True)
        with open(self._zones_file, 'w') as f:
            json.dump(self._zones, f, indent=2, ensure_ascii=False)

    @staticmethod
    def _validate_polygon(polygon) -> bool:
        if not isinstance(polygon, list) or len(polygon) < 3:
            return False
        try:
            [(float(p[0]), float(p[1])) for p in polygon]
        except (TypeError, ValueError, IndexError):
            return False
        return True

    def _publish_zone_polygon(self, polygon):
        msg = String()
        msg.data = json.dumps({"polygon": polygon})
        self.zone_pub.publish(msg)

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

        @app.route('/api/scan/start', methods=['POST'])
        def start_scan():
            # Reparcourt la zone déjà connue de la carte SANS activer
            # brosse/aspirateur - pour mettre à jour la carte après avoir
            # déplacé des meubles. Nécessite une carte déjà existante :
            # pour une toute première carte, utilise /api/teleop.
            with self._lock:
                if self._safety_stop:
                    return jsonify({
                        "error": "safety_stop actif, vérifie le robot avant de démarrer"
                    }), 409
                if self._map is None:
                    return jsonify({
                        "error": "aucune carte existante - utilise /api/teleop pour la toute première carte"
                    }), 409
            msg = Bool()
            msg.data = True
            self.start_scan_pub.publish(msg)
            return jsonify({"status": "scan_requested"})

        @app.route('/api/scan/stop', methods=['POST'])
        def stop_scan():
            msg = Bool()
            msg.data = False
            self.start_scan_pub.publish(msg)
            return jsonify({"status": "scan_stop_requested"})

        @app.route('/api/explore/start', methods=['POST'])
        def start_explore():
            # Exploration automatique par frontières - contrairement à
            # /api/scan/start, ne nécessite PAS de carte préexistante (elle
            # se construit au fur et à mesure). Fonctionne dès qu'au moins
            # un scan a été reçu.
            with self._lock:
                if self._safety_stop:
                    return jsonify({
                        "error": "safety_stop actif, vérifie le robot avant de démarrer"
                    }), 409
            msg = Bool()
            msg.data = True
            self.start_explore_pub.publish(msg)
            return jsonify({"status": "exploration_requested"})

        @app.route('/api/explore/stop', methods=['POST'])
        def stop_explore():
            msg = Bool()
            msg.data = False
            self.start_explore_pub.publish(msg)
            return jsonify({"status": "exploration_stop_requested"})

        @app.route('/api/dock/return', methods=['POST'])
        def dock_return():
            # ⚠️ Navigation Nav2/SLAM vers la position enregistrée comme
            # "départ" - PAS le retour au socle natif du Neato (balise
            # infrarouge, alignement précis sur les contacts de charge).
            # Amène le robot près du socle, sans garantie de recharge
            # effective. Voir coverage_planner.py pour le détail.
            with self._lock:
                if self._safety_stop:
                    return jsonify({
                        "error": "safety_stop actif, vérifie le robot avant de démarrer"
                    }), 409
            msg = Bool()
            msg.data = True
            self.return_to_dock_pub.publish(msg)
            return jsonify({"status": "dock_return_requested"})

        @app.route('/api/dock/stop', methods=['POST'])
        def dock_stop():
            msg = Bool()
            msg.data = False
            self.return_to_dock_pub.publish(msg)
            return jsonify({"status": "dock_return_stop_requested"})

        @app.route('/api/dock/set_home', methods=['POST'])
        def dock_set_home():
            # Redéfinit la position "socle" à la position ACTUELLE du
            # robot - à utiliser après l'avoir replacé toi-même dessus, si
            # la capture automatique au démarrage du conteneur ne
            # correspondait pas à la vraie position du socle.
            msg = Bool()
            msg.data = True
            self.set_home_pub.publish(msg)
            return jsonify({"status": "home_position_set"})

        @app.route('/api/dock/native', methods=['POST'])
        def dock_native():
            # Retour au socle NATIF (recommandé) : utilise la balise
            # infrarouge du Neato via son mode "Clean House" - marche
            # depuis n'importe où, pas besoin d'être déjà près du socle.
            # ⚠️ Réactive brièvement brosse+aspirateur (impossible à
            # éviter, contrainte du protocole Neato lui-même - House et
            # CleaningDisable sont mutuellement exclusifs).
            with self._lock:
                if self._safety_stop:
                    return jsonify({
                        "error": "safety_stop actif, vérifie le robot avant de démarrer"
                    }), 409
            msg = Bool()
            msg.data = True
            self.dock_native_pub.publish(msg)
            return jsonify({"status": "native_dock_requested"})

        @app.route('/api/dock/native/stop', methods=['POST'])
        def dock_native_stop():
            msg = Bool()
            msg.data = False
            self.dock_native_pub.publish(msg)
            return jsonify({"status": "native_dock_stop_requested"})

        @app.route('/api/teleop', methods=['POST'])
        def teleop():
            # Téléopération manuelle - pour construire la toute première
            # carte (pas encore de zone connue à parcourir automatiquement).
            # Vitesses bridées bas, mêmes raisons que partout ailleurs dans
            # ce projet (débit capteur lent, ~1-2s entre deux vérifications
            # sécurité). La sécurité (chocs/vide) reste PLEINEMENT active
            # pendant la téléopération - contrairement au mode "pause" de
            # calibrate.py, ceci passe par le round-robin normal côté ESP.
            MAX_LINEAR = 0.12   # m/s
            MAX_ANGULAR = 0.5   # rad/s

            with self._lock:
                if self._safety_stop:
                    return jsonify({
                        "error": "safety_stop actif, vérifie le robot avant de continuer"
                    }), 409

            body = request.get_json(silent=True) or {}
            try:
                linear_x = float(body.get("linear_x", 0.0))
                angular_z = float(body.get("angular_z", 0.0))
            except (TypeError, ValueError):
                return jsonify({"error": "linear_x/angular_z doivent être numériques"}), 400

            linear_x = max(-MAX_LINEAR, min(MAX_LINEAR, linear_x))
            angular_z = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular_z))

            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            self.teleop_pub.publish(twist)
            return jsonify({"status": "ok", "linear_x": linear_x, "angular_z": angular_z})

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
            if not self._validate_polygon(polygon):
                return jsonify({"error": "polygon doit contenir au moins 3 points [x,y] numériques"}), 400

            self._publish_zone_polygon(polygon)
            return jsonify({"status": "zone_cleaning_requested", "points": len(polygon)})

        @app.route('/api/zones')
        def list_zones():
            with self._lock:
                return jsonify(list(self._zones.values()))

        @app.route('/api/zones', methods=['POST'])
        def save_zone():
            body = request.get_json(silent=True)
            if not body:
                return jsonify({"error": "corps JSON attendu"}), 400

            name = str(body.get("name", "")).strip()
            polygon = body.get("polygon")
            color = body.get("color", "gray")
            icon = body.get("icon", "box")

            if not name:
                return jsonify({"error": "'name' requis"}), 400
            if not self._validate_polygon(polygon):
                return jsonify({"error": "polygon doit contenir au moins 3 points [x,y] numériques"}), 400
            if color not in self.ZONE_COLORS:
                return jsonify({"error": f"color doit être l'une de: {sorted(self.ZONE_COLORS)}"}), 400
            if icon not in self.ZONE_ICONS:
                return jsonify({"error": f"icon doit être l'une de: {sorted(self.ZONE_ICONS)}"}), 400

            zone_id = str(uuid.uuid4())[:8]
            zone = {
                "id": zone_id,
                "name": name,
                "polygon": polygon,
                "color": color,
                "icon": icon,
                "created_at": time.time(),
            }
            with self._lock:
                self._zones[zone_id] = zone
                self._save_zones()
            return jsonify(zone), 201

        @app.route('/api/zones/<zone_id>', methods=['DELETE'])
        def delete_zone(zone_id):
            with self._lock:
                if zone_id not in self._zones:
                    return jsonify({"error": "zone inconnue"}), 404
                del self._zones[zone_id]
                self._save_zones()
            return jsonify({"status": "deleted"})

        @app.route('/api/clean/zone/<zone_id>', methods=['POST'])
        def clean_named_zone(zone_id):
            with self._lock:
                if self._safety_stop:
                    return jsonify({
                        "error": "safety_stop actif, vérifie le robot avant de démarrer"
                    }), 409
                zone = self._zones.get(zone_id)
            if zone is None:
                return jsonify({"error": "zone inconnue"}), 404

            self._publish_zone_polygon(zone["polygon"])
            return jsonify({"status": "zone_cleaning_requested", "zone": zone["name"]})

        @app.route('/api/diagnose/run', methods=['POST'])
        def run_diagnose():
            # Bloquant ~15s (4 commandes x 3s + connexion) - c'est voulu,
            # le navigateur/HA attend simplement la fin avant de proposer
            # le téléchargement. Ne teste JAMAIS SetMotor (voir diagnostics.py).
            fmt = request.args.get('format', 'md')
            report = diagnostics.run_diagnostics(
                self.mqtt_broker, self.mqtt_port, self.mqtt_prefix
            )
            filename_ts = time.strftime('%Y%m%d_%H%M%S')

            if fmt == 'json':
                body = json.dumps(report, indent=2, ensure_ascii=False)
                mimetype = 'application/json'
                filename = f"diagnostic_neato_{filename_ts}.json"
            else:
                body = diagnostics.report_to_markdown(report)
                mimetype = 'text/markdown'
                filename = f"diagnostic_neato_{filename_ts}.md"

            resp = Response(body, mimetype=mimetype)
            resp.headers['Content-Disposition'] = f'attachment; filename="{filename}"'
            return resp

        t = threading.Thread(
            target=lambda: app.run(host='0.0.0.0', port=self.rest_port, debug=False, threaded=True),
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
