#!/usr/bin/env python3
"""
slam_bridge.py
Reçoit les données du robot Neato D7 gen3 (via MQTT ou WebSocket) et les publie
en topics ROS2 pour slam_toolbox.

--------------------------------------------------------------------------
LiDAR (JSON, produit par le parsing de "GetLDSScan" sur le port debug) :
--------------------------------------------------------------------------
IMPORTANT : le hardware de ce projet n'a QU'UN SEUL fil vers l'ESP32 (le
port debug principal, cf install-esp-device-gen3.md) - pas de second fil
direct sur le module LDS. Le flux binaire brut du capteur (protocole
XV-11 : paquets 22 octets, sync 0xFA, checksum, cf recherche publique)
n'est donc PAS accessible ici. Les scans passent par la commande texte
"GetLDSScan" du port debug, comme GetCharger/GetMotor/etc. L'ancien
composant fang-custom/components/neato_lidar/ (qui supposait un second
uart dédié) NE S'APPLIQUE PAS à ce hardware et ne doit pas être inclus.

Topic MQTT : <mqtt_prefix>/scan, payload JSON (publié par le parsing de
GetLDSScan dans config/comp/gen3.yaml) :
    {
      "ranges": [float|null, ...],   # 360 valeurs, une par degré, mètres
                                       # (null = pas de retour valide)
      "angle_min": 0.0,
      "angle_max": 6.2657,            # ~359° en radians
      "angle_increment": 0.017453,    # 1° en radians
      "timestamp": float
    }

ATTENTION - débit limité : GetMotor et GetLDSScan partagent le même bus
UART (un seul fil) et sont envoyés en alternance stricte côté ESPHome
(cf config/comp/slam-odom.yaml). Sur ce hardware, attends-toi à un scan
et une mise à jour d'odométrie environ toutes les 1-2 secondes, pas du
10Hz. C'est une limite physique du montage à un seul fil, pas un bug du
bridge : slam_toolbox fonctionnera, mais avec une carte qui se construit
plus lentement et une intégration d'odométrie plus sensible aux dérives
entre deux échantillons (le robot doit rester lent pendant le mapping).

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
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, TransformStamped
from std_msgs.msg import Bool

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


# --- Constantes du scan LiDAR (doivent matcher le JSON publié par
#     config/comp/gen3.yaml, branche "GetLDSScan") ---
LIDAR_POINTS_PER_SCAN = 360  # une mesure par degré (résolution du GetLDSScan texte)
RANGE_MIN_M = 0.1
RANGE_MAX_M = 8.0


class SlamBridge(Node):
    def __init__(self):
        super().__init__('slam_bridge')

        # --- Déclarations de paramètres ---
        self.mqtt_broker = self.declare_parameter('mqtt_broker', '192.168.10.126').value
        self.mqtt_port = int(self.declare_parameter('mqtt_port', 1883).value)
        self.mqtt_prefix = self.declare_parameter('mqtt_prefix', 'neato/robot').value
        self.ws_port = int(self.declare_parameter('ws_port', 2003).value)

        # Topic MQTT vers lequel republier les commandes de vitesse produites
        # par Nav2 (ROS2 topic /cmd_vel, standard) à destination de l'ESP32.
        # Voir config/comp/slam-odom.yaml côté ESPHome pour la traduction en
        # SetMotor.
        self.cmd_vel_out_topic = self.declare_parameter(
            'cmd_vel_out_topic', 'neato/robot/cmd_vel_out'
        ).value

        # Empattement (distance entre les deux roues motrices), en mètres.
        # Valeur par défaut approximative pour un Neato D-series (~0.248 m,
        # cf. drivers Neato ROS1 open-source) : À VÉRIFIER / mesurer sur ton D7,
        # une erreur ici fausse directement l'estimation de rotation (theta).
        self.wheel_base_m = float(self.declare_parameter('wheel_base_m', 0.248).value)

        # --- Publishers ROS2 ---
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        # /cmd_vel_manual : chemin MQTT->ROS2 existant (télécommande manuelle
        # via une app, par ex.). Volontairement PAS nommé /cmd_vel : ce nom
        # est réservé à la sortie de Nav2 (controller_server), voir
        # _on_nav2_cmd_vel ci-dessous. Publier les deux sur le même topic
        # ferait interférer manuel et autonome sans arbitrage.
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # /safety_stop : republie l'état de sécurité lu sur l'ESP32 (chocs,
        # roue soulevée, vide détecté) pour que coverage_planner.py (et
        # tout autre nœud Nav2) puisse s'arrêter/annuler ses objectifs.
        # NOTE : l'arrêt PHYSIQUE réel se fait déjà côté ESP32
        # (config/comp/gen3.yaml + slam-odom.yaml), indépendamment de ce
        # topic - ce publisher sert à informer le reste de la stack, pas
        # à garantir la sécurité elle-même (qui doit rester locale à l'ESP).
        self.safety_pub = self.create_publisher(Bool, '/safety_stop', 10)
        self._last_safety_stop = False

        # --- Subscriber ROS2 : /cmd_vel produit par Nav2 -> MQTT vers l'ESP ---
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._on_nav2_cmd_vel, 10
        )

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
            f"wheel_base={self.wheel_base_m}m cmd_vel_out={self.cmd_vel_out_topic}"
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
            # Tout est en JSON, y compris le scan (voir config/comp/gen3.yaml,
            # branche GetLDSScan, et config/comp/slam-odom.yaml pour le topic).
            client.subscribe(f"{prefix}/odom", qos=1)
            client.subscribe(f"{prefix}/wheels", qos=1)
            client.subscribe(f"{prefix}/cmd_vel", qos=1)
            client.subscribe(f"{prefix}/scan", qos=0)
            client.subscribe(f"{prefix}/safety", qos=1)
            self.get_logger().info(
                f"MQTT subscribed to {prefix}/{{odom,wheels,cmd_vel,scan}}"
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

        # --- Tout est en JSON (scan inclus, depuis la découverte que le
        #     hardware n'a qu'un seul fil vers le port debug) ---
        try:
            data = json.loads(msg.payload.decode('utf-8'))
        except (json.JSONDecodeError, UnicodeDecodeError):
            self.get_logger().warn(f"Payload JSON invalide sur {topic}, ignoré")
            return

        if topic.endswith('/scan'):
            self._handle_scan_json(data)
        elif topic.endswith('/wheels'):
            self._handle_wheels(data)
        elif topic.endswith('/odom'):
            self._handle_odom_precomputed(data)
        elif topic.endswith('/cmd_vel'):
            self._handle_cmd_vel(data)
        elif topic.endswith('/safety'):
            self._handle_safety(data)

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
                    if isinstance(raw, (bytes, bytearray)):
                        # Plus de flux binaire sur ce hardware (un seul fil,
                        # cf en-tête du fichier) : on ignore silencieusement.
                        continue
                    try:
                        data = json.loads(raw)
                    except json.JSONDecodeError:
                        continue
                    msg_type = data.get('type', '')
                    if msg_type == 'scan':
                        self._handle_scan_json(data)
                    elif msg_type == 'wheels':
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

    # --------------------------------------------------------- LiDAR (JSON)
    def _handle_scan_json(self, data: dict):
        """Parse le JSON publié par la branche GetLDSScan de gen3.yaml :
        {"ranges": [float|null, ...] (360), "angle_min":.., "angle_max":..,
         "angle_increment":.., "timestamp":..}"""
        raw_ranges = data.get('ranges')
        if not isinstance(raw_ranges, list):
            self.get_logger().warn("Message /scan sans champ 'ranges' valide, ignoré")
            return

        ranges = []
        for r in raw_ranges:
            if r is None:
                ranges.append(math.inf)
                continue
            try:
                v = float(r)
            except (TypeError, ValueError):
                ranges.append(math.inf)
                continue
            if v < RANGE_MIN_M or v > RANGE_MAX_M:
                ranges.append(math.inf)
            else:
                ranges.append(v)

        n = len(ranges)
        if n == 0:
            return

        angle_min = float(data.get('angle_min', 0.0))
        angle_max = float(data.get('angle_max', 2.0 * math.pi * (n - 1) / n))
        angle_increment = float(data.get('angle_increment', 2.0 * math.pi / n))

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_link'
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = RANGE_MIN_M
        scan.range_max = RANGE_MAX_M
        scan.ranges = ranges
        # GetLDSScan ne fournit pas d'intensité exploitable ici (l'intensité
        # brute XV-11 n'est de toute façon pas accessible sur ce hardware,
        # cf en-tête du fichier) : on laisse intensities vide, slam_toolbox
        # ne s'en sert pas pour le scan matching.
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

    def _handle_safety(self, data: dict):
        """Reçoit le statut de sécurité publié par l'ESP32 (choc, roue
        soulevée, vide détecté) et le republie en ROS2. L'arrêt physique
        réel a déjà eu lieu côté ESP au moment où ce message arrive -
        c'est purement informatif/pour annulation de trajectoire ici."""
        stop = bool(data.get('stop', False))
        if stop and not self._last_safety_stop:
            self.get_logger().error(
                f"ARRÊT DE SÉCURITÉ déclenché sur le robot : "
                f"bump={data.get('bump')} cliff={data.get('cliff')} "
                f"wheel_extended={data.get('wheel_extended')} — "
                f"vérifie le robot avant tout réarmement."
            )
        elif not stop and self._last_safety_stop:
            self.get_logger().info("Statut de sécurité redevenu OK côté ESP.")
        self._last_safety_stop = stop

        msg = Bool()
        msg.data = stop
        self.safety_pub.publish(msg)

    def _on_nav2_cmd_vel(self, msg: Twist):
        """Reçoit les commandes de vitesse produites par Nav2
        (controller_server, topic standard /cmd_vel) et les republie en
        MQTT pour que l'ESP32 les traduise en commandes SetMotor.
        Voir config/comp/slam-odom.yaml pour la réception côté ESP et la
        logique de sécurité (arrêt automatique si plus aucun message ne
        vient d'ici pendant cmd_vel_stale_timeout)."""
        if self._mqtt_client is None or not self._mqtt_connected:
            return
        payload = json.dumps({
            "linear_x": msg.linear.x,
            "angular_z": msg.angular.z,
            "timestamp": time.time(),
        })
        try:
            self._mqtt_client.publish(self.cmd_vel_out_topic, payload, qos=0, retain=False)
        except Exception as e:
            self.get_logger().warn(f"Échec publish cmd_vel_out: {e}")

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
