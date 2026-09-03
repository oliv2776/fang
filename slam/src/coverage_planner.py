#!/usr/bin/env python3
"""
coverage_planner.py
Génère un parcours de couverture (balayage en zigzag / "boustrophedon") à
partir de la carte produite par slam_toolbox, et le fait exécuter par Nav2
via l'action FollowWaypoints.

PREMIÈRE VERSION - à considérer comme un point de départ, pas une solution
peaufinée :
  - Couvre TOUTE la carte connue, pas de restriction par zone/pièce pour
    l'instant (cf INSTALL.md, roadmap).
  - Génère juste une liste de points, aucune orientation particulière n'est
    imposée entre les points (Nav2 orientera le robot vers le point suivant).
  - Pas de logique de "retour au point de charge" à la fin.

SÉCURITÉ :
  - S'abonne à /safety_stop (republié par slam_bridge.py depuis le statut
    ESP32). Si true, annule immédiatement le FollowWaypoints en cours et
    coupe la brosse/aspirateur. Ne redémarre PAS automatiquement.
  - L'arrêt physique réel du robot ne dépend PAS de ce nœud (déjà garanti
    côté ESP32) - ce nœud gère juste l'orchestration du cycle de nettoyage.

Déclenchement : s'abonne à /start_cleaning (std_msgs/Bool). True = démarre
un cycle sur la carte actuelle, False = arrête/annule le cycle en cours.
Voir slam_server.py pour les endpoints REST correspondants.
"""

import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from nav2_msgs.action import FollowWaypoints

try:
    import paho.mqtt.client as mqtt
    HAS_MQTT = True
except ImportError:
    HAS_MQTT = False


class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')

        self.robot_radius_m = float(self.declare_parameter('robot_radius_m', 0.20).value)
        # Espacement entre deux lignes de balayage. ~1.6x le rayon robot
        # donne un léger recouvrement pour ne pas laisser de bandes non
        # couvertes entre deux passages.
        self.row_spacing_m = float(self.declare_parameter('row_spacing_m', 0.32).value)
        self.min_segment_m = float(self.declare_parameter('min_segment_m', 0.25).value)

        self.mqtt_broker = self.declare_parameter('mqtt_broker', '192.168.10.126').value
        self.mqtt_port = int(self.declare_parameter('mqtt_port', 1883).value)
        self.mqtt_clean_cmd_topic = self.declare_parameter(
            'mqtt_clean_cmd_topic', 'neato/robot/clean_cmd'
        ).value

        self._map = None
        self._map_lock = threading.Lock()
        self._safety_stop = False
        self._active_goal_handle = None
        self._running = False

        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.create_subscription(Bool, '/safety_stop', self._on_safety_stop, 10)
        self.create_subscription(Bool, '/start_cleaning', self._on_start_cleaning, 10)

        self._waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self._mqtt_client = None
        if HAS_MQTT:
            self._init_mqtt()
        else:
            self.get_logger().error("paho-mqtt non installé, impossible de piloter brosse/aspirateur")

        self.get_logger().info(
            f"coverage_planner initialisé | robot_radius={self.robot_radius_m}m "
            f"row_spacing={self.row_spacing_m}m"
        )

    # ------------------------------------------------------------------ MQTT
    def _init_mqtt(self):
        try:
            self._mqtt_client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION1)
        except AttributeError:
            self._mqtt_client = mqtt.Client()
        try:
            self._mqtt_client.connect(self.mqtt_broker, self.mqtt_port, keepalive=60)
            self._mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT connect failed (coverage_planner): {e}")

    def _publish_clean_cmd(self, on: bool):
        if self._mqtt_client is None:
            return
        try:
            self._mqtt_client.publish(
                self.mqtt_clean_cmd_topic, "clean_on" if on else "clean_off", qos=1
            )
        except Exception as e:
            self.get_logger().warn(f"Échec publish clean_cmd: {e}")

    # --------------------------------------------------------------- ROS callbacks
    def _on_map(self, msg: OccupancyGrid):
        with self._map_lock:
            self._map = msg

    def _on_safety_stop(self, msg: Bool):
        was_stopped = self._safety_stop
        self._safety_stop = msg.data
        if msg.data and not was_stopped and self._running:
            self.get_logger().error(
                "safety_stop reçu pendant un cycle de nettoyage -> annulation immédiate"
            )
            self._abort_cleaning()

    def _on_start_cleaning(self, msg: Bool):
        if msg.data:
            if self._safety_stop:
                self.get_logger().error(
                    "Démarrage refusé : safety_stop actif. Vérifie le robot et "
                    "réarme-le (mqtt_clean_cmd_topic: clear_safety_stop) avant de relancer."
                )
                return
            self._start_cleaning()
        else:
            self._abort_cleaning()

    # ------------------------------------------------------------- Cycle nettoyage
    def _start_cleaning(self):
        if self._running:
            self.get_logger().warn("Un cycle de nettoyage est déjà en cours, ignoré")
            return

        with self._map_lock:
            map_msg = self._map
        if map_msg is None:
            self.get_logger().error("Pas de carte disponible pour l'instant, annulé")
            return

        waypoints = self._generate_coverage_waypoints(map_msg)
        if not waypoints:
            self.get_logger().error("Aucun waypoint de couverture généré (carte vide/trop petite ?)")
            return

        self.get_logger().info(f"Cycle de nettoyage : {len(waypoints)} points de passage")

        if not self._waypoints_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action follow_waypoints indisponible (Nav2 lancé ? NAV2_ENABLED=true ?)")
            return

        goal = FollowWaypoints.Goal()
        goal.poses = waypoints

        self._running = True
        self._publish_clean_cmd(True)

        send_future = self._waypoints_client.send_goal_async(
            goal, feedback_callback=self._on_waypoints_feedback
        )
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Objectif de couverture refusé par Nav2")
            self._running = False
            self._publish_clean_cmd(False)
            return
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_waypoints_feedback(self, feedback_msg):
        if self._safety_stop:
            self._abort_cleaning()

    def _on_result(self, future):
        try:
            result = future.result().result
            missed = list(result.missed_waypoints)
            if missed:
                self.get_logger().warn(f"Cycle terminé, {len(missed)} point(s) manqué(s): {missed}")
            else:
                self.get_logger().info("Cycle de nettoyage terminé, tous les points couverts")
        except Exception as e:
            self.get_logger().warn(f"Cycle terminé avec erreur/annulation: {e}")
        finally:
            self._running = False
            self._active_goal_handle = None
            self._publish_clean_cmd(False)

    def _abort_cleaning(self):
        self._publish_clean_cmd(False)
        if self._active_goal_handle is not None:
            try:
                self._active_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"Échec annulation objectif: {e}")
        self._running = False

    # ------------------------------------------------------- Génération du parcours
    def _generate_coverage_waypoints(self, map_msg: OccupancyGrid):
        info = map_msg.info
        w, h, res = info.width, info.height, info.resolution
        if w == 0 or h == 0:
            return []

        data = np.array(map_msg.data, dtype=np.int16).reshape((h, w))
        # Convention OccupancyGrid : -1 = inconnu, 0-100 = proba occupation.
        # "Libre" = connu ET peu probable d'être occupé.
        free = (data >= 0) & (data < 50)

        margin_cells = max(1, int(round(self.robot_radius_m / res)))
        safe = self._erode_free(free, margin_cells)

        row_spacing_cells = max(1, int(round(self.row_spacing_m / res)))
        min_segment_cells = max(1, int(round(self.min_segment_m / res)))

        waypoints = []
        reverse = False
        for row in range(0, h, row_spacing_cells):
            segments = self._free_segments_in_row(safe[row, :], min_segment_cells)
            if not segments:
                continue
            if reverse:
                segments = list(reversed(segments))
            for (col_start, col_end) in segments:
                cols = (col_end, col_start) if reverse else (col_start, col_end)
                for col in cols:
                    wx = info.origin.position.x + (col + 0.5) * res
                    wy = info.origin.position.y + (row + 0.5) * res
                    waypoints.append(self._make_pose(wx, wy))
            reverse = not reverse

        return waypoints

    @staticmethod
    def _erode_free(free: np.ndarray, margin_cells: int) -> np.ndarray:
        """Un cellule libre est "sûre" seulement si toutes les cellules
        dans un rayon margin_cells autour d'elle sont aussi libres
        (évite de planifier un point trop près d'un mur/obstacle connu).
        Implémentation numpy pure (pas de dépendance scipy)."""
        h, w = free.shape
        safe = free.copy()
        r2 = margin_cells * margin_cells
        for dy in range(-margin_cells, margin_cells + 1):
            for dx in range(-margin_cells, margin_cells + 1):
                if dx * dx + dy * dy > r2:
                    continue
                if dx == 0 and dy == 0:
                    continue
                shifted = np.zeros_like(free)
                y0, y1 = max(0, dy), h + min(0, dy)
                x0, x1 = max(0, dx), w + min(0, dx)
                sy0, sy1 = max(0, -dy), h + min(0, -dy)
                sx0, sx1 = max(0, -dx), w + min(0, -dx)
                if y1 <= y0 or x1 <= x0:
                    continue
                shifted[y0:y1, x0:x1] = free[sy0:sy1, sx0:sx1]
                safe &= shifted
        return safe

    @staticmethod
    def _free_segments_in_row(row_mask: np.ndarray, min_len: int):
        """Renvoie la liste des segments (col_start, col_end) contigus de
        cellules sûres, en ignorant les segments trop courts pour être
        utiles (bruit de carte)."""
        segments = []
        in_seg = False
        start = 0
        for col, val in enumerate(row_mask):
            if val and not in_seg:
                in_seg = True
                start = col
            elif not val and in_seg:
                in_seg = False
                if col - start >= min_len:
                    segments.append((start, col - 1))
        if in_seg and len(row_mask) - start >= min_len:
            segments.append((start, len(row_mask) - 1))
        return segments

    @staticmethod
    def _make_pose(x: float, y: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
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
