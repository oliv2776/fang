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
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

try:
    import paho.mqtt.client as mqtt
    HAS_MQTT = True
except ImportError:
    HAS_MQTT = False


class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')

        self.robot_radius_m = float(self.declare_parameter('robot_radius_m', 0.168).value)
        # Espacement entre deux lignes de balayage = largeur mesurée de la
        # brosse centrale (27.5cm) x 0.85 pour un recouvrement de 15%
        # (compense l'imprécision de l'odométrie/du suivi de trajectoire,
        # évite les bandes non nettoyées entre deux passages).
        self.row_spacing_m = float(self.declare_parameter('row_spacing_m', 0.234).value)
        self.min_segment_m = float(self.declare_parameter('min_segment_m', 0.25).value)

        # --- Replanification incrémentale pendant le nettoyage ---
        # Au lieu de calculer tout le parcours une fois pour toutes, on
        # envoie de petits lots de points, et on RECALCULE à partir d'une
        # carte fraîche entre deux lots - ce qui permet de prendre en
        # compte un meuble déplacé (nouvel obstacle évité au recalcul
        # suivant, espace libéré ajouté au parcours restant).
        self.replanning_batch_size = int(self.declare_parameter('replanning_batch_size', 8).value)
        # Rayon (m) marqué "déjà couvert" autour de chaque point de passage
        # atteint - APPROXIMATIF : ne garantit pas que le sol a été
        # physiquement brossé sur toute la ligne entre deux points (surtout
        # pour les sauts entre segments/lignes différents, pas juste un
        # aller-retour en ligne droite), mais évite de revenir sans cesse
        # sur une zone déjà traversée.
        self.coverage_mark_radius_m = float(
            self.declare_parameter('coverage_mark_radius_m', self.row_spacing_m / 2).value
        )

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
        self._scan_only = False
        self._clean_polygon = None
        self._covered_points = []
        self._current_batch = []

        # --- Exploration automatique (frontières) ---
        self.min_frontier_area_m2 = float(self.declare_parameter('min_frontier_area_m2', 0.3).value)
        self.max_explore_iterations = int(self.declare_parameter('max_explore_iterations', 300).value)
        self._exploring = False
        self._explore_iterations = 0
        self._explore_blacklist = []  # frontières ayant échoué, évitées un moment
        self._explore_goal_handle = None
        self._current_explore_target = None

        # --- Retour au socle ---
        # ⚠️ Ce n'est PAS le retour au socle natif du Neato (qui utilise la
        # balise infrarouge du socle pour un alignement précis sur les
        # contacts de charge, intégré à son mode "House" natif). Ici, on
        # navigue juste jusqu'à la position enregistrée comme "départ" via
        # Nav2/SLAM - ça amène le robot PRÈS du socle, mais rien ne garantit
        # un alignement assez précis pour recharger réellement. À valider
        # au premier essai, robot supervisé.
        self._home_pose = None  # (x, y, theta), capturé automatiquement au 1er TF valide
        self._home_captured = False
        self._docking = False
        self._dock_goal_handle = None
        self.create_timer(1.0, self._maybe_capture_home_pose)

        # --- Retour au socle NATIF (recommandé) ---
        # Utilise la balise infrarouge du socle via le mode "Clean House"
        # natif du Neato (fonctionnalité v1 stable, cf status.md) - marche
        # depuis n'importe où dans le rayon de la balise, pas besoin de
        # rapprocher le robot au préalable avec Nav2. Contrainte du
        # protocole Neato : "House" et "CleaningDisable" sont mutuellement
        # exclusifs (cf research/hidden-commands.md), donc IMPOSSIBLE
        # d'obtenir la navigation native sans réactiver brièvement brosse
        # + aspirateur - aucun moyen de contourner ça, ce n'est pas une
        # limitation de ce code mais du firmware Neato lui-même.
        self.native_dock_min_charge = int(self.declare_parameter('native_dock_min_charge', 90).value)
        self._native_docking = False
        self.create_subscription(Bool, '/dock_native_request', self._on_dock_native, 10)

        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.create_subscription(Bool, '/safety_stop', self._on_safety_stop, 10)
        self.create_subscription(Bool, '/start_cleaning', self._on_start_cleaning, 10)
        self.create_subscription(Bool, '/start_scan', self._on_start_scan, 10)
        self.create_subscription(Bool, '/start_explore', self._on_start_explore, 10)
        self.create_subscription(Bool, '/return_to_dock', self._on_return_to_dock, 10)
        self.create_subscription(Bool, '/set_home_position', self._on_set_home_position, 10)
        self.create_subscription(String, '/clean_zone_request', self._on_zone_request, 10)

        self._waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- TF (pose robot, nécessaire pour choisir la frontière la plus proche) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

    def _send_raw_command(self, text: str):
        """Envoie une commande MQTT brute (topic clean_cmd), réutilisant
        le mécanisme raw:/pause_polling/resume_polling déjà géré côté ESP
        dans config/comp/slam-odom.yaml."""
        if self._mqtt_client is None:
            return
        try:
            self._mqtt_client.publish(self.mqtt_clean_cmd_topic, text, qos=1)
        except Exception as e:
            self.get_logger().warn(f"Échec publish commande brute '{text}': {e}")

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

    def _on_start_scan(self, msg: Bool):
        """Mode 'rescan' : reparcourt la zone déjà connue de la carte SANS
        activer brosse/aspirateur - utile pour mettre à jour la carte après
        avoir déplacé des meubles. NE FONCTIONNE PAS pour une toute
        première carte (aucun waypoint généré si /map est vide) : pour ça,
        utilise la téléopération manuelle (/api/teleop) à la place.
        Assure-toi que SLAM_MODE=mapping (pas 'localize') côté .env pour
        que slam_toolbox intègre vraiment les changements détectés,
        sinon il se contentera de se localiser sur l'ancienne carte."""
        if msg.data:
            if self._safety_stop:
                self.get_logger().error(
                    "Démarrage refusé : safety_stop actif. Vérifie le robot et "
                    "réarme-le (mqtt_clean_cmd_topic: clear_safety_stop) avant de relancer."
                )
                return
            self._start_cleaning(scan_only=True)
        else:
            self._abort_cleaning()

    # --------------------------------------------------- Exploration automatique
    def _on_start_explore(self, msg: Bool):
        """Exploration automatique par frontières : le robot repère les
        limites entre zone connue et inconnue, va vers la plus proche,
        répète jusqu'à ce qu'il n'y ait plus rien à explorer. Aucun
        nettoyage, jamais de brosse/aspirateur. Fonctionne dès le premier
        scan reçu (pas besoin d'une carte préexistante), contrairement au
        mode scan/nettoyage classique - remplace la téléopération manuelle
        pour la toute première carte dans la majorité des cas."""
        if msg.data:
            if self._safety_stop:
                self.get_logger().error(
                    "Démarrage refusé : safety_stop actif. Vérifie le robot et "
                    "réarme-le (mqtt_clean_cmd_topic: clear_safety_stop) avant de relancer."
                )
                return
            self._start_exploration()
        else:
            self._stop_exploration()

    def _start_exploration(self):
        if self._running or self._exploring or self._docking or self._native_docking:
            self.get_logger().warn("Un cycle est déjà en cours, ignoré")
            return
        self._exploring = True
        self._explore_iterations = 0
        self._explore_blacklist = []
        self.get_logger().info("Exploration automatique démarrée")
        self._exploration_step()

    def _stop_exploration(self):
        self._exploring = False
        if self._explore_goal_handle is not None:
            try:
                self._explore_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"Échec annulation objectif d'exploration: {e}")
        self._explore_goal_handle = None
        self.get_logger().info("Exploration arrêtée")

    def _get_robot_pose(self):
        """Renvoie (x, y, theta) en repère 'map', ou None si la TF n'est
        pas encore disponible."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            q = t.transform.rotation
            theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return t.transform.translation.x, t.transform.translation.y, theta
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _maybe_capture_home_pose(self):
        """Capture la position de départ UNE SEULE FOIS, à la première TF
        disponible après le démarrage - suppose que le robot est posé sur
        son socle au lancement du conteneur. Si ce n'est pas le cas,
        utilise /api/dock/set_home pour redéfinir la position manuellement."""
        if self._home_captured:
            return
        pose = self._get_robot_pose()
        if pose is None:
            return
        self._home_pose = pose
        self._home_captured = True
        self.get_logger().info(
            f"Position de départ capturée automatiquement : "
            f"({pose[0]:.2f}, {pose[1]:.2f}, {math.degrees(pose[2]):.0f}°). "
            f"Si le robot n'était pas sur son socle à cet instant, "
            f"utilise /api/dock/set_home pour corriger."
        )

    def _on_set_home_position(self, msg: Bool):
        """Redéfinit manuellement la position 'socle' à la position
        actuelle du robot (ex: après l'avoir replacé toi-même dessus)."""
        if not msg.data:
            return
        pose = self._get_robot_pose()
        if pose is None:
            self.get_logger().error("Pose robot indisponible (TF), impossible de redéfinir le socle")
            return
        self._home_pose = pose
        self._home_captured = True
        self.get_logger().info(
            f"Position du socle redéfinie manuellement : ({pose[0]:.2f}, {pose[1]:.2f})"
        )

    def _on_return_to_dock(self, msg: Bool):
        if not msg.data:
            self._abort_docking()
            return
        if self._running or self._exploring or self._docking or self._native_docking:
            self.get_logger().warn("Un cycle est déjà en cours, ignoré")
            return
        if self._safety_stop:
            self.get_logger().error(
                "Retour au socle refusé : safety_stop actif. Vérifie le robot et "
                "réarme-le avant de relancer."
            )
            return
        if self._home_pose is None:
            self.get_logger().error(
                "Position du socle inconnue (aucune TF capturée depuis le "
                "démarrage) - utilise /api/dock/set_home une fois le robot "
                "posé sur son socle."
            )
            return

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action navigate_to_pose indisponible (Nav2 lancé ?)")
            return

        x, y, theta = self._home_pose
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(theta / 2.0)

        self._docking = True
        self.get_logger().info(
            f"Retour au socle : navigation vers ({x:.2f}, {y:.2f}) - "
            f"approximatif, ne garantit pas un alignement de charge précis."
        )

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_dock_goal_response)

    def _on_dock_goal_response(self, future):
        if not self._docking:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Objectif de retour au socle refusé par Nav2")
            self._docking = False
            return
        self._dock_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_dock_result)

    def _on_dock_result(self, future):
        self._dock_goal_handle = None
        self._docking = False
        try:
            future.result()
            self.get_logger().info(
                "Retour au socle terminé - vérifie visuellement que le "
                "robot est bien en charge, l'alignement précis n'est pas garanti."
            )
        except Exception as e:
            self.get_logger().warn(f"Retour au socle échoué/annulé: {e}")

    def _abort_docking(self):
        self._docking = False
        if self._dock_goal_handle is not None:
            try:
                self._dock_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"Échec annulation retour au socle: {e}")
        self._dock_goal_handle = None

    def _on_dock_native(self, msg: Bool):
        """Retour au socle NATIF : pause notre round-robin (GetMotor/
        GetLDSScan/SetMotor - pour ne pas interférer avec la navigation
        interne du Neato le temps qu'elle tourne), puis déclenche
        "Clean House" avec un MinCharge élevé pour l'inciter à rentrer
        se recharger plutôt qu'à explorer longuement.

        ⚠️ Réactive brièvement brosse+aspirateur (contrainte du protocole
        Neato, voir commentaire dans __init__ - pas contournable).
        ⚠️ Pendant que ce mode tourne, NOTRE surveillance de sécurité est
        en pause (round-robin arrêté) - le Neato utilise SA PROPRE
        sécurité native à la place, ce qui est normal et attendu pour ce
        mode (il n'a pas besoin de notre supervision pour naviguer avec
        son propre firmware). Pense à cliquer "Arrêter" (ou envoyer
        resume_polling) si tu veux reprendre le contrôle SLAM/Nav2 avant
        que le Neato ait fini de rentrer tout seul.
        """
        if not msg.data:
            self._native_docking = False
            self._send_raw_command("resume_polling")
            return

        if self._running or self._exploring or self._docking or self._native_docking:
            self.get_logger().warn("Un cycle est déjà en cours, ignoré")
            return
        if self._safety_stop:
            self.get_logger().error(
                "Retour au socle natif refusé : safety_stop actif. Vérifie "
                "le robot et réarme-le avant de relancer."
            )
            return

        self._native_docking = True
        self.get_logger().info(
            f"Retour au socle NATIF : pause du round-robin, puis "
            f"'Clean House MinCharge {self.native_dock_min_charge}' - "
            f"brosse/aspirateur vont brièvement se réactiver (contrainte "
            f"du protocole Neato, pas évitable)."
        )
        self._send_raw_command("pause_polling")
        threading.Timer(
            1.0,
            lambda: self._send_raw_command(f"raw:Clean House MinCharge {self.native_dock_min_charge}"),
        ).start()

    def _is_blacklisted(self, frontier, radius_m=0.5):
        for b in self._explore_blacklist:
            if (frontier["x"] - b["x"]) ** 2 + (frontier["y"] - b["y"]) ** 2 < radius_m ** 2:
                return True
        return False

    def _exploration_step(self):
        if not self._exploring:
            return
        if self._safety_stop:
            self.get_logger().error("Exploration interrompue : safety_stop actif")
            self._exploring = False
            return

        self._explore_iterations += 1
        if self._explore_iterations > self.max_explore_iterations:
            self.get_logger().warn(
                f"Exploration arrêtée : limite de {self.max_explore_iterations} "
                f"étapes atteinte (carte trop grande, ou boucle sur des "
                f"frontières inatteignables ?)"
            )
            self._exploring = False
            return

        with self._map_lock:
            map_msg = self._map
        if map_msg is None:
            self.get_logger().error(
                "Aucun scan reçu pour l'instant, impossible de démarrer "
                "l'exploration (vérifie que GetLDSScan fonctionne)"
            )
            self._exploring = False
            return

        margin_cells = max(1, int(round(self.robot_radius_m / map_msg.info.resolution)))
        data = np.array(map_msg.data, dtype=np.int16).reshape((map_msg.info.height, map_msg.info.width))
        # Important : la marge de sécurité pour l'exploration ne doit
        # écarter le robot que des vrais OBSTACLES (data >= 50), pas de
        # l'inconnu - sinon toute cellule "frontière" (par définition
        # voisine de l'inconnu) se fait éroder et il n'y a plus jamais
        # rien à explorer. C'est différent de _generate_coverage_waypoints
        # qui érode par rapport à "libre" (incluant l'inconnu comme
        # obstacle), ce qui est correct pour NE PAS nettoyer dans
        # l'inconnu, mais faux pour trouver où explorer.
        not_obstacle = data < 50
        safe = self._erode_free(not_obstacle, margin_cells)

        frontiers = self._find_frontiers(map_msg, safe)
        frontiers = [f for f in frontiers if not self._is_blacklisted(f)]

        if not frontiers:
            self.get_logger().info(
                f"Exploration terminée : plus de frontière à explorer "
                f"({self._explore_iterations} étape(s))"
            )
            self._exploring = False
            return

        pose = self._get_robot_pose()
        if pose is None:
            self.get_logger().warn("Pose robot indisponible (TF pas encore prête), nouvel essai dans 2s")
            threading.Timer(2.0, self._exploration_step).start()
            return

        rx, ry, _rtheta = pose
        frontiers.sort(key=lambda f: (f["x"] - rx) ** 2 + (f["y"] - ry) ** 2)
        target = frontiers[0]
        self._current_explore_target = target

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = target["x"]
        goal.pose.pose.position.y = target["y"]
        goal.pose.pose.orientation.w = 1.0

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action navigate_to_pose indisponible (Nav2 lancé ? NAV2_ENABLED=true ?)")
            self._exploring = False
            return

        self.get_logger().info(
            f"Exploration [{self._explore_iterations}/{self.max_explore_iterations}] : "
            f"frontière la plus proche à ({target['x']:.2f}, {target['y']:.2f}), "
            f"taille ~{target['size']:.2f}m²"
        )

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_explore_goal_response)

    def _on_explore_goal_response(self, future):
        if not self._exploring:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Objectif d'exploration refusé par Nav2, frontière ignorée")
            self._blacklist_current_target()
            self._exploration_step()
            return
        self._explore_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_explore_result)

    def _on_explore_result(self, future):
        self._explore_goal_handle = None
        try:
            future.result()
        except Exception as e:
            self.get_logger().warn(f"Objectif d'exploration échoué/annulé: {e}")
            self._blacklist_current_target()

        if not self._exploring:
            return
        # Petite pause pour laisser la carte se mettre à jour avant de
        # recalculer les frontières sur des données fraîches.
        threading.Timer(1.5, self._exploration_step).start()

    def _blacklist_current_target(self):
        if self._current_explore_target is not None:
            self._explore_blacklist.append(self._current_explore_target)

    def _find_frontiers(self, map_msg: OccupancyGrid, safe_mask: np.ndarray):
        """Renvoie la liste des groupes de cellules "frontière" (libres,
        sûres, et adjacentes à de l'inconnu), sous forme de
        [{"x":.., "y":.., "size":..(m²)}]. Regroupement par parcours en
        largeur (BFS) pur Python/numpy, pas de dépendance scipy."""
        info = map_msg.info
        w, h, res = info.width, info.height, info.resolution
        data = np.array(map_msg.data, dtype=np.int16).reshape((h, w))
        unknown = data < 0

        def shift(arr, dy, dx):
            out = np.zeros_like(arr)
            y0, y1 = max(0, dy), h + min(0, dy)
            x0, x1 = max(0, dx), w + min(0, dx)
            sy0, sy1 = max(0, -dy), h + min(0, -dy)
            sx0, sx1 = max(0, -dx), w + min(0, -dx)
            if y1 > y0 and x1 > x0:
                out[y0:y1, x0:x1] = arr[sy0:sy1, sx0:sx1]
            return out

        adj_unknown = shift(unknown, 1, 0) | shift(unknown, -1, 0) | shift(unknown, 0, 1) | shift(unknown, 0, -1)
        free = (data >= 0) & (data < 50)
        frontier_mask = free & safe_mask & adj_unknown

        min_cells = max(1, int(self.min_frontier_area_m2 / (res * res)))
        visited = np.zeros_like(frontier_mask)
        clusters = []
        ys, xs = np.where(frontier_mask)
        for y0, x0 in zip(ys.tolist(), xs.tolist()):
            if visited[y0, x0]:
                continue
            stack = [(y0, x0)]
            visited[y0, x0] = True
            cells = []
            while stack:
                cy, cx = stack.pop()
                cells.append((cy, cx))
                for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    ny, nx = cy + dy, cx + dx
                    if 0 <= ny < h and 0 <= nx < w and frontier_mask[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = True
                        stack.append((ny, nx))
            if len(cells) >= min_cells:
                cy_avg = sum(c[0] for c in cells) / len(cells)
                cx_avg = sum(c[1] for c in cells) / len(cells)
                # Le centroïde géométrique peut tomber HORS du cluster pour
                # une frontière en anneau (ex: zone isolée entourée
                # d'inconnu de tous les côtés) - on vise donc la vraie
                # cellule du cluster la plus proche du centroïde, jamais un
                # point fictif qui ne serait même pas une frontière.
                best_cell = min(cells, key=lambda c: (c[0] - cy_avg) ** 2 + (c[1] - cx_avg) ** 2)
                cy, cx = best_cell
                wx = info.origin.position.x + (cx + 0.5) * res
                wy = info.origin.position.y + (cy + 0.5) * res
                clusters.append({"x": wx, "y": wy, "size": len(cells) * res * res})
        return clusters

    def _on_zone_request(self, msg: String):
        """Reçoit un polygone à nettoyer, payload JSON :
        {"polygon": [[x1,y1],[x2,y2],[x3,y3],...]} en coordonnées monde
        (mètres, repère "map" - le même que la carte SLAM), au moins 3
        points. Publié par slam_server.py (/api/clean/zone), lui-même
        appelé par la carte interactive côté Home Assistant."""
        if self._safety_stop:
            self.get_logger().error(
                "Démarrage zone refusé : safety_stop actif. Vérifie le robot avant de relancer."
            )
            return
        try:
            data = json.loads(msg.data)
            polygon = [(float(p[0]), float(p[1])) for p in data["polygon"]]
        except (json.JSONDecodeError, KeyError, TypeError, ValueError, IndexError) as e:
            self.get_logger().error(f"Requête de zone invalide, ignorée: {e}")
            return
        if len(polygon) < 3:
            self.get_logger().error("Polygone de zone invalide (moins de 3 points), ignoré")
            return
        self._start_cleaning(polygon=polygon)

    # ------------------------------------------------------------- Cycle nettoyage
    def _start_cleaning(self, polygon=None, scan_only=False):
        if self._running or self._exploring or self._docking or self._native_docking:
            self.get_logger().warn("Un cycle est déjà en cours, ignoré")
            return

        with self._map_lock:
            map_msg = self._map
        if map_msg is None:
            self.get_logger().error(
                "Pas de carte disponible pour l'instant, annulé - pour une "
                "toute première carte, utilise l'exploration automatique ou "
                "la téléopération manuelle (/api/teleop) plutôt que ce mode."
            )
            return

        if not self._waypoints_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action follow_waypoints indisponible (Nav2 lancé ? NAV2_ENABLED=true ?)")
            return

        self._running = True
        self._scan_only = scan_only
        self._clean_polygon = polygon
        self._covered_points = []

        mode_desc = "scan seul, pas de nettoyage" if scan_only else "nettoyage"
        zone_desc = f"zone ({len(polygon)} points)" if polygon else "carte entière"
        self.get_logger().info(
            f"Cycle [{mode_desc}, {zone_desc}] démarré - replanification "
            f"par lots de {self.replanning_batch_size} points"
        )

        if not scan_only:
            self._publish_clean_cmd(True)

        self._cleaning_step()

    def _cleaning_step(self):
        """Un 'pas' du cycle : recalcule le parcours restant à partir d'une
        carte FRAÎCHE (donc réagit aux meubles déplacés depuis le dernier
        pas), envoie le prochain petit lot de points, et se rappelle
        lui-même une fois ce lot terminé."""
        if not self._running:
            return
        if self._safety_stop:
            self.get_logger().error("Cycle interrompu : safety_stop actif")
            self._abort_cleaning()
            return

        with self._map_lock:
            map_msg = self._map
        if map_msg is None:
            self.get_logger().error("Carte devenue indisponible, arrêt du cycle")
            self._running = False
            if not self._scan_only:
                self._publish_clean_cmd(False)
            return

        remaining = self._generate_coverage_waypoints(
            map_msg, polygon=self._clean_polygon, exclude_points=self._covered_points
        )

        if not remaining:
            label = "scan" if self._scan_only else "nettoyage"
            self.get_logger().info(
                f"Cycle [{label}] terminé : plus de zone accessible non couverte "
                f"({len(self._covered_points)} point(s) de passage au total)"
            )
            self._running = False
            if not self._scan_only:
                self._publish_clean_cmd(False)
            return

        batch = remaining[: self.replanning_batch_size]
        self._current_batch = batch

        goal = FollowWaypoints.Goal()
        goal.poses = [self._make_pose(x, y) for (x, y, _connect) in batch]

        send_future = self._waypoints_client.send_goal_async(
            goal, feedback_callback=self._on_waypoints_feedback
        )
        send_future.add_done_callback(self._on_cleaning_goal_response)

    def _on_cleaning_goal_response(self, future):
        if not self._running:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Lot refusé par Nav2, arrêt du cycle")
            self._running = False
            if not self._scan_only:
                self._publish_clean_cmd(False)
            return
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_cleaning_batch_result)

    def _on_waypoints_feedback(self, feedback_msg):
        if self._safety_stop:
            self._abort_cleaning()

    def _on_cleaning_batch_result(self, future):
        self._active_goal_handle = None
        try:
            future.result()
            # Qu'un point du lot ait été manqué ou non, on le marque quand
            # même "couvert" pour ne pas boucler indéfiniment dessus - une
            # frontière/segment inatteignable serait sinon reproposé à
            # chaque replanification.
            self._mark_covered(self._current_batch)
        except Exception as e:
            self.get_logger().warn(f"Lot terminé avec erreur/annulation: {e}")
            self._mark_covered(self._current_batch)

        if not self._running:
            return
        # Petite pause pour laisser la carte se mettre à jour (nouveau
        # scan reçu) avant de recalculer le parcours restant.
        threading.Timer(1.0, self._cleaning_step).start()

    def _mark_covered(self, batch):
        """Marque comme couverte toute la ligne entre points consécutifs
        du lot QUAND c'est un vrai balayage en ligne droite (connect_to_prev
        True, même segment) - pour les sauts entre segments/lignes, on ne
        marque que le point d'arrivée, pas toute la distance parcourue par
        Nav2 pour y aller (qui ne suit pas forcément une ligne droite)."""
        if not batch:
            return
        pts = []
        prev_xy = None
        for (x, y, connect) in batch:
            if connect and prev_xy is not None:
                x0, y0 = prev_xy
                dist = math.hypot(x - x0, y - y0)
                steps = max(1, int(dist / max(self.coverage_mark_radius_m, 0.01)))
                for s in range(1, steps + 1):
                    t = s / steps
                    pts.append((x0 + (x - x0) * t, y0 + (y - y0) * t))
            else:
                pts.append((x, y))
            prev_xy = (x, y)
        self._covered_points.extend(pts)

    def _abort_cleaning(self):
        if not self._scan_only:
            self._publish_clean_cmd(False)
        if self._active_goal_handle is not None:
            try:
                self._active_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"Échec annulation objectif: {e}")
        self._running = False

    # ------------------------------------------------------- Génération du parcours
    def _generate_coverage_waypoints(self, map_msg: OccupancyGrid, polygon=None, exclude_points=None):
        """Renvoie une liste de tuples (x, y) monde (PAS des PoseStamped -
        conversion via _make_pose au moment d'envoyer un lot à Nav2)."""
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

        if polygon is not None:
            zone_mask = self._polygon_mask(polygon, info, w, h)
            safe = safe & zone_mask

        if exclude_points:
            covered = self._covered_mask_for(info, w, h, exclude_points, self.coverage_mark_radius_m)
            safe = safe & ~covered

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
                connect_to_prev = False  # 1er point d'un segment = un saut, pas un balayage
                for col in cols:
                    wx = info.origin.position.x + (col + 0.5) * res
                    wy = info.origin.position.y + (row + 0.5) * res
                    # Le 3e élément indique si ce point est relié au
                    # précédent par un VRAI balayage en ligne droite (même
                    # segment) plutôt qu'un saut vers un autre segment/ligne
                    # - _mark_covered s'en sert pour ne pas marquer comme
                    # "couvert" tout l'espace traversé par un saut (qui suit
                    # le chemin de Nav2, pas une ligne droite).
                    waypoints.append((wx, wy, connect_to_prev))
                    connect_to_prev = True
            reverse = not reverse

        return waypoints

    @staticmethod
    def _covered_mask_for(info, w: int, h: int, points, radius_m: float) -> np.ndarray:
        """Masque booléen (h, w) : True pour les cellules à moins de
        radius_m d'au moins un des points donnés (coordonnées monde)."""
        if not points:
            return np.zeros((h, w), dtype=bool)
        res = info.resolution
        cols = np.arange(w)
        rows = np.arange(h)
        wx = info.origin.position.x + (cols + 0.5) * res
        wy = info.origin.position.y + (rows + 0.5) * res
        WX, WY = np.meshgrid(wx, wy)
        mask = np.zeros((h, w), dtype=bool)
        r2 = radius_m * radius_m
        for (px, py) in points:
            mask |= (WX - px) ** 2 + (WY - py) ** 2 <= r2
        return mask

    @staticmethod
    def _polygon_mask(polygon_world, info, w: int, h: int) -> np.ndarray:
        """Renvoie un masque booléen (h, w) : True pour les cellules dont
        le centre tombe à l'intérieur du polygone (coordonnées monde,
        repère "map"). Algorithme du ray casting (règle pair/impair),
        vectorisé avec numpy - pas de dépendance à shapely/matplotlib."""
        res = info.resolution
        # Grille des coordonnées monde du centre de chaque cellule
        cols = np.arange(w)
        rows = np.arange(h)
        wx = info.origin.position.x + (cols + 0.5) * res          # (w,)
        wy = info.origin.position.y + (rows + 0.5) * res          # (h,)
        WX, WY = np.meshgrid(wx, wy)                                # (h, w) chacun

        mask = np.zeros((h, w), dtype=bool)
        n = len(polygon_world)
        for i in range(n):
            x1, y1 = polygon_world[i]
            x2, y2 = polygon_world[(i + 1) % n]
            # Arête traversée par le rayon horizontal partant de (WX,WY) ?
            cond = ((y1 > WY) != (y2 > WY))
            with np.errstate(divide='ignore', invalid='ignore'):
                x_intersect = (x2 - x1) * (WY - y1) / (y2 - y1 + 1e-12) + x1
            crosses = cond & (WX < x_intersect)
            mask ^= crosses
        return mask

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
