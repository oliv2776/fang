#!/usr/bin/env python3
"""
diagnose.py — Script de diagnostic pour valider les inconnues du pipeline
SLAM Neato D7 gen3, sans dépendre de ROS2/Docker (juste MQTT + paho-mqtt).

À lancer directement sur le serveur Linux (ou n'importe où sur le réseau
qui peut joindre le broker MQTT) :

    pip3 install paho-mqtt --break-system-packages   # si pas déjà installé
    python3 diagnose.py --broker 192.168.10.108

Ce script fait deux choses :
  1. MODE ÉCOUTE (par défaut) : s'abonne à tous les topics neato/robot/*,
     affiche chaque message reçu en clair avec des vérifications de
     cohérence (nombre de points du scan, plausibilité des deltas roues,
     présence des champs de sécurité, etc.).
  2. MODE TEST GUIDÉ (--guided) : suspend le round-robin automatique côté
     ESP32, te fait tester une commande à la fois (GetMotor, GetLDSScan,
     GetDigitalSensors, GetAnalogSensors), en affichant le résultat décodé
     après chacune, avant de passer à la suivante.

Ce script affiche les données APRÈS parsing côté ESP32 (donc si le format
GetLDSScan réel diffère de ce qui est supposé dans gen3.yaml, ce script ne
le verra pas forcément - pour comparer au format BRUT réel, regarde en
parallèle les logs ESPHome (`esphome logs config/local.yaml`), qui
affichent chaque ligne reçue via uart_debug avant tout parsing.
"""

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("paho-mqtt manquant. Installe-le avec :")
    print("  pip3 install paho-mqtt --break-system-packages")
    sys.exit(1)


def ts():
    return datetime.now().strftime("%H:%M:%S")


def color(text, code):
    return f"\033[{code}m{text}\033[0m"


def ok(text):
    return color(text, "32")


def warn(text):
    return color(text, "33")


def err(text):
    return color(text, "31")


def read_env_default(key: str, fallback: str) -> str:
    """Lit une valeur depuis slam/.env si le fichier existe (même dossier
    que ce script), sinon renvoie fallback. Parseur minimal (KEY=VALUE),
    pas de dépendance à python-dotenv. Permet à ce script autonome de
    rester synchronisé avec le même .env que docker-compose, sans avoir
    à maintenir une valeur par défaut séparée à la main."""
    env_path = Path(__file__).parent / ".env"
    if not env_path.exists():
        return fallback
    try:
        for line in env_path.read_text().splitlines():
            line = line.strip()
            if not line or line.startswith('#') or '=' not in line:
                continue
            k, _, v = line.partition('=')
            if k.strip() == key:
                return v.strip().strip('"').strip("'")
    except OSError:
        pass
    return fallback


class Diagnostics:
    def __init__(self, broker, port, prefix, username="", password=""):
        self.broker = broker
        self.port = port
        self.prefix = prefix
        self._last_wheels = None

        self.client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION1)
        if username:
            self.client.username_pw_set(username, password)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

    def connect(self):
        print(f"[{ts()}] Connexion à {self.broker}:{self.port} ...")
        self.client.connect(self.broker, self.port, keepalive=60)
        self.client.loop_start()

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 5:
            print(err(f"[{ts()}] Échec de connexion MQTT : accès refusé (rc=5) - "
                       f"vérifie --username/--password ou MQTT_USERNAME/MQTT_PASSWORD dans .env"))
            return
        if rc != 0:
            print(err(f"[{ts()}] Échec de connexion MQTT (code {rc})"))
            return
        print(ok(f"[{ts()}] Connecté."))
        client.subscribe(f"{self.prefix}/#", qos=1)
        print(f"[{ts()}] Abonné à {self.prefix}/#\n")

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        try:
            payload = msg.payload.decode('utf-8')
        except UnicodeDecodeError:
            print(warn(f"[{ts()}] {topic}: payload binaire non-UTF8 ({len(msg.payload)} octets) - inattendu, tout est censé être en JSON/texte"))
            return

        if topic.endswith('/wheels'):
            self._check_wheels(payload)
        elif topic.endswith('/scan'):
            self._check_scan(payload)
        elif topic.endswith('/safety'):
            self._check_safety(payload)
        elif topic.endswith('/cmd_vel_out'):
            print(f"[{ts()}] cmd_vel_out: {payload}")
        else:
            print(f"[{ts()}] {topic}: {payload[:200]}")

    # ---------------------------------------------------------------- checks
    def _check_wheels(self, payload):
        try:
            data = json.loads(payload)
            left = float(data['left_mm'])
            right = float(data['right_mm'])
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            print(err(f"[{ts()}] /wheels: JSON invalide ou champs manquants ({e}) -> {payload[:150]}"))
            return

        delta_str = ""
        if self._last_wheels is not None:
            dl = left - self._last_wheels[0]
            dr = right - self._last_wheels[1]
            flag = warn(" ⚠ delta > 1m, suspect (reset compteur ?)") if (abs(dl) > 1000 or abs(dr) > 1000) else ""
            delta_str = f" (Δleft={dl:+.1f}mm Δright={dr:+.1f}mm){flag}"
        self._last_wheels = (left, right)
        print(ok(f"[{ts()}] /wheels OK") + f"  left={left}mm right={right}mm{delta_str}")

    def _check_scan(self, payload):
        try:
            data = json.loads(payload)
            ranges = data['ranges']
        except (json.JSONDecodeError, KeyError) as e:
            print(err(f"[{ts()}] /scan: JSON invalide ou champ 'ranges' manquant ({e})"))
            return

        n = len(ranges)
        n_valid = sum(1 for r in ranges if r is not None)
        n_null = n - n_valid
        status = ok("OK") if n == 360 else warn(f"ATTENDU 360, reçu {n}")
        print(f"[{ts()}] /scan {status}  {n} points, {n_valid} valides, {n_null} null "
              f"(pas de retour) — angle_min={data.get('angle_min')} angle_max={data.get('angle_max')}")

        if n_valid > 0:
            valid_vals = [r for r in ranges if r is not None]
            vmin, vmax = min(valid_vals), max(valid_vals)
            if vmin < 0.05 or vmax > 8.5:
                print(warn(f"    ⚠ valeurs hors plage attendue [0.1, 8.0]m : min={vmin} max={vmax}"))
            else:
                print(f"    distances valides: min={vmin:.2f}m max={vmax:.2f}m")
        if n_valid == 0:
            print(warn("    ⚠ AUCUN point valide dans ce scan — soit le robot ne voit que du vide "
                        "(peu probable en intérieur), soit le parsing GetLDSScan ne trouve rien: "
                        "compare avec les logs ESPHome bruts."))

    def _check_safety(self, payload):
        try:
            data = json.loads(payload)
            required = {'bump', 'cliff', 'wheel_extended', 'stop'}
            missing = required - set(data.keys())
        except json.JSONDecodeError as e:
            print(err(f"[{ts()}] /safety: JSON invalide ({e})"))
            return
        if missing:
            print(err(f"[{ts()}] /safety: champs manquants: {missing}"))
            return

        if data['stop']:
            print(err(f"[{ts()}] /safety: ARRÊT ACTIF  bump={data['bump']} "
                       f"cliff={data['cliff']} wheel_extended={data['wheel_extended']}"))
        else:
            print(ok(f"[{ts()}] /safety OK") + f"  bump={data['bump']} cliff={data['cliff']} "
                  f"wheel_extended={data['wheel_extended']}")

    # -------------------------------------------------------------- commandes
    def send_cmd(self, text):
        topic = f"{self.prefix}/clean_cmd"
        self.client.publish(topic, text, qos=1)
        print(f"[{ts()}] -> envoyé sur {topic}: {text}")


def guided_mode(diag: Diagnostics):
    print("\n" + "=" * 60)
    print("MODE TEST GUIDÉ")
    print("=" * 60)
    print("Le round-robin automatique va être mis en pause : plus AUCUNE")
    print("commande automatique (y compris la surveillance sécurité) ne")
    print("partira tant que tu n'auras pas repris. Robot immobile/roues")
    print("en l'air recommandé, surtout avant de tester SetMotor.\n")
    input("Appuie sur Entrée pour mettre en pause et commencer...")
    diag.send_cmd("pause_polling")
    time.sleep(1)

    commands = [
        ("GetMotor", "Odométrie (positions roues)"),
        ("GetDigitalSensors", "Chocs / roue soulevée"),
        ("GetAnalogSensors", "Détection de vide (cliff)"),
        ("GetLDSScan", "Scan LiDAR (le plus incertain)"),
    ]

    try:
        for cmd, desc in commands:
            input(f"\n--- {cmd} ({desc}) — Entrée pour envoyer ---")
            diag.send_cmd(f"raw:{cmd}")
            print("Résultat décodé (si applicable) affiché ci-dessus dans le flux MQTT.")
            print("Compare aussi avec les logs ESPHome bruts pour voir la réponse AVANT parsing.")
            time.sleep(2)

        test_setmotor = input(
            "\n⚠️  Tester SetMotor maintenant ? Robot DOIT être immobile/roues en "
            "l'air. Distance minimale, à observer attentivement. (o/N) "
        )
        if test_setmotor.lower() == 'o':
            input("Entrée pour envoyer un SetMotor de test très court...")
            diag.send_cmd("raw:SetMotor RWheelDist 50 LWheelDist 50 Speed 20")
            time.sleep(2)
            diag.send_cmd("raw:SetMotor LWheelDisable RWheelDisable")
            print("Arrêt envoyé. Les roues ont-elles tourné d'une distance plausible (~5cm) ?")
    finally:
        input("\nEntrée pour reprendre le round-robin automatique normal...")
        diag.send_cmd("resume_polling")
        print(ok("Round-robin repris."))


def main():
    parser = argparse.ArgumentParser(description="Diagnostic MQTT du pipeline SLAM Neato")
    parser.add_argument("--broker", default=read_env_default("MQTT_BROKER", "192.168.10.108"),
                         help="IP du broker MQTT (lu depuis slam/.env si présent)")
    parser.add_argument("--port", type=int, default=int(read_env_default("MQTT_PORT", "1883")))
    parser.add_argument("--prefix", default=read_env_default("MQTT_PREFIX", "neato/robot"),
                         help="Doit matcher MQTT_PREFIX")
    parser.add_argument("--guided", action="store_true", help="Lance le mode test guidé (commande par commande)")
    parser.add_argument("--username", default=read_env_default("MQTT_USERNAME", ""))
    parser.add_argument("--password", default=read_env_default("MQTT_PASSWORD", ""))
    args = parser.parse_args()

    diag = Diagnostics(args.broker, args.port, args.prefix, args.username, args.password)
    diag.connect()
    time.sleep(1.5)  # laisse le temps à la connexion/abonnement de se faire

    if args.guided:
        guided_mode(diag)
    else:
        print("Mode écoute (Ctrl+C pour quitter). Utilise --guided pour le mode test pas-à-pas.\n")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nArrêt.")


if __name__ == '__main__':
    main()
