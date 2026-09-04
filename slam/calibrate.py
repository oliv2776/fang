#!/usr/bin/env python3
"""
calibrate.py — Calibration guidée des capteurs/actionneurs du Neato D7 gen3.

Comme diagnose.py --guided : LOCAL uniquement, jamais déclenchable depuis
HA/à distance, parce que ça fait bouger le robot et/ou demande une mesure
physique de ta part. Toutes les valeurs calibrées sont envoyées via MQTT
vers des globals ESPHome persistés en flash (pas besoin de reflasher entre
deux essais, ni après - elles survivent au redémarrage de l'ESP32).

    pip3 install paho-mqtt --break-system-packages   # si pas déjà fait
    python3 calibrate.py --broker 192.168.10.108

Trois étapes, chacune indépendante (tu peux Ctrl+C entre deux sans casser
ce qui a déjà été calibré) :
  1. Seuil de détection de vide (g_cal_drop_threshold_mm)
  2. Échelle distance SetMotor (g_cal_dist_scale)
  3. Empattement / wheel_base (g_cal_wheel_base_m) — OPTIONNEL, expérimental,
     dépend d'un comportement de SetMotor (distances négatives) non
     confirmé. Skippée par défaut, à activer explicitement.
"""

import argparse
import json
import statistics
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


def ok(t): return color(t, "32")
def warn(t): return color(t, "33")
def err(t): return color(t, "31")


def read_env_default(key: str, fallback: str) -> str:
    """Lit une valeur depuis slam/.env si le fichier existe (même dossier
    que ce script), sinon renvoie fallback. Voir diagnose.py pour la même
    fonction - dupliquée ici pour garder ces deux scripts autonomes
    (pas de module partagé entre les deux, volontairement simple)."""
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


class Calibrator:
    def __init__(self, broker, port, prefix, username="", password=""):
        self.prefix = prefix
        self._last_safety = None
        self._last_wheels = None

        self.client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION1)
        if username:
            self.client.username_pw_set(username, password)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.connect(broker, port, keepalive=30)
        self.client.loop_start()

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 5:
            print(err("Échec de connexion MQTT : accès refusé (rc=5) - "
                       "vérifie --username/--password ou MQTT_USERNAME/MQTT_PASSWORD dans .env"))
        if rc == 0:
            client.subscribe(f"{self.prefix}/#", qos=1)

    def _on_message(self, client, userdata, msg):
        if msg.topic.endswith('/safety'):
            try:
                self._last_safety = json.loads(msg.payload.decode())
            except json.JSONDecodeError:
                pass
        elif msg.topic.endswith('/wheels'):
            try:
                self._last_wheels = json.loads(msg.payload.decode())
            except json.JSONDecodeError:
                pass

    def send_cmd(self, text):
        self.client.publish(f"{self.prefix}/clean_cmd", text, qos=1)

    def close(self):
        self.client.loop_stop()
        self.client.disconnect()

    # -------------------------------------------------------- primitives
    def sample_drop_sensors(self, n=5, delay=0.6):
        """Envoie GetAnalogSensors n fois, renvoie la liste des
        (drop_left_mm, drop_right_mm) lus."""
        samples = []
        for _ in range(n):
            self._last_safety = None
            self.send_cmd("raw:GetAnalogSensors")
            deadline = time.time() + 3.0
            while time.time() < deadline and self._last_safety is None:
                time.sleep(0.1)
            if self._last_safety and 'drop_left_mm' in self._last_safety:
                s = self._last_safety
                samples.append((s['drop_left_mm'], s['drop_right_mm']))
                print(f"  [{ts()}] drop_left={s['drop_left_mm']}mm drop_right={s['drop_right_mm']}mm")
            else:
                print(warn(f"  [{ts()}] pas de réponse, on continue"))
            time.sleep(delay)
        return samples

    def sample_wheels(self):
        self._last_wheels = None
        self.send_cmd("raw:GetMotor")
        # 3s était trop court : si le round-robin était en plein GetLDSScan
        # (jusqu'à 3.6s+ de réponse) au moment du pause_polling, notre
        # propre synchronisation (g_awaiting_response côté ESP) fait
        # légitimement attendre la fin de cette réponse avant de traiter
        # quoi que ce soit d'autre - dépassement possible même sans
        # aucun problème réel de communication.
        deadline = time.time() + 8.0
        while time.time() < deadline and self._last_wheels is None:
            time.sleep(0.1)
        return self._last_wheels


# ------------------------------------------------------------------ étapes
def calibrate_drop_threshold(cal: Calibrator):
    print("\n" + "=" * 60)
    print("ÉTAPE 1 — Seuil de détection de vide (anti-chute)")
    print("=" * 60)
    print("Cette étape ne fait PAS bouger le robot, juste des lectures de")
    print("capteur. Mets le round-robin en pause pour isoler les mesures.\n")

    cal.send_cmd("pause_polling")
    time.sleep(1)

    input("Pose le robot bien à plat sur un sol NORMAL, puis Entrée...")
    floor_samples = cal.sample_drop_sensors()
    if not floor_samples:
        print(err("Aucune donnée reçue, abandon de cette étape."))
        cal.send_cmd("resume_polling")
        return
    floor_vals = [v for pair in floor_samples for v in pair]
    floor_avg = statistics.mean(floor_vals)
    floor_max = max(floor_vals)
    print(f"\nSol normal : moyenne={floor_avg:.1f}mm  max={floor_max:.1f}mm\n")

    input(
        "Maintenant, tiens le robot AU-DESSUS DU VIDE (au-dessus d'une "
        "marche/table, capteurs anti-chute vers le bas, robot bien stable "
        "dans tes mains), puis Entrée..."
    )
    void_samples = cal.sample_drop_sensors()
    cal.send_cmd("resume_polling")

    if not void_samples:
        print(err("Aucune donnée reçue au-dessus du vide, abandon (pas de seuil envoyé)."))
        return
    void_vals = [v for pair in void_samples for v in pair]
    void_avg = statistics.mean(void_vals)
    void_min = min(void_vals)
    print(f"\nAu-dessus du vide : moyenne={void_avg:.1f}mm  min={void_min:.1f}mm")

    if void_min <= floor_max:
        print(err(
            f"\n⚠️ PROBLÈME : la valeur mini au-dessus du vide ({void_min}) "
            f"n'est pas clairement supérieure à la valeur max au sol "
            f"({floor_max}). Impossible de fixer un seuil fiable avec ces "
            f"mesures - le capteur ne discrimine peut-être pas comme "
            f"attendu, ou le robot n'était pas assez au-dessus du vide. "
            f"Aucun seuil envoyé, recommence cette étape."
        ))
        return

    # Seuil à mi-chemin entre le pire cas "sol" et le meilleur cas "vide",
    # avec une marge de sécurité vers le côté prudent (plus bas = plus
    # sensible = déclenche plus facilement, préférable pour la sécurité).
    threshold = floor_max + (void_min - floor_max) * 0.4
    print(ok(f"\nSeuil calculé : {threshold:.1f}mm") +
          f"  (sol max={floor_max:.1f}, vide min={void_min:.1f})")

    confirm = input(f"Envoyer ce seuil ({threshold:.1f}mm) au robot ? (o/N) ")
    if confirm.lower() == 'o':
        cal.send_cmd(f"cal:drop_threshold:{threshold:.1f}")
        print(ok(f"Envoyé : cal:drop_threshold:{threshold:.1f}"))
        print("Persisté en flash sur l'ESP32 - survira à un redémarrage.")
    else:
        print("Annulé, seuil non modifié.")


def calibrate_distance_scale(cal: Calibrator):
    print("\n" + "=" * 60)
    print("ÉTAPE 2 — Échelle distance SetMotor")
    print("=" * 60)
    print("⚠️ Cette étape FAIT BOUGER LE ROBOT (~1m en ligne droite).")
    print("TestMode utilisé ici de façon ponctuelle et isolée (activé juste")
    print("avant SetMotor, désactivé juste après) - différent de la")
    print("réaffirmation en continu par le round-robin qui posait problème")
    print("par le passé.")
    print("Place-le sur un sol DÉGAGÉ, plat, sans obstacle sur au moins 1.5m")
    print("devant lui. Reste à portée de main.\n")

    confirm = input("Robot prêt, sol dégagé confirmé ? (o/N) ")
    if confirm.lower() != 'o':
        print("Annulé.")
        return

    cal.send_cmd("pause_polling")
    time.sleep(1)
    cal.send_cmd("raw:TestMode on")
    time.sleep(0.3)

    before = cal.sample_wheels()
    if not before:
        print(err("Pas de lecture GetMotor initiale, abandon."))
        cal.send_cmd("raw:TestMode off")
        cal.send_cmd("resume_polling")
        return
    print(f"Avant : left={before['left_mm']}mm right={before['right_mm']}mm")

    commanded_mm = 1000
    speed = 50
    print(f"\nEnvoi de SetMotor ({commanded_mm}mm, vitesse {speed})...")
    print("Observe le robot avancer. Attends qu'il s'arrête tout seul...")
    cal.send_cmd(f"raw:SetMotor RWheelDist {commanded_mm} LWheelDist {commanded_mm} Speed {speed}")
    time.sleep(8)

    after = cal.sample_wheels()
    cal.send_cmd("raw:TestMode off")
    cal.send_cmd("resume_polling")
    if not after:
        print(err("Pas de lecture GetMotor finale, abandon."))
        return
    print(f"Après : left={after['left_mm']}mm right={after['right_mm']}mm")


    delta_left = after['left_mm'] - before['left_mm']
    delta_right = after['right_mm'] - before['right_mm']
    encoder_avg = (abs(delta_left) + abs(delta_right)) / 2
    print(f"\nDelta encodeur : left={delta_left:+.1f}mm right={delta_right:+.1f}mm "
          f"(moyenne={encoder_avg:.1f}mm, commandé={commanded_mm}mm)")

    if encoder_avg < 5:
        print(err(
            "Delta quasi nul (<5mm) - le robot n'a probablement pas bougé "
            "du tout malgré TestMode actif. Abandon de cette méthode."
        ))
        return

    measured = input(
        "\nMesure maintenant la distance RÉELLEMENT parcourue au sol (mètre "
        "ruban, en mm), ou Entrée pour utiliser uniquement la valeur "
        "encodeur ci-dessus : "
    ).strip()

    if measured:
        try:
            physical_mm = float(measured)
            scale = physical_mm / commanded_mm
            print(f"Échelle physique (mesure au sol / commandé) : {scale:.4f}")
        except ValueError:
            print(warn("Valeur non numérique, on retombe sur l'encodeur."))
            scale = encoder_avg / commanded_mm if commanded_mm else 1.0
    else:
        scale = encoder_avg / commanded_mm if commanded_mm else 1.0
        print(f"Échelle encodeur (delta encodeur / commandé) : {scale:.4f}")

    if scale <= 0 or scale > 3.0:
        print(err(f"Valeur d'échelle suspecte ({scale:.4f}), hors de [0, 3] - abandon, rien envoyé."))
        return

    confirm = input(f"Envoyer cette échelle ({scale:.4f}) au robot ? (o/N) ")
    if confirm.lower() == 'o':
        cal.send_cmd(f"cal:dist_scale:{scale:.4f}")
        print(ok(f"Envoyé : cal:dist_scale:{scale:.4f}"))
    else:
        print("Annulé, échelle non modifiée.")



def calibrate_wheel_base(cal: Calibrator):
    print("\n" + "=" * 60)
    print("ÉTAPE 3 (OPTIONNELLE, EXPÉRIMENTALE) — Empattement (wheel_base)")
    print("=" * 60)
    print("⚠️ Repose sur une rotation pure (une roue avance, l'autre recule)")
    print("via SetMotor avec une distance NÉGATIVE sur une roue - jamais")
    print("confirmé fonctionner sur ce robot (aucune trace documentée d'un")
    print("test réussi avec une valeur négative). TestMode utilisé ici de")
    print("façon ponctuelle et isolée, comme à l'Étape 2.\n")

    confirm = input("Continuer quand même ? (o/N) ")
    if confirm.lower() != 'o':
        print("Étape sautée.")
        return

    print("Marque bien l'orientation actuelle du robot (repère au sol, "
          "flèche de direction...) avant de continuer.")
    input("Prêt ? Entrée pour lancer la rotation de test...")

    cal.send_cmd("pause_polling")
    time.sleep(1)
    cal.send_cmd("raw:TestMode on")
    time.sleep(0.3)

    rot_mm = 150  # petite rotation, prudence
    cal.send_cmd(f"raw:SetMotor RWheelDist {rot_mm} LWheelDist -{rot_mm} Speed 20")
    print("Observe la rotation. Attends l'arrêt complet...")
    time.sleep(6)
    cal.send_cmd("raw:SetMotor LWheelDisable RWheelDisable")
    cal.send_cmd("raw:TestMode off")
    cal.send_cmd("resume_polling")

    angle_str = input(
        "\nMesure l'angle RÉELLEMENT tourné (degrés, au jugé avec un "
        "rapporteur/repère au sol) : "
    ).strip()
    try:
        angle_deg = float(angle_str)
    except ValueError:
        print(err("Valeur non numérique, abandon."))
        return
    if angle_deg <= 0:
        print(err("Angle nul ou négatif, abandon."))
        return

    import math
    angle_rad = math.radians(angle_deg)
    # Cinématique différentielle : angle = (delta_right - delta_left) / wheel_base
    # delta_right - delta_left = 2 * rot_mm (roues en sens opposés)
    wheel_base_m = (2 * rot_mm / 1000.0) / angle_rad
    print(f"\nEmpattement calculé : {wheel_base_m:.4f} m (mesuré: {angle_deg}° pour "
          f"±{rot_mm}mm par roue)")

    if wheel_base_m <= 0.05 or wheel_base_m > 0.6:
        print(err(f"Valeur suspecte ({wheel_base_m:.4f}m), hors de [0.05, 0.6] - abandon."))
        return

    confirm = input(f"Envoyer cet empattement ({wheel_base_m:.4f}m) au robot ? (o/N) ")
    if confirm.lower() == 'o':
        cal.send_cmd(f"cal:wheel_base:{wheel_base_m:.4f}")
        print(ok(f"Envoyé : cal:wheel_base:{wheel_base_m:.4f}"))
        print(warn(
            "N'oublie pas de reporter cette même valeur dans le paramètre "
            "ROS wheel_base_m de slam_bridge.py (via .env ou entrypoint.sh) "
            "- les deux DOIVENT rester synchronisés manuellement, rien ne "
            "le fait automatiquement entre l'ESP32 et le conteneur Docker."
        ))
    else:
        print("Annulé, empattement non modifié.")


def test_negative_distance(cal: Calibrator):
    print("\n" + "=" * 60)
    print("TEST DIAGNOSTIC — Distance négative sur une seule roue")
    print("=" * 60)
    print("Aucune trace, dans toute la documentation communautaire existante,")
    print("d'un test confirmé avec une distance NÉGATIVE envoyée à SetMotor -")
    print("l'auteur original n'a documenté que des valeurs positives (\"drive")
    print("forward\"). Ce test isole UNE SEULE roue (l'autre explicitement")
    print("désactivée) pour observer précisément ce qui se passe : la roue")
    print("recule-t-elle vraiment, reste-t-elle immobile (valeur ignorée),")
    print("ou avance-t-elle quand même (signe ignoré) ?\n")

    confirm = input("Robot sur sol dégagé, prêt ? (o/N) ")
    if confirm.lower() != 'o':
        print("Annulé.")
        return

    cal.send_cmd("pause_polling")
    time.sleep(1)
    cal.send_cmd("raw:TestMode on")
    time.sleep(0.3)

    before = cal.sample_wheels()
    if not before:
        print(err("Pas de lecture GetMotor initiale, abandon."))
        cal.send_cmd("raw:TestMode off")
        cal.send_cmd("resume_polling")
        return
    print(f"Avant : left={before['left_mm']}mm right={before['right_mm']}mm")

    test_mm = -100
    print(f"\nEnvoi de SetMotor sur la roue DROITE UNIQUEMENT "
          f"(RWheelDist {test_mm}, gauche explicitement désactivée)...")
    cal.send_cmd(f"raw:SetMotor RWheelDist {test_mm} LWheelDisable Speed 20")
    print("Observe ATTENTIVEMENT la roue droite - avance, recule, ou immobile ?")
    time.sleep(4)

    after = cal.sample_wheels()
    cal.send_cmd("raw:SetMotor LWheelDisable RWheelDisable")
    cal.send_cmd("raw:TestMode off")
    cal.send_cmd("resume_polling")
    if not after:
        print(err("Pas de lecture GetMotor finale, abandon."))
        return
    print(f"Après : left={after['left_mm']}mm right={after['right_mm']}mm")

    delta_right = after['right_mm'] - before['right_mm']
    print(f"\nDelta roue droite : {delta_right:+.1f}mm (commandé : {test_mm}mm)")

    if abs(delta_right) < 5:
        print(warn(
            "Delta quasi nul : la valeur négative semble être IGNORÉE par "
            "le robot (roue traitée comme si aucune commande n'avait été "
            "envoyée pour elle) - explique le comportement asymétrique "
            "observé lors du test d'empattement."
        ))
    elif delta_right < -5:
        print(ok(
            "Delta négatif confirmé : la roue a bien reculé - une distance "
            "négative fonctionne réellement pour ce sens. Le souci observé "
            "précédemment vient peut-être d'autre chose (timing, les deux "
            "roues envoyées dans la MÊME commande plutôt que testées "
            "séparément)."
        ))
    else:
        print(warn(
            "Delta POSITIF malgré une commande négative : le signe semble "
            "ignoré par le robot (traité comme une valeur absolue) - la "
            "roue a avancé au lieu de reculer."
        ))


def main():
    parser = argparse.ArgumentParser(description="Calibration guidée Neato D7 gen3")
    parser.add_argument("--broker", default=read_env_default("MQTT_BROKER", "192.168.10.108"))
    parser.add_argument("--port", type=int, default=int(read_env_default("MQTT_PORT", "1883")))
    parser.add_argument("--prefix", default=read_env_default("MQTT_PREFIX", "neato/robot"))
    parser.add_argument("--with-wheel-base", action="store_true",
                         help="Propose l'étape 3 (empattement, expérimentale) - "
                              "désactivée par défaut par prudence : distance "
                              "négative sur SetMotor jamais confirmée fonctionner "
                              "sur ce robot")
    parser.add_argument("--test-negative", action="store_true",
                         help="Lance uniquement le test diagnostic de distance négative, sans le reste")
    parser.add_argument("-u", "--username", default=read_env_default("MQTT_USERNAME", ""))
    parser.add_argument("-P", "--password", default=read_env_default("MQTT_PASSWORD", ""))
    args = parser.parse_args()

    cal = Calibrator(args.broker, args.port, args.prefix, args.username, args.password)
    time.sleep(1.5)

    try:
        if args.test_negative:
            test_negative_distance(cal)
        else:
            calibrate_drop_threshold(cal)
            calibrate_distance_scale(cal)
            if args.with_wheel_base:
                calibrate_wheel_base(cal)
            else:
                print(warn(
                    "\nÉtape 3 (empattement) sautée par défaut - repose sur une "
                    "distance négative jamais confirmée fonctionner. Relance "
                    "avec --with-wheel-base si tu veux essayer."
                ))
    except KeyboardInterrupt:
        print("\n\nInterrompu. Reprise du round-robin normal par sécurité...")
        cal.send_cmd("resume_polling")
    finally:
        cal.send_cmd("resume_polling")
        time.sleep(0.5)
        cal.close()
        print(ok("\nTerminé. Round-robin normal repris."))


if __name__ == '__main__':
    main()
