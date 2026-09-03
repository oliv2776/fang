#!/usr/bin/env python3
"""
diagnostics.py
Diagnostic automatisé SANS MOUVEMENT du pipeline SLAM Neato D7 gen3,
conçu pour tourner DANS le conteneur Docker et être déclenché à distance
(REST API / Home Assistant).

⚠️ VOLONTAIREMENT LIMITÉ AUX COMMANDES DE LECTURE (GetMotor, GetLDSScan,
GetDigitalSensors, GetAnalogSensors). Le test de SetMotor (déplacement
physique) N'EST PAS inclus ici et ne le sera jamais : ça reste dans
diagnose.py --guided, en local, avec confirmation manuelle à chaque étape.
Un bouton qui fait bouger le robot depuis une appli web, sans garantie que
quelqu'un soit physiquement à côté, est le genre de raccourci qu'on ne
prend pas.

Ce module tourne DANS le conteneur : il voit les données déjà décodées
côté ESP32 (JSON MQTT), pas le flux série brut. Pour comparer au format
RÉEL avant tout parsing (le seul moyen fiable de valider une hypothèse de
format comme GetLDSScan), il faut toujours regarder en parallèle
`esphome logs config/local.yaml` - ce rapport ne remplace pas ça, il
complète.
"""

import json
import time
import threading
from datetime import datetime

try:
    import paho.mqtt.client as mqtt
    HAS_MQTT = True
except ImportError:
    HAS_MQTT = False


# Commandes testées, dans cet ordre. Aucune ne fait bouger le robot.
COMMANDS = ["GetMotor", "GetDigitalSensors", "GetAnalogSensors", "GetLDSScan"]
WAIT_PER_COMMAND_S = 3.0


def run_diagnostics(mqtt_broker: str, mqtt_port: int, mqtt_prefix: str,
                     mqtt_username: str = "", mqtt_password: str = "") -> dict:
    """Exécute la séquence de diagnostic complète et renvoie un rapport
    (dict, sérialisable en JSON). Bloquant pendant environ
    len(COMMANDS) * WAIT_PER_COMMAND_S secondes + connexion."""
    if not HAS_MQTT:
        return {"error": "paho-mqtt non installé dans le conteneur"}

    captured = {cmd: [] for cmd in COMMANDS}
    captured_lock = threading.Lock()
    connected = threading.Event()
    report_errors_holder = {"auth_failed": False}

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(f"{mqtt_prefix}/#", qos=1)
            connected.set()
        elif rc == 5:
            report_errors_holder["auth_failed"] = True

    def on_message(client, userdata, msg):
        # On route chaque message vers la commande actuellement testée,
        # en se basant sur le topic (pas idéal si plusieurs commandes
        # publient sur le même topic en parallèle, mais on est en pause
        # round-robin donc rien d'autre ne devrait publier pendant ce temps).
        topic = msg.topic
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
        except (json.JSONDecodeError, UnicodeDecodeError):
            payload = {"_raw": msg.payload[:200].hex()}
        with captured_lock:
            current = userdata.get('current_cmd')
            if current is not None:
                captured[current].append({"topic": topic, "payload": payload})

    state = {'current_cmd': None}
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION1, userdata=state)
    if mqtt_username:
        client.username_pw_set(mqtt_username, mqtt_password)
    client.on_connect = on_connect
    client.on_message = on_message

    report = {
        "started_at": datetime.now().isoformat(),
        "broker": f"{mqtt_broker}:{mqtt_port}",
        "prefix": mqtt_prefix,
        "results": {},
        "errors": [],
    }

    try:
        client.connect(mqtt_broker, mqtt_port, keepalive=30)
        client.loop_start()

        if not connected.wait(timeout=5.0):
            if report_errors_holder["auth_failed"]:
                report["errors"].append(
                    "Connexion MQTT refusée (rc=5, accès non autorisé) - "
                    "vérifie mqtt_username/mqtt_password"
                )
            else:
                report["errors"].append("Connexion MQTT impossible (timeout 5s)")
            return _finalize(report)

        cmd_topic = f"{mqtt_prefix}/clean_cmd"

        client.publish(cmd_topic, "pause_polling", qos=1)
        time.sleep(1.0)

        for cmd in COMMANDS:
            with captured_lock:
                state['current_cmd'] = cmd
            client.publish(cmd_topic, f"raw:{cmd}", qos=1)
            time.sleep(WAIT_PER_COMMAND_S)
            with captured_lock:
                state['current_cmd'] = None

        client.publish(cmd_topic, "resume_polling", qos=1)
        time.sleep(0.5)

    except Exception as e:
        report["errors"].append(f"Exception pendant le diagnostic: {e}")
        # Toujours tenter de reprendre le polling même en cas d'erreur.
        try:
            client.publish(f"{mqtt_prefix}/clean_cmd", "resume_polling", qos=1)
        except Exception:
            pass
    finally:
        client.loop_stop()
        client.disconnect()

    report["results"] = _analyze(captured)
    return _finalize(report)


def _analyze(captured: dict) -> dict:
    results = {}

    # GetMotor -> attend un message sur .../wheels
    wheels_msgs = [m for m in captured["GetMotor"] if m["topic"].endswith("/wheels")]
    if wheels_msgs:
        p = wheels_msgs[-1]["payload"]
        ok = "left_mm" in p and "right_mm" in p
        results["GetMotor"] = {
            "status": "ok" if ok else "champs manquants",
            "messages_reçus": len(wheels_msgs),
            "dernier_payload": p,
        }
    else:
        results["GetMotor"] = {"status": "AUCUN MESSAGE REÇU", "messages_reçus": 0}

    # GetDigitalSensors / GetAnalogSensors -> tous les deux publient sur .../safety
    for cmd in ("GetDigitalSensors", "GetAnalogSensors"):
        safety_msgs = [m for m in captured[cmd] if m["topic"].endswith("/safety")]
        if safety_msgs:
            p = safety_msgs[-1]["payload"]
            required = {"bump", "cliff", "wheel_extended", "stop"}
            missing = required - set(p.keys())
            results[cmd] = {
                "status": "ok" if not missing else f"champs manquants: {missing}",
                "messages_reçus": len(safety_msgs),
                "dernier_payload": p,
            }
        else:
            results[cmd] = {"status": "AUCUN MESSAGE REÇU", "messages_reçus": 0}

    # GetLDSScan -> attend un message sur .../scan avec 360 points
    scan_msgs = [m for m in captured["GetLDSScan"] if m["topic"].endswith("/scan")]
    if scan_msgs:
        p = scan_msgs[-1]["payload"]
        ranges = p.get("ranges", [])
        n = len(ranges)
        n_valid = sum(1 for r in ranges if r is not None)
        status = "ok (360 points)" if n == 360 else f"ATTENDU 360, reçu {n} - format probablement différent de ce qui est supposé"
        results["GetLDSScan"] = {
            "status": status,
            "messages_reçus": len(scan_msgs),
            "nombre_points": n,
            "points_valides": n_valid,
            "points_null": n - n_valid,
            "note": (
                "Ce rapport montre les données APRÈS parsing côté ESP32. "
                "Si n != 360 ou si tous les points sont null, compare avec "
                "`esphome logs` pendant un test manuel pour voir la vraie "
                "réponse brute du robot et ajuster le parsing dans "
                "config/comp/gen3.yaml (branche GetLDSScan)."
            ),
        }
    else:
        results["GetLDSScan"] = {
            "status": "AUCUN MESSAGE REÇU",
            "messages_reçus": 0,
            "note": "Soit la commande n'a pas été envoyée/reçue, soit le parsing "
                    "GetLDSScan ne produit jamais de message MQTT valide.",
        }

    return results


def _finalize(report: dict) -> dict:
    report["finished_at"] = datetime.now().isoformat()
    return report


def report_to_markdown(report: dict) -> str:
    lines = [
        "# Rapport de diagnostic SLAM Neato",
        "",
        f"Démarré : {report.get('started_at')}",
        f"Terminé : {report.get('finished_at')}",
        f"Broker : {report.get('broker')}  Préfixe : {report.get('prefix')}",
        "",
    ]

    if report.get("errors"):
        lines.append("## Erreurs")
        for e in report["errors"]:
            lines.append(f"- ⚠️ {e}")
        lines.append("")

    lines.append("## Résultats par commande")
    for cmd, res in report.get("results", {}).items():
        status = res.get("status", "?")
        icon = "✅" if status.startswith("ok") else "❌"
        lines.append(f"\n### {icon} {cmd}")
        lines.append(f"- Statut : {status}")
        lines.append(f"- Messages reçus : {res.get('messages_reçus', 0)}")
        if "note" in res:
            lines.append(f"- Note : {res['note']}")
        if "dernier_payload" in res:
            lines.append(f"- Dernier payload : `{json.dumps(res['dernier_payload'])}`")
        if "nombre_points" in res:
            lines.append(
                f"- Points : {res['nombre_points']} total, "
                f"{res['points_valides']} valides, {res['points_null']} null"
            )

    lines.append(
        "\n---\n"
        "Rappel : ce rapport ne teste PAS SetMotor (déplacement physique) - "
        "ça reste volontairement manuel, via `diagnose.py --guided` en local, "
        "robot supervisé."
    )
    return "\n".join(lines)
