#!/bin/bash
# Script de test de connectivité pour le Neato D7 gen3
# À exécuter sur le serveur Ubuntu AVANT de lancer le container SLAM
# Usage : ./test_connectivity.sh [IP_ROBOT] [PORT_MQTT]

ROBOT_IP="${1:-192.168.10.126}"
MQTT_PORT="${2:-1883}"
REST_PORT="${3:-2000}"

echo "=========================================="
echo "  Test de connectivité - Neato D7 gen3"
echo "  IP robot/ESP32 : $ROBOT_IP"
echo "  Port MQTT        : $MQTT_PORT"
echo "  Port REST        : $REST_PORT"
echo "=========================================="

# --- 1. Ping ---
echo ""
echo "[1/5] Ping $ROBOT_IP ..."
if ping -c 3 -W 2 "$ROBOT_IP" > /dev/null 2>&1; then
    echo "    [OK] $ROBOT_IP répond au ping"
else
    echo "    [FAIL] $ROBOT_IP ne répond pas au ping"
    echo "    -> Vérifier que l'ESP32 est branché et sur le même réseau"
    exit 1
fi

# --- 2. Port MQTT ---
echo ""
echo "[2/5] Test port MQTT $ROBOT_IP:$MQTT_PORT ..."
if command -v nc > /dev/null 2>&1; then
    if nc -z -w 3 "$ROBOT_IP" "$MQTT_PORT" 2>/dev/null; then
        echo "    [OK] Port MQTT $MQTT_PORT ouvert"
    else
        echo "    [FAIL] Port MQTT $MQTT_PORT fermé"
        echo "    -> Vérifier que le broker MQTT tourne sur l'ESP32 ou le serveur"
        echo "    -> Si le broker est sur le serveur, décommenter le service 'mqtt' dans docker-compose.yml"
        exit 1
    fi
else
    echo "    [SKIP] 'nc' non installé, on passe"
fi

# --- 3. Test MQTT avec mosquitto_sub ---
echo ""
echo "[3/5] Test abonnement MQTT (5s) ..."
if command -v mosquitto_sub > /dev/null 2>&1; then
    echo "  Écoute neato/robot/# pendant 5s..."
    timeout 5 mosquitto_sub -h "$ROBOT_IP" -p "$MQTT_PORT" -t "neato/robot/#" -v 2>/dev/null
    RC=$?
    if [ $RC -eq 124 ]; then
        echo "    [OK] mosquitto_sub a tourné 5s (aucun message reçu, normal si robot idle)"
    elif [ $RC -eq 0 ]; then
        echo "    [OK] Messages MQTT reçus"
    else
        echo "    [FAIL] mosquitto_sub a échoué (code $RC)"
        echo "    -> Vérifier que le broker MQTT est accessible"
        exit 1
    fi
else
    echo "    [SKIP] mosquitto_sub non installé"
    echo "    -> Installer avec : sudo apt install mosquitto-clients"
fi

# --- 4. Test API REST (si le container SLAM tourne déjà) ---
echo ""
echo "[4/5] Test API REST sur :$REST_PORT ..."
if command -v curl > /dev/null 2>&1; then
    HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" "http://localhost:$REST_PORT/api/status" 2>/dev/null)
    if [ "$HTTP_CODE" = "200" ]; then
        echo "    [OK] API REST répond sur :$REST_PORT"
        echo "  Statut :"
        curl -s "http://localhost:$REST_PORT/api/status" | python3 -m json.tool 2>/dev/null || \
            curl -s "http://localhost:$REST_PORT/api/status"
        echo ""
        echo "  Pose robot :"
        curl -s "http://localhost:$REST_PORT/api/robot/pose" | python3 -m json.tool 2>/dev/null || \
            curl -s "http://localhost:$REST_PORT/api/robot/pose"
    else
        echo "    [INFO] API REST pas encore active (code HTTP: $HTTP_CODE)"
        echo "    -> Lancer : cd slam && docker compose up -d --build"
    fi
else
    echo "    [SKIP] curl non installé"
fi

# --- 5. Test WebSocket ---
echo ""
echo "[5/5] Test WebSocket sur :$REST_PORT ..."
if command -v python3 > /dev/null 2>&1; then
    python3 -c "
import asyncio, websockets, sys
async def test():
    try:
        async with websockets.connect('ws://localhost:2001', open_timeout=3) as ws:
            print('    [OK] WebSocket connecté sur :2001')
    except Exception as e:
        print(f'    [INFO] WebSocket pas encore actif: {e}')
        print('    -> Lancer : cd slam && docker compose up -d --build')
asyncio.run(test())
" 2>/dev/null || echo "    [SKIP] websockets non installé"
else
    echo "    [SKIP] python3 non installé"
fi

echo ""
echo "=========================================="
echo "  Tests terminés"
echo "=========================================="
echo ""
echo "Prochaines étapes :"
echo "    1. cd slam"
echo "    2. cp .env.example .env"
echo "    3. docker compose up -d --build"
echo "    4. docker logs -f neato-slam"
echo "    5. curl http://localhost:$REST_PORT/api/status"
echo ""
