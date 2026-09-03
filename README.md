# Documentation — Projet `fang` (oliv2776)

Dépôt : https://github.com/oliv2776/fang — fork de [vacuula/fang](https://github.com/vacuula/fang) *(Neato Brainslug)*

---

## Vue d'ensemble

Le projet a **deux couches indépendantes**, à ne pas confondre :

1. **Couche « v1 » (stable)** — contrôle local d'un aspirateur Neato (gen2/gen3) via un **ESP32 flashé avec ESPHome**, branché sur le port debug UART du robot. Fonctionne seule, avec ou sans Home Assistant. C'est la base historique du projet (`neato-brainslug`), **mature et supportée**.
2. **Couche « v2 » (en développement actif, expérimentale)** — ajoutée par ce fork : un **conteneur Docker ROS2 Humble + slam_toolbox** qui reçoit, via **MQTT**, les données odométrie/LiDAR/sécurité remontées par l'ESP32, pour faire de la cartographie (SLAM) et, à terme, du nettoyage par zone piloté par Nav2.

D'après `status.md` : la v1 est *« will always maintain and get support »*, alors que la v2 (SLAM) est explicitement indiquée comme un **work in progress**, avec une limitation importante déjà identifiée par l'auteur : si le robot sort de la zone couverte par la carte, la seule action possible aujourd'hui est de le repositionner manuellement.

Schéma :

```
Robot Neato (port debug UART)
        │
        ▼
     ESP32 (ESPHome, config/local.yaml)
        │
        ├── Interface web embarquée (http://neato-vacuum.local)
        ├── API native ESPHome ──► Home Assistant (intégration + entités)
        └── MQTT (broker) ──► Conteneur Docker "neato-slam"
                                   ├── slam_toolbox (cartographie)
                                   ├── Nav2 (optionnel, NAV2_ENABLED)
                                   ├── slam_bridge.py (MQTT ↔ ROS2)
                                   └── slam_server.py (API REST :2000 + WebSocket :2001)
                                              │
                                              └──► Carte Lovelace HA (neato-zone-map-card.js)
```

⚠️ **Important sur le broker MQTT** : ce projet ne fournit pas de broker MQTT « clé en main » actif par défaut — il en faut un sur votre réseau (Mosquitto par exemple), joignable à la fois par l'ESP32 et par le conteneur Docker. Un service `mqtt` (image `eclipse-mosquitto:2`) est fourni **en commentaire** dans `slam/docker-compose.yml`, à décommenter si vous n'en avez pas déjà un.

---

## 1. Serveur Docker (SLAM / v2)

### 1.1 Prérequis

- Docker Engine + plugin Docker Compose (`docker compose`, syntaxe V2) sur la machine hôte.
- Un broker **MQTT** accessible à la fois par l'ESP32 et par le serveur Docker (Mosquitto, ou tout broker existant chez vous).
- L'ESP32 déjà flashé et connecté au robot (voir parties 3 et 4), car le SLAM dépend entièrement des données qu'il publie sur MQTT.

### 1.2 Structure des fichiers (`slam/`)

```
slam/
├── Dockerfile              # ROS2 Humble + slam_toolbox + Nav2 + dépendances Python
├── docker-compose.yml      # Service "slam", réseau host, variables d'environnement
├── entrypoint.sh           # Lance tous les processus ROS2/Python + watchdog auto-restart
├── requirements.txt        # paho-mqtt, websockets, flask, flask-cors, numpy
├── .env.example            # Modèle de configuration à copier en .env
├── config/
│   ├── slam_toolbox.yaml   # Paramètres slam_toolbox (frames, topic scan...)
│   ├── nav2_params.yaml    # Paramètres Nav2 (costmaps, contrôleur, planificateur)
│   ├── robot_params.yaml   # Paramètres ekf_node (fusion odométrie)
│   ├── rsp_params.yaml     # robot_state_publisher (TF base_link -> laser_link)
│   └── robot.urdf          # Modèle physique du robot
├── src/
│   ├── slam_bridge.py      # Pont MQTT <-> topics ROS2 (/odom, /scan, /cmd_vel)
│   ├── slam_server.py      # API REST (:2000) + WebSocket (:2001) pour l'interface web
│   ├── coverage_planner.py # Génère un parcours de nettoyage en zigzag via Nav2 (v2, 1ʳᵉ version)
│   └── diagnostics.py      # Utilisé par l'endpoint /api/diagnose/run
├── calibrate.py            # Script de calibration guidée (à lancer en LOCAL, hors Docker)
├── diagnose.py              # Script de diagnostic MQTT (à lancer en LOCAL, hors Docker)
└── test_connectivity.sh    # Script de test réseau avant de lancer le conteneur
```

### 1.3 Configuration (`.env`)

```bash
cd slam
cp .env.example .env
```

Contenu de `.env.example` à adapter :

```bash
# IP de l'ESP32 (Brainslug) sur le robot
MQTT_BROKER=192.168.10.126
MQTT_PORT=1883
MQTT_PREFIX=neato/robot
SLAM_MODE=mapping          # ou "localize" une fois la carte construite
REST_PORT=2000
WS_PORT=2001
ROSBRIDGE_PORT=2002
WS_IN_PORT=2003
ROBOT_TYPE=d7_gen3

# Nav2 / coverage_planner — désactivé par défaut, à activer seulement
# après avoir validé GetMotor/GetLDSScan/GetDigitalSensors/GetAnalogSensors
# et testé SetMotor en local (voir slam/diagnose.py --guided)
NAV2_ENABLED=false
CMD_VEL_OUT_TOPIC=neato/robot/cmd_vel_out
MQTT_CLEAN_CMD_TOPIC=neato/robot/clean_cmd
ROBOT_RADIUS_M=0.20
ROW_SPACING_M=0.32
```

> ⚠️ `MQTT_BROKER` doit correspondre à l'adresse de votre **broker MQTT** (pas forcément l'IP de l'ESP32 lui-même si votre broker tourne ailleurs — mais dans la config par défaut de ce projet, c'est bien pensé pour être l'IP de l'ESP32/robot). Cette valeur doit être identique à celle utilisée côté ESPHome dans `config/comp/slam-odom.yaml` (`mqtt_broker`).

### 1.4 Test de connectivité avant lancement

Un script est fourni pour valider la chaîne réseau **avant** de démarrer le conteneur :

```bash
cd slam
./test_connectivity.sh [IP_ROBOT] [PORT_MQTT] [PORT_REST]
# valeurs par défaut : 192.168.10.126 1883 2000
```

Il vérifie dans l'ordre : ping de l'ESP32 → port MQTT ouvert → réception de messages sur `neato/robot/#` (nécessite `mosquitto-clients` : `sudo apt install mosquitto-clients`) → API REST du conteneur (si déjà lancé) → WebSocket (si déjà lancé).

### 1.5 Construction et lancement du conteneur

```bash
cd slam
docker compose up -d --build
docker logs -f neato-slam
```

**Via Portainer** : `Stacks` → `Add stack` → coller le contenu de `slam/docker-compose.yml` (le build utilise le contexte `..` donc le dossier parent `fang/` doit être accessible — sinon construisez l'image manuellement puis référencez-la).

Le `docker-compose.yml` définit :

```yaml
services:
  slam:
    build:
      context: ..
      dockerfile: slam/Dockerfile
    container_name: neato-slam
    restart: unless-stopped
    network_mode: host
    environment:
        - ROS_DOMAIN_ID=0
        - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        - MQTT_BROKER=${MQTT_BROKER:-192.168.10.126}
        - MQTT_PORT=${MQTT_PORT:-1883}
        - MQTT_PREFIX=${MQTT_PREFIX:-neato/robot}
        - SLAM_MODE=${SLAM_MODE:-mapping}
        - REST_PORT=${REST_PORT:-2000}
        - WS_PORT=${WS_PORT:-2001}
        - ROSBRIDGE_PORT=${ROSBRIDGE_PORT:-2002}
        - WS_IN_PORT=${WS_IN_PORT:-2003}
        - ROBOT_TYPE=${ROBOT_TYPE:-d7_gen3}
        - NAV2_ENABLED=${NAV2_ENABLED:-false}
        - CMD_VEL_OUT_TOPIC=${CMD_VEL_OUT_TOPIC:-neato/robot/cmd_vel_out}
        - MQTT_CLEAN_CMD_TOPIC=${MQTT_CLEAN_CMD_TOPIC:-neato/robot/clean_cmd}
        - ROBOT_RADIUS_M=${ROBOT_RADIUS_M:-0.20}
        - ROW_SPACING_M=${ROW_SPACING_M:-0.32}
    volumes:
        - ./maps:/app/maps
        - ./logs:/app/logs

  # Broker MQTT — décommenter si pas déjà installé sur le serveur
  # mqtt:
  #   image: eclipse-mosquitto:2
  #   container_name: neato-mqtt
  #   restart: unless-stopped
  #   ports:
  #       - "1883:1883"
  #       - "9001:9001"
  #   volumes:
  #       - ./mosquitto/config:/mosquitto/config
  #       - ./mosquitto/data:/mosquitto/data
  #       - ./mosquitto/log:/mosquitto/log
```

Le conteneur tourne en `network_mode: host` (pas de mapping de ports nécessaire, mais cela ne fonctionne que sous Linux). Les cartes générées sont persistées dans `slam/maps/` et les logs dans `slam/logs/` (montés en volumes).

**Processus lancés par `entrypoint.sh`** (avec redémarrage automatique toutes les 10 s en cas de plantage) :
- `robot_state_publisher` (TF `base_link` → `laser_link`)
- `ekf_node` (fusion odométrie)
- `slam_toolbox` (mode `mapping` ou `localize` selon `SLAM_MODE`)
- Nav2 (`controller_server`, `planner_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`, `lifecycle_manager`) + `coverage_planner.py` — **uniquement si `NAV2_ENABLED=true`**
- `slam_bridge.py` (pont MQTT ↔ ROS2)
- `slam_server.py` (API REST + WebSocket)
- `rosbridge_server` (WebSocket ROS2 → JSON)

### 1.6 API REST exposée par `slam_server.py`

| Méthode | Endpoint | Description |
|---|---|---|
| `GET` | `/api/status` | `{slam_running, map_size, last_update}` |
| `GET` | `/api/robot/pose` | `{x, y, theta}` — pose lue depuis le TF `map → base_link` |
| `GET` | `/api/map` | Carte complète au format JSON (grille d'occupation) |
| `POST` | `/api/slam/start` | Démarre/reprend le suivi du mapping |
| `POST` | `/api/slam/stop` | Marque le mapping comme arrêté |
| `POST` | `/api/slam/save` | Sauvegarde la carte courante dans `/app/maps/map_<timestamp>.yaml` |
| `GET` | `/api/safety` | `{stop: bool}` — état de l'arrêt de sécurité |
| `POST` | `/api/clean/start` | Démarre le nettoyage (refusé avec `409` si `safety_stop` actif) |
| `POST` | `/api/clean/stop` | Arrête le nettoyage |
| `POST` | `/api/clean/zone` | Corps JSON `{"polygon": [[x,y], ...]}` (≥3 points) — nettoyage d'une zone précise |
| `POST` | `/api/diagnose/run?format=md\|json` | Lance un diagnostic MQTT (~15 s) et renvoie un rapport téléchargeable |
| `WS` | `ws://<ip>:2001` | Flux temps réel : `{"type":"map",...}`, `{"type":"pose",...}`, `{"type":"safety_stop",...}` |

Test rapide :
```bash
curl http://localhost:2000/api/status
curl http://localhost:2000/api/robot/pose
curl -X POST http://localhost:2000/api/slam/save
```

### 1.7 Calibration et diagnostic (à lancer **en local**, hors Docker)

Ces deux scripts communiquent en MQTT directement (pas besoin de ROS2/Docker pour les exécuter) et **ne doivent jamais être déclenchables à distance** (ils font bouger le robot ou demandent une mesure physique) :

```bash
pip3 install paho-mqtt --break-system-packages

# Diagnostic — mode écoute passive par défaut
python3 slam/diagnose.py --broker 192.168.10.126

# Diagnostic — mode test guidé (teste une commande à la fois : GetMotor, GetLDSScan, GetDigitalSensors, GetAnalogSensors)
python3 slam/diagnose.py --broker 192.168.10.126 --guided

# Calibration guidée (3 étapes indépendantes, Ctrl+C possible entre deux) :
#   1. Seuil de détection de vide (g_cal_drop_threshold_mm)
#   2. Échelle de distance SetMotor (g_cal_dist_scale)
#   3. Empattement / wheel_base (optionnel, expérimental — désactivé par défaut)
python3 slam/calibrate.py --broker 192.168.10.126
python3 slam/calibrate.py --broker 192.168.10.126 --skip-wheel-base
```
Les valeurs calibrées sont envoyées par MQTT vers des variables globales ESPHome **persistées en flash** (`g_cal_*`) — inutile de reflasher l'ESP32 entre deux essais.

### 1.8 ⚠️ Limitations connues (à lire avant d'activer Nav2/nettoyage automatique)

- **Le format `GetLDSScan` (scan LiDAR) n'a pas été vérifié sur un robot réel** : le parsing dans `config/comp/gen3.yaml` suppose un format standard documenté publiquement, mais aucune capture réelle n'a confirmé l'ordre des colonnes. À valider vous-même avec `esphome logs` en `TestMode` avant de faire confiance aux données publiées.
- **Un seul fil UART partagé** entre toutes les commandes (`GetMotor`, `GetLDSScan`, capteurs de sécurité, `SetMotor`...). Résultat : un scan + une mise à jour d'odométrie environ toutes les 1-2 secondes, pas de flux temps réel à 10 Hz. Le robot doit rester **lent** pendant le mapping.
- **Sécurité** : `GetDigitalSensors` (chocs, roue soulevée) et `GetAnalogSensors` (détection de vide) sont vérifiés bien plus souvent que le reste dans le round-robin ESPHome. Dès qu'un problème est détecté, un `SetMotor` d'arrêt est envoyé **immédiatement**, et `g_safety_stop` reste actif tant qu'il n'est pas explicitement réarmé (payload MQTT `"clear_safety_stop"` sur le topic de commande — voir §1.9). **Pas de reprise automatique.**
- `NAV2_ENABLED=false` par défaut : n'activez la navigation autonome (Nav2 + `coverage_planner.py`) qu'après avoir validé les commandes de base en mode guidé.
- `coverage_planner.py` est explicitement documenté comme une **première version** : il couvre toute la carte connue (pas de sélection par pièce autre que via `/api/clean/zone`), n'a pas de logique de retour au socle, et ne redémarre pas automatiquement après un arrêt de sécurité.

### 1.9 Commandes MQTT manuelles (topic `MQTT_CLEAN_CMD_TOPIC`, par défaut `neato/robot/clean_cmd`)

Payloads texte brut (pas de JSON) acceptés côté ESPHome (`config/comp/slam-odom.yaml`) :

| Payload | Effet |
|---|---|
| `clean_on` | Active la brosse + l'aspirateur (`Clean CleaningEnable`) |
| `clean_off` | Désactive brosse + aspirateur |
| `clear_safety_stop` | Réarme manuellement l'arrêt de sécurité (après vérification physique du robot) |
| `pause_polling` | Suspend le round-robin automatique — **suspend aussi la sécurité automatique**, à utiliser robot immobile/supervisé uniquement |
| `resume_polling` | Reprend le round-robin normal |
| `raw:<commande>` | Envoie une commande UART brute une seule fois (ex. `raw:GetLDSScan`) |
| `cal:drop_threshold:<mm>` | Règle le seuil de détection de vide |
| `cal:dist_scale:<facteur>` | Règle le facteur d'échelle de distance |
| `cal:wheel_base:<m>` | Règle l'empattement (expérimental) |

---

## 2. Installer les add-ons nécessaires dans Home Assistant

*(Étape 1 du guide officiel `install-ha.md`)*

### 2.1 Add-on ESPHome Device Builder
1. `Paramètres` → `Modules complémentaires` → `Boutique des modules` → rechercher **« ESPHome Device Builder »**.
   - Ou lien direct : `https://my.home-assistant.io/redirect/supervisor_addon/?addon=5c53de3b_esphome&repository_url=https%3A%2F%2Fgithub.com%2Fesphome%2Fhome-assistant-addon`
2. Cliquez sur **Installer**.
3. Activez de préférence **« Ajouter à la barre latérale »** et **« Démarrer au démarrage »**.

### 2.2 HACS
Si vous n'avez pas encore HACS : https://www.hacs.xyz/docs/use/

Puis installez (recherche par ID) :

| Intégration | ID | Rôle |
|---|---|---|
| `button-card` | `146194325` | Carte bouton personnalisable |
| `browser_mod` | `194140521` | Popups (paramètres, appui long sur « spot clean »). Ajoutez ensuite l'intégration `Browser Mod` dans `Paramètres` → `Appareils et services` → `Ajouter une intégration`. Pas besoin d'enregistrer votre navigateur comme appareil. |

Rafraîchissement forcé du navigateur après installation (`Ctrl + Shift + R`), redémarrez Home Assistant si besoin.

---

## 3. Configurer ESPHome et flasher l'ESP32

Le fichier de configuration central est **`config/local.yaml`**, structuré en `packages` (chaque brique est un fichier YAML séparé) :

```yaml
substitutions:
  name: neato-vacuum        # nom de l'appareil — à garder cohérent avec la carte HA
  comment: ""
  infointerval: 2sec
  chargerinterval: 2min
  ota_password: !secret neato_vacuum_ota
  wifi_ssid: !secret wifi_ssid
  wifi_password: !secret wifi_password
  ha_encryption_key: !secret neato_vacuum_api
  # uart_tx: 17   # à décommenter si vous devez changer les pins UART
  # uart_rx: 16

packages:
  - !include boards/esp32.yaml        # <- carte matérielle : esp32 / esp32c3 / esp32s3 / esp32s2
  # - !include boards/esp32c3.yaml
  # - !include boards/esp32s3.yaml
  # - !include boards/esp32s2.yaml

  - !include comp/ha.yaml             # intégration Home Assistant (ou comp/no-ha.yaml sans HA)
  # - !include comp/no-ha.yaml

  - !include comp/webserver.yaml      # interface web embarquée (port 80)
  - !include comp/gen3.yaml           # logique robot gen3 (ou comp/gen2.yaml)
  # - !include comp/gen2.yaml

  - !include comp/slam-odom.yaml      # pont MQTT vers le conteneur Docker SLAM (facultatif)
```

**Points d'attention** :
- **Une seule ligne carte** doit rester décommentée dans `boards/`.
- **Une seule ligne génération** (`gen2.yaml` / `gen3.yaml`).
- `comp/ha.yaml` **ou** `comp/no-ha.yaml`, pas les deux.
- Retirez (commentez) `comp/slam-odom.yaml` si vous n'utilisez pas le conteneur Docker SLAM.
- Il existe une variante `config/remote.yaml` qui télécharge les mêmes fichiers `packages` directement depuis le dépôt GitHub `philip2809/neato-brainslug` au lieu de fichiers locaux — pratique si vous ne voulez pas cloner tout le dépôt, mais moins simple à personnaliser.

### 3.1 Secrets ESPHome

Créez/éditez le fichier `secrets.yaml` de votre installation ESPHome :

```yaml
# À générer sur https://esphome.io/components/api/#api-key
neato_vacuum_api: "<API_KEY>"
# À générer sur https://bitwarden.com/password-generator/
neato_vacuum_ota: "<OTA_PASSWORD>"

wifi_ssid: "<WIFI_SSID>"
wifi_password: "<WIFI_PASSWORD>"
```

### 3.2 Si vous utilisez le conteneur Docker SLAM

Dans `config/comp/slam-odom.yaml`, adaptez les substitutions réseau pour qu'elles **correspondent exactement** à votre `slam/.env` :

```yaml
substitutions:
  mqtt_broker: "192.168.10.126"      # = MQTT_BROKER dans slam/.env
  mqtt_port: "1883"                  # = MQTT_PORT
  mqtt_wheels_topic: "neato/robot/wheels"
  mqtt_scan_topic: "neato/robot/scan"
  mqtt_safety_topic: "neato/robot/safety"
  mqtt_cmd_vel_topic: "neato/robot/cmd_vel_out"   # = CMD_VEL_OUT_TOPIC
  mqtt_clean_cmd_topic: "neato/robot/clean_cmd"   # = MQTT_CLEAN_CMD_TOPIC
```
Si votre broker MQTT demande une authentification, décommentez dans le même fichier :
```yaml
mqtt:
  - id: neato_mqtt
    broker: ${mqtt_broker}
    port: ${mqtt_port}
    username: !secret mqtt_username
    password: !secret mqtt_password
    discovery: false
```

### 3.3 Compiler et flasher via ESPHome Builder (Home Assistant)

1. Dans ESPHome Builder, ajoutez un nouvel appareil, importez/collez le contenu de `config/local.yaml` (adapté comme ci-dessus).
2. Cliquez sur **« Install »** → **« Téléchargement manuel »** → format **« Factory »**.
3. Ouvrez [ESPHome Web](https://web.esphome.io/) (navigateur Chromium, WebSerial).
4. Reliez l'ESP32 en USB (testez un autre câble s'il est « charge uniquement »).
5. Passez l'ESP32 en mode bootloader (bouton **BOOT**, ou `GPIO0` relié à `GND` s'il n'y en a pas).
6. Sélectionnez le port série, uploadez le fichier `.factory.bin`.
7. Vérifiez que l'interface web répond : `http://neato-vacuum.local` ou `http://neato-vacuum.lan`.

### 3.4 Méthode alternative sans Home Assistant : Web Flasher

Images précompilées disponibles pour `ESP32`, `ESP32-S3`, `ESP32-C3` (et `ESP32-C6` dans `brainslug-tools/public/webflash/` et `docs/webflash/`) pour gen2 **et** gen3.

1. https://tools.vacuula.phma.dev/
2. Connectez l'ESP32 en USB, sélectionnez la génération du robot → **Connect**.
3. Sélectionnez votre appareil série, puis **« Install Neato Brainslug »**.
4. Configurez le Wi-Fi directement dans l'outil si possible, sinon connectez-vous au réseau diffusé par l'ESP :
   - SSID : `neato-brainslug`
   - Mot de passe : `make-it-suck-again`
   - Portail captif automatique, ou manuellement `http://192.168.4.1/`.
5. Vérifiez l'accès via `http://neato-vacuum.local` ou `http://neato-vacuum.lan`.

*(Cette méthode standalone ne permet pas d'activer facilement le pont MQTT/SLAM — pour l'utiliser avec le conteneur Docker, privilégiez la méthode ESPHome Builder avec `comp/slam-odom.yaml`.)*

### 3.5 Construire vos propres images (optionnel, pour développeurs)

Le dossier `build/` contient un environnement Docker autonome pour compiler ESPHome sans passer par Home Assistant :

```bash
cd build
docker compose up -d          # lance le conteneur esphome_builder (ghcr.io/esphome/esphome)
```
- `build/build.sh` compile les 6 variantes officielles (gen2/gen3 × esp32/esp32s3/esp32c3) et produit les fichiers `.factory.bin`/`.ota.bin` dans `config/prebuilt/`.
- `build/dev.sh [upload]` compile `config/local.yaml`, produit `dev.factory.bin`/`dev.ota.bin`, et peut uploader directement en OTA vers l'IP du robot si vous passez l'argument `upload` (l'IP cible est en dur dans le script — à adapter).

---

## 4. Connecter l'ESP32 au robot

### 4.1 Test préalable (avant montage définitif)

| Robot | ESP |
|---|---|
| RX | GPIO17 (TX de l'ESP) |
| 3.3V | 3.3V |
| TX | GPIO16 (RX de l'ESP) |
| GND | GND |

> Sur ESP32-C3 : TX = GPIO7, RX = GPIO6. Règle UART : TX se relie toujours à RX et inversement.

Robot sous tension → la page web de l'ESP32 doit afficher les données du robot. Testez les boutons, et sur gen3, testez la conduite manuelle (**bumper démonté = prudence**).

### 4.2 Installation physique définitive

Guides dédiés :
- **Gen2** → `install-esp-device-gen2.md`
- **Gen3** → `install-esp-device-gen3.md`

**Résumé gen3** (le plus documenté) :

**Matériel recommandé**
- Câble **JST-XH vers DuPont**
- Colle chaude, ruban électrique, ruban Kapton (anti court-circuit)
- Embout Torx T10 long + tournevis cruciforme
- Cutter (si montage « derrière le pare-chocs, découpe »)

**4 méthodes de montage**, du meilleur au moins recommandé :
1. **Interne** (recommandé)
2. **Derrière le pare-chocs, avec découpe plastique**
3. **Derrière le pare-chocs, petit ESP32 (type C3) soudé**
4. **Externe** (non recommandé — risque d'erreur « deck debris »)

**Étapes clés (montage interne)** :
1. Protégez le plan de travail.
2. Retirez bac à poussière, brosse principale (+ latérale), pare-chocs.
3. Retirez les 2 vis du compartiment batterie, **débranchez la batterie**.
4. Retirez les 6 vis Torx sous le robot, 2 vis côté bac à poussière, puis le capot supérieur.
5. **Revérifiez que la batterie est débranchée** (aucune LED allumée).
6. Branchez le JST-XH sur le port debug (avant-gauche de la carte), poussé bien à fond.
7. Placez l'ESP32 à droite de la carte principale, connectez les fils.
8. Enroulez l'ESP32 de ruban Kapton, sécurisez les connexions.
9. Fixez le câble JST-XH avec du ruban électrique.
10. Remontez le capot, refixez les vis, second tour de ruban électrique.
11. Remontez les 6 vis Torx du dessous.
12. Rebranchez la batterie, refermez.
13. Rallumez le robot.

⚠️ Consultez les photos du guide avant de démonter : `install-esp-device-gen3.md`.

---

## 5. Ajouter l'ESP32 dans Home Assistant

1. Robot allumé.
2. `Paramètres` → `Appareils et services` → `Ajouter une intégration` → **« ESPHome »**.
3. Hostname/IP : `neato-vacuum.local` (ou IP fixe via réservation DHCP).

### 5.1 Carte du tableau de bord

Fichiers disponibles dans [`config/home-assistant/`](https://github.com/oliv2776/fang/tree/main/config/home-assistant) :
- `gen2-card.yaml` / `gen3_card.yaml`
- `gen2-entity.yaml` / `gen3-entity.yaml`

Si vous avez renommé l'appareil : remplacez toutes les occurrences de `neato_vacuum` par votre `entity_id` réel (visible dans `Outils de développement` → `États`, filtre `_fuel_percent`).

**Ajouter la carte** : icône crayon → `Ajouter une carte` → tout en bas → `Manuelle` → coller le contenu.

### 5.2 Entité « vacuum »

Nécessaire pour les automatisations/scripts standards.

1. Installez l'add-on **« File editor »**.
2. Dans `configuration.yaml` :
   ```yaml
   template: !include_dir_merge_list templates/
   ```
3. Créez `templates/vacuums.yaml`, collez le contenu de `gen2-entity.yaml` ou `gen3-entity.yaml` (dupliquez le bloc `- name:` pour plusieurs robots).
4. `Outils de développement` → `YAML` → `Vérifier la configuration` → recharger.

### 5.3 Carte de cartographie / nettoyage par zone (v2, SLAM)

Fichier : `HomeAssistant/neato-zone-map-card.js` — carte Lovelace personnalisée qui **affiche la carte SLAM en direct** (via l'API REST du conteneur Docker, pas via le backend Home Assistant) et permet de **dessiner un polygone au clic** pour ne nettoyer qu'une zone précise.

**Installation :**
1. Copiez le fichier dans `www/neato-zone-map-card.js` de votre config Home Assistant (créez le dossier `www/` s'il n'existe pas).
2. `Paramètres` → `Tableaux de bord` → `Ressources` → `Ajouter une ressource` :
   - URL : `/local/neato-zone-map-card.js`
   - Type : **Module JavaScript**
3. Ajoutez la carte au dashboard (mode YAML) :
   ```yaml
   type: custom:neato-zone-map-card
   api_base_url: http://192.168.10.50:2000   # IP:port du conteneur SLAM
   refresh_ms: 4000                          # optionnel, 4000 par défaut
   ```

⚠️ Le navigateur qui affiche le dashboard doit pouvoir joindre `api_base_url` **directement** (même réseau local) — la carte n'a pas de fonctionnement via proxy Home Assistant, c'est volontairement simple. Si vous consultez votre dashboard depuis l'extérieur sans exposer ce port, la carte ne se chargera pas.

### 5.4 Planification automatique

`Paramètres` → `Automatisations et scènes` → `Créer une automatisation` :
- Déclencheur : ex. tous les jours à 08h00.
- Action : envoyer l'événement de démarrage (bouton ESPHome **ou** entité vacuum).

### 5.5 Notifications

Déclencheur : changement d'état du capteur d'erreur (+ alerte pour gen3). Modèle de message :
```
Alert: {{ state_attr('vacuum.template_neato_vacuum', 'alert') }}
Error: {{ state_attr('vacuum.template_neato_vacuum', 'error') }}
```
(retirez la ligne Alert si pas de gen3). Alternative directe : `{{ states('sensor.neato_vacuum_robot_error') }}`

---

## 6. Utilisation sans Home Assistant

Interface web embarquée : `http://neato-vacuum.local`.

### Mise à jour OTA
1. Téléchargez le `.ota.bin` correspondant à votre génération/carte (voir `brainslug-tools/public/webflash/` ou `docs/webflash/` dans le dépôt, ou les releases GitHub du projet parent).
2. Page web de l'ESP → section **OTA** → uploadez → **UPDATE**.
3. Attendez le redémarrage, rafraîchissez.

---

## 7. Utiliser les fonctionnalités du robot

### Via l'interface web / Home Assistant (couche v1)
- **Boutons & dernier nettoyage** — gen2 n'a pas de données de dernier nettoyage.
- **Données de base & nettoyage ciblé** — batterie, état docké, erreurs/alertes ; « spot clean » réglable en taille (fiabilité variable sur gen2).
- **Planification (ESP schedule)** — indépendante de Home Assistant, basée sur l'heure interne de l'ESP.
- **Paramètres du robot** (variables selon génération) :
  - Gen3 : mode Navigation `gentle` (évite les objets plus hauts que lui) / `deep` (nettoie les coins en profondeur) — à resélectionner après chaque redémarrage.
  - `Intense clean` : réduit la distance entre les bandes.
  - `Wall Enable/Follower` : suit les murs en un tour puis nettoie les zones pertinentes ; sans, longe le mur puis nettoie par zones successives.
- **Conduite manuelle** — gen3 uniquement, démarrer le mode manuel avant d'utiliser les boutons.
- **Données détaillées** — champs techniques, sélecteur de fuseau horaire, niveau de log.
- **Fuseau horaire, OTA & infos** — upload OTA, type de firmware (gen2/gen3), logs debug.
- **Logs de debug** — voir `GetState`, `GetErr`, `GetCharger` régulièrement est **normal** (interrogation périodique de l'état du robot).

### Via le conteneur Docker SLAM (couche v2)
- Démarrer/arrêter le suivi du mapping (`/api/slam/start`, `/api/slam/stop`).
- Sauvegarder la carte courante (`/api/slam/save`).
- Consulter la pose en temps réel (`/api/robot/pose`, ou WebSocket `:2001`).
- Démarrer/arrêter un nettoyage global (`/api/clean/start`, `/api/clean/stop`) — bloqué si l'arrêt de sécurité (`safety_stop`) est actif.
- Nettoyer une zone précise en dessinant un polygone (via la carte HA `neato-zone-map-card.js`, ou directement `POST /api/clean/zone`).
- Lancer un diagnostic complet téléchargeable (`/api/diagnose/run`).
- Envoyer des commandes MQTT manuelles (brosse on/off, réarmement sécurité, calibration à chaud — voir §1.9).

---

## 8. Choisir son ESP32

D'après `supported-esp32.md` :
- Modèles recommandés par ESPHome : `ESP32`, `ESP32-S3`, `ESP32-C3` (le projet fournit aussi des images `ESP32-C6`).
- Budget minimum conseillé : **6-8 €/£/$**.
- **Évitez les cartes « SuperMini »** (antenne trop proche de l'électronique → interférences).
- Préférez une carte **blindée**, évitez les antennes céramiques.

**Deux problèmes fréquents identifiés par la communauté :**
1. Cartes bon marché sans protection contre les micro-coupures de tension (« brownout ») → redémarrages intempestifs.
2. Antenne trop proche de l'électronique → interférences / refus de connexion.

Référence : [discussion Reddit sur les cartes C3 SuperMini](https://www.reddit.com/r/esp32/comments/1dsh3b5/warning_some_c3_super_mini_boards_have_a_design/)

---

## 9. État du projet et feuille de route (`status.md`)

| Version | État | Contenu |
|---|---|---|
| **v1** | ✅ Stable, maintenue | Contrôle local de base (v1.2 actuelle) : Connected/D70-D85/XV-series, commandes événementielles, retour au socle et conduite manuelle (D3-D7), version sans Home Assistant complète, entité HA + automatisations. |
| **v1.3** | 🔜 Planifiée | Gestion d'état personnalisée (ui-state/robot-state/capteurs), messages d'erreur gen2, notifications push (hors HA), amélioration des notifications. |
| **v1.4** | 🔜 Planifiée | Traductions, récupération de packages ESPHome directement depuis GitHub. |
| **v2** | 🚧 En développement (ETA indicative : mars 2026) | Ce que documente ce guide : hybride Neato + ROS2, lignes « nogo » et nettoyage par zone. **Limitation connue de l'auteur** : si le robot sort de la zone cartographiée, la seule solution actuelle est de le repositionner manuellement. |
| **v3** | 🔮 Objectif final, sans échéance | Support générique multi-marques via un « driver » par modèle d'aspirateur, navigation entièrement personnalisée. |

---

## 10. FAQ (extraits de `faq.md`)

- **Quel ESP choisir ?** ESP32/ESP32-S3/ESP32-C3, les plus testés. Attention aux cartes ESP32 bas de gamme avec des défauts de composants. Le support ESP8266 est maintenu au mieux mais pourrait mal se comporter avec les v2/v3 (davantage de RAM nécessaire).
- **Pourquoi je vois `GetErr` / `GetState` tout le temps dans les logs ?** Normal — l'état du robot est interrogé par défaut toutes les 2 secondes. `UI_ALERT_INVALID` signifie qu'il n'y a aucune alerte/erreur.
- **Valetudo est-il compatible ?** Non, et ça ne le sera probablement jamais : le blocage vient du *certificate pinning SSL* du firmware Neato (l'IP du serveur cloud codée en dur n'est pas le problème). Même si ce blocage était levé, ce ne serait pas intégré à Valetudo (nécessiterait un paquet Docker séparé type Congatudo, et Neato/Vorwerk ne sont explicitement pas supportés par Valetudo).
- **Peut-on créer son propre firmware pour le robot lui-même (pas l'ESP) ?** Non — les images de firmware Neato sont chiffrées et signées ; même si la signature ne semble pas vérifiée strictement, le chiffrement empêche toute modification.

---

## 11. Récapitulatif des liens utiles

| Ressource | Lien |
|---|---|
| Dépôt (votre fork) | https://github.com/oliv2776/fang |
| Dépôt parent (Neato Brainslug) | https://github.com/vacuula/fang |
| Guide HA complet | `install-ha.md` |
| Guide sans HA | `install-no-ha.md` |
| Install ESP gen2 / gen3 | `install-esp-device-gen2.md` / `install-esp-device-gen3.md` |
| Manuel d'utilisation | `manual.md` |
| FAQ | `faq.md` |
| Cartes ESP32 supportées | `supported-esp32.md` |
| État du projet / roadmap | `status.md` |
| Web Flasher | https://tools.vacuula.phma.dev/ |
| ESPHome Web | https://web.esphome.io/ |
| HACS | https://www.hacs.xyz/docs/use/ |

---

## 12. Aide-mémoire des ports et variables (SLAM)

| Variable `.env` | Défaut | Usage |
|---|---|---|
| `MQTT_BROKER` | `192.168.10.126` | IP du broker MQTT (doit correspondre à `mqtt_broker` dans `slam-odom.yaml`) |
| `MQTT_PORT` | `1883` | Port du broker |
| `MQTT_PREFIX` | `neato/robot` | Préfixe des topics MQTT |
| `SLAM_MODE` | `mapping` | `mapping` (construction de carte) ou `localize` (une fois la carte établie) |
| `REST_PORT` | `2000` | API REST (`slam_server.py`) |
| `WS_PORT` | `2001` | WebSocket sortant (flux carte/pose vers le front) |
| `ROSBRIDGE_PORT` | `2002` | Pont WebSocket ROS2 → JSON |
| `WS_IN_PORT` | `2003` | WebSocket entrant (données robot) |
| `ROBOT_TYPE` | `d7_gen3` | Type de robot |
| `NAV2_ENABLED` | `false` | Active Nav2 + `coverage_planner.py` (⚠️ ne pas activer sans validation préalable, voir §1.8) |
| `CMD_VEL_OUT_TOPIC` | `neato/robot/cmd_vel_out` | Topic de commande de vitesse (Nav2 → ESP32) |
| `MQTT_CLEAN_CMD_TOPIC` | `neato/robot/clean_cmd` | Topic de commandes manuelles (brosse, sécurité, calibration) |
| `ROBOT_RADIUS_M` | `0.20` | Rayon du robot (planification de couverture) |
| `ROW_SPACING_M` | `0.32` | Espacement entre bandes de nettoyage |
