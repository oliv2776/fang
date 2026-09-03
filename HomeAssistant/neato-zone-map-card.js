// neato-zone-map-card.js
//
// Custom Lovelace card Home Assistant : affiche la carte SLAM (via l'API
// REST du conteneur Docker) et permet de dessiner un polygone au clic pour
// ne nettoyer que cette zone.
//
// INSTALLATION :
//   1. Copie ce fichier dans www/neato-zone-map-card.js (dossier www/ de
//      ta config Home Assistant - crée-le s'il n'existe pas).
//   2. Paramètres > Tableaux de bord > Ressources > Ajouter une ressource :
//        URL: /local/neato-zone-map-card.js
//        Type: Module JavaScript
//   3. Ajoute la carte à ton dashboard (YAML) :
//        type: custom:neato-zone-map-card
//        api_base_url: http://192.168.10.50:2000   # IP:port du conteneur SLAM
//
// ⚠️ Le navigateur qui affiche le dashboard doit pouvoir atteindre
// api_base_url directement (même réseau local). Si tu regardes ton
// dashboard depuis l'extérieur sans exposer ce port, la carte ne
// chargera pas - c'est un choix volontaire pour rester simple, pas une
// intégration passant par le backend Home Assistant.
//
// Ce fichier ne dépend d'aucune bibliothèque externe, juste un <canvas>.

// Doit rester aligné avec ZONE_COLORS / ZONE_ICONS dans slam_server.py -
// si tu ajoutes une couleur/icône ici, ajoute-la aussi côté backend,
// sinon le POST /api/zones sera rejeté avec une erreur 400.
const ZONE_COLORS = {
  red: '#e53935', blue: '#1e88e5', green: '#43a047', amber: '#ffb300',
  purple: '#8e24aa', teal: '#00897b', pink: '#d81b60', gray: '#757575',
};
const ZONE_ICONS = {
  sofa: '🛋️', cooking: '🍳', bed: '🛏️', bath: '🛁', door: '🚪',
  box: '📦', plant: '🪴', tv: '📺', stairs: '🪜', 'washing-machine': '🧺',
};

class NeatoZoneMapCard extends HTMLElement {
  setConfig(config) {
    if (!config.api_base_url) {
      throw new Error('neato-zone-map-card: "api_base_url" est requis (ex: http://192.168.10.50:2000)');
    }
    this._config = config;
    this._apiBase = config.api_base_url.replace(/\/$/, '');
    this._refreshMs = config.refresh_ms || 4000;
    this._polygon = []; // points en coordonnées MONDE (mètres), pas pixels
    this._zones = []; // zones nommées enregistrées, récupérées de /api/zones
    this._mapInfo = null;
    this._rendered = false;
    this._safetyStop = false;
    this._statusText = '';
  }

  set hass(hass) {
    this._hass = hass;
    if (!this._rendered) {
      this._render();
      this._rendered = true;
      this._startPolling();
    }
  }

  disconnectedCallback() {
    if (this._pollTimer) clearInterval(this._pollTimer);
  }

  getCardSize() {
    return 8;
  }

  // ------------------------------------------------------------ Rendu initial
  _render() {
    this.attachShadow({ mode: 'open' });
    this.shadowRoot.innerHTML = `
      <style>
        ha-card { padding: 12px; }
        .toolbar { display: flex; gap: 8px; margin-bottom: 8px; flex-wrap: wrap; align-items: center; }
        button {
          padding: 6px 12px; border-radius: 6px; border: none; cursor: pointer;
          font-size: 0.9em; background: var(--primary-color, #03a9f4); color: white;
        }
        button.secondary { background: var(--secondary-background-color, #888); }
        button:disabled { opacity: 0.5; cursor: not-allowed; }
        canvas { width: 100%; max-width: 600px; display: block; margin: 0 auto; cursor: crosshair; background: #222; border-radius: 4px; }
        .status { font-size: 0.85em; margin-top: 6px; color: var(--secondary-text-color, #888); }
        .safety-banner {
          background: #c62828; color: white; padding: 6px 10px; border-radius: 6px;
          margin-bottom: 8px; font-weight: bold; display: none;
        }
        .save-form {
          display: none; gap: 8px; align-items: center; flex-wrap: wrap;
          margin: 8px 0; padding: 8px; border-radius: 6px;
          background: var(--secondary-background-color, #f0f0f0);
        }
        .save-form.visible { display: flex; }
        .save-form input[type="text"] {
          padding: 5px 8px; border-radius: 4px; border: 1px solid #ccc; font-size: 0.9em; flex: 1; min-width: 100px;
        }
        .save-form select { padding: 5px; border-radius: 4px; border: 1px solid #ccc; font-size: 0.9em; }
        .zones-list { display: flex; flex-wrap: wrap; gap: 6px; margin-top: 10px; }
        .zone-chip {
          display: flex; align-items: center; gap: 6px; padding: 4px 8px 4px 6px;
          border-radius: 16px; font-size: 0.85em; color: white; cursor: pointer; border: none;
        }
        .zone-chip .del {
          background: rgba(255,255,255,0.3); border-radius: 50%; width: 16px; height: 16px;
          display: flex; align-items: center; justify-content: center; font-size: 0.8em; cursor: pointer;
        }
        .drive-pad {
          display: none; grid-template-columns: repeat(3, 48px); grid-template-rows: repeat(3, 48px);
          gap: 4px; margin: 10px auto; justify-content: center;
        }
        .drive-pad.visible { display: grid; }
        .drive-pad button {
          font-size: 1.2em; padding: 0; margin: 0; width: 48px; height: 48px;
          user-select: none; touch-action: none;
        }
        .drive-hint { text-align: center; font-size: 0.8em; color: var(--secondary-text-color, #888); display: none; }
        .drive-hint.visible { display: block; }
      </style>
      <ha-card header="Carte Neato - sélection de zone">
        <div class="safety-banner" id="safetyBanner">
          ⚠️ Arrêt de sécurité actif sur le robot - vérifie-le avant de nettoyer
        </div>
        <div class="toolbar">
          <button id="btnClear" class="secondary">Effacer la zone</button>
          <button id="btnCleanZone" disabled>Nettoyer cette zone</button>
          <button id="btnCleanAll" class="secondary">Nettoyer tout</button>
          <button id="btnStop" class="secondary">Arrêter</button>
          <button id="btnDiagnose" class="secondary">Lancer le diagnostic</button>
          <button id="btnScan" class="secondary">Scanner (maj carte)</button>
          <button id="btnExplore" class="secondary">Explorer automatiquement</button>
          <button id="btnDock" class="secondary">Retour au socle</button>
          <button id="btnToggleDrive" class="secondary">Conduite manuelle</button>
        </div>
        <canvas id="mapCanvas" width="600" height="600"></canvas>

        <div class="drive-pad" id="drivePad">
          <div></div>
          <button id="drvUp" title="Avancer">▲</button>
          <div></div>
          <button id="drvLeft" title="Tourner à gauche">◄</button>
          <div></div>
          <button id="drvRight" title="Tourner à droite">►</button>
          <div></div>
          <button id="drvDown" title="Reculer">▼</button>
          <div></div>
        </div>
        <div class="drive-hint" id="driveHint">
          Maintiens le bouton appuyé pour avancer/tourner - relâche pour arrêter.
          Vitesse volontairement lente et bridée par le serveur.
        </div>

        <div class="save-form" id="saveForm">
          <input type="text" id="zoneName" placeholder="Nom de la zone (ex: Salon)" maxlength="30" />
          <select id="zoneColor">
            ${Object.keys(ZONE_COLORS).map(c => `<option value="${c}">${c}</option>`).join('')}
          </select>
          <select id="zoneIcon">
            ${Object.entries(ZONE_ICONS).map(([k, v]) => `<option value="${k}">${v} ${k}</option>`).join('')}
          </select>
          <button id="btnSaveZone">Enregistrer la zone</button>
        </div>

        <div class="zones-list" id="zonesList"></div>

        <div class="status" id="statusText">Chargement de la carte...</div>
      </ha-card>
    `;

    this._canvas = this.shadowRoot.getElementById('mapCanvas');
    this._ctx = this._canvas.getContext('2d');

    this._canvas.addEventListener('click', (e) => this._onCanvasClick(e));
    this.shadowRoot.getElementById('btnClear').addEventListener('click', () => {
      this._polygon = [];
      this._draw();
      this._updateButtons();
    });
    this.shadowRoot.getElementById('btnCleanZone').addEventListener('click', () => this._sendZone());
    this.shadowRoot.getElementById('btnCleanAll').addEventListener('click', () => this._cleanAll());
    this.shadowRoot.getElementById('btnStop').addEventListener('click', () => this._stopCleaning());
    this.shadowRoot.getElementById('btnDiagnose').addEventListener('click', () => this._runDiagnose());
    this.shadowRoot.getElementById('btnSaveZone').addEventListener('click', () => this._saveZone());
    this.shadowRoot.getElementById('btnScan').addEventListener('click', () => this._startScan());
    this.shadowRoot.getElementById('btnExplore').addEventListener('click', () => this._startExplore());
    this.shadowRoot.getElementById('btnDock').addEventListener('click', () => this._returnToDock());
    this.shadowRoot.getElementById('btnToggleDrive').addEventListener('click', () => this._toggleDrivePad());
    this._wireDriveButton('drvUp', 0.1, 0);
    this._wireDriveButton('drvDown', -0.1, 0);
    this._wireDriveButton('drvLeft', 0, 0.4);
    this._wireDriveButton('drvRight', 0, -0.4);

    this._fetchMap();
    this._fetchSafety();
    this._fetchZones();
  }

  _startPolling() {
    this._pollTimer = setInterval(() => {
      this._fetchMap();
      this._fetchSafety();
      this._fetchZones();
    }, this._refreshMs);
  }

  // ------------------------------------------------------------------ Réseau
  async _fetchMap() {
    try {
      const res = await fetch(`${this._apiBase}/api/map`);
      if (res.status === 404) {
        this._setStatus('Pas encore de carte disponible (SLAM en cours de démarrage ?)');
        return;
      }
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      const map = await res.json();
      this._mapInfo = map;
      this._draw();
      this._setStatus(`Carte : ${map.width}x${map.height} cellules, résolution ${map.resolution}m`);
    } catch (e) {
      this._setStatus(`Erreur de connexion au conteneur SLAM (${this._apiBase}): ${e.message}`);
    }
  }

  async _fetchSafety() {
    try {
      const res = await fetch(`${this._apiBase}/api/safety`);
      if (!res.ok) return;
      const data = await res.json();
      this._safetyStop = !!data.stop;
      const banner = this.shadowRoot.getElementById('safetyBanner');
      banner.style.display = this._safetyStop ? 'block' : 'none';
      this._updateButtons();
    } catch (e) {
      // Silencieux : pas critique si un seul poll échoue.
    }
  }

  async _sendZone() {
    if (this._polygon.length < 3) return;
    this._setStatus('Envoi de la zone au robot...');
    try {
      const res = await fetch(`${this._apiBase}/api/clean/zone`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ polygon: this._polygon }),
      });
      const data = await res.json();
      if (!res.ok) {
        this._setStatus(`Erreur: ${data.error || res.status}`);
        return;
      }
      this._setStatus(`Nettoyage de zone lancé (${data.points} points de délimitation)`);
    } catch (e) {
      this._setStatus(`Erreur d'envoi: ${e.message}`);
    }
  }

  async _fetchZones() {
    try {
      const res = await fetch(`${this._apiBase}/api/zones`);
      if (!res.ok) return;
      this._zones = await res.json();
      this._renderZonesList();
      this._draw();
    } catch (e) {
      // Silencieux : pas critique si un seul poll échoue.
    }
  }

  async _saveZone() {
    if (this._polygon.length < 3) return;
    const name = this.shadowRoot.getElementById('zoneName').value.trim();
    const color = this.shadowRoot.getElementById('zoneColor').value;
    const icon = this.shadowRoot.getElementById('zoneIcon').value;
    if (!name) {
      this._setStatus('Donne un nom à la zone avant de l\'enregistrer.');
      return;
    }
    try {
      const res = await fetch(`${this._apiBase}/api/zones`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ name, color, icon, polygon: this._polygon }),
      });
      const data = await res.json();
      if (!res.ok) {
        this._setStatus(`Erreur: ${data.error || res.status}`);
        return;
      }
      this._setStatus(`Zone "${name}" enregistrée`);
      this._polygon = [];
      this.shadowRoot.getElementById('zoneName').value = '';
      this._updateButtons();
      await this._fetchZones();
    } catch (e) {
      this._setStatus(`Erreur d'enregistrement: ${e.message}`);
    }
  }

  async _deleteZone(id, name) {
    try {
      const res = await fetch(`${this._apiBase}/api/zones/${id}`, { method: 'DELETE' });
      if (!res.ok) {
        this._setStatus(`Erreur suppression: HTTP ${res.status}`);
        return;
      }
      this._setStatus(`Zone "${name}" supprimée`);
      await this._fetchZones();
    } catch (e) {
      this._setStatus(`Erreur suppression: ${e.message}`);
    }
  }

  async _cleanNamedZone(id, name) {
    this._setStatus(`Nettoyage de "${name}" demandé...`);
    try {
      const res = await fetch(`${this._apiBase}/api/clean/zone/${id}`, { method: 'POST' });
      const data = await res.json();
      this._setStatus(res.ok ? `Nettoyage de "${name}" lancé` : `Erreur: ${data.error || res.status}`);
    } catch (e) {
      this._setStatus(`Erreur d'envoi: ${e.message}`);
    }
  }

  _renderZonesList() {
    const container = this.shadowRoot.getElementById('zonesList');
    if (!container) return;
    container.innerHTML = '';
    this._zones.forEach((zone) => {
      const chip = document.createElement('div');
      chip.className = 'zone-chip';
      chip.style.background = ZONE_COLORS[zone.color] || ZONE_COLORS.gray;
      chip.title = `Nettoyer "${zone.name}"`;

      const label = document.createElement('span');
      label.textContent = `${ZONE_ICONS[zone.icon] || '📦'} ${zone.name}`;
      chip.appendChild(label);

      const del = document.createElement('span');
      del.className = 'del';
      del.textContent = '×';
      del.title = 'Supprimer cette zone';
      del.addEventListener('click', (e) => {
        e.stopPropagation();
        this._deleteZone(zone.id, zone.name);
      });
      chip.appendChild(del);

      chip.addEventListener('click', () => this._cleanNamedZone(zone.id, zone.name));
      container.appendChild(chip);
    });
  }

  async _cleanAll() {
    this._setStatus('Envoi de la demande de nettoyage complet...');
    try {
      const res = await fetch(`${this._apiBase}/api/clean/start`, { method: 'POST' });
      const data = await res.json();
      this._setStatus(res.ok ? 'Nettoyage complet lancé' : `Erreur: ${data.error || res.status}`);
    } catch (e) {
      this._setStatus(`Erreur d'envoi: ${e.message}`);
    }
  }

  async _startExplore() {
    // Exploration automatique par frontières - pas besoin de carte
    // préexistante, marche dès le premier scan reçu. Aucun nettoyage.
    this._setStatus('Exploration automatique en cours (pas de nettoyage)...');
    try {
      const res = await fetch(`${this._apiBase}/api/explore/start`, { method: 'POST' });
      const data = await res.json();
      this._setStatus(res.ok ? 'Exploration lancée' : `Erreur: ${data.error || res.status}`);
    } catch (e) {
      this._setStatus(`Erreur d'envoi: ${e.message}`);
    }
  }

  async _returnToDock() {
    // ⚠️ Navigation approximative vers la position de départ, PAS le
    // retour au socle natif du Neato (pas d'alignement précis sur les
    // contacts de charge garanti) - vérifie visuellement à l'arrivée.
    this._setStatus('Retour au socle en cours (approximatif, à vérifier)...');
    try {
      const res = await fetch(`${this._apiBase}/api/dock/return`, { method: 'POST' });
      const data = await res.json();
      this._setStatus(res.ok ? 'Retour au socle lancé' : `Erreur: ${data.error || res.status}`);
    } catch (e) {
      this._setStatus(`Erreur d'envoi: ${e.message}`);
    }
  }

  async _startScan() {
    // Reparcourt la carte déjà connue SANS activer brosse/aspirateur -
    // pour mettre à jour la carte après avoir déplacé des meubles.
    // Ne fonctionne pas s'il n'y a aucune carte du tout (voir _toggleDrivePad
    // pour la toute première carte).
    this._setStatus('Scan (sans aspirer) en cours...');
    try {
      const res = await fetch(`${this._apiBase}/api/scan/start`, { method: 'POST' });
      const data = await res.json();
      this._setStatus(res.ok ? 'Scan lancé (pas de nettoyage)' : `Erreur: ${data.error || res.status}`);
    } catch (e) {
      this._setStatus(`Erreur d'envoi: ${e.message}`);
    }
  }

  _toggleDrivePad() {
    const pad = this.shadowRoot.getElementById('drivePad');
    const hint = this.shadowRoot.getElementById('driveHint');
    const visible = pad.classList.toggle('visible');
    hint.classList.toggle('visible', visible);
    if (!visible) this._sendTeleop(0, 0); // sécurité : coupe si on referme pendant un appui
  }

  _wireDriveButton(id, linearX, angularZ) {
    const btn = this.shadowRoot.getElementById(id);
    let interval = null;

    const start = (e) => {
      e.preventDefault();
      if (this._safetyStop) return;
      this._sendTeleop(linearX, angularZ);
      interval = setInterval(() => this._sendTeleop(linearX, angularZ), 600);
    };
    const stop = () => {
      if (interval) { clearInterval(interval); interval = null; }
      this._sendTeleop(0, 0);
    };

    btn.addEventListener('pointerdown', start);
    btn.addEventListener('pointerup', stop);
    btn.addEventListener('pointerleave', stop);
    btn.addEventListener('pointercancel', stop);
  }

  async _sendTeleop(linearX, angularZ) {
    try {
      const res = await fetch(`${this._apiBase}/api/teleop`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ linear_x: linearX, angular_z: angularZ }),
      });
      if (!res.ok) {
        const data = await res.json().catch(() => ({}));
        this._setStatus(`Conduite: ${data.error || res.status}`);
      }
    } catch (e) {
      this._setStatus(`Erreur conduite: ${e.message}`);
    }
  }

  async _stopCleaning() {
    // Un seul bouton "Arrêter" pour tout : nettoyage, scan et exploration
    // vivent dans des états séparés côté coverage_planner (topics
    // différents), donc on coupe les trois plutôt que de forcer
    // l'utilisateur à savoir lequel est actif.
    try {
      await Promise.all([
        fetch(`${this._apiBase}/api/clean/stop`, { method: 'POST' }),
        fetch(`${this._apiBase}/api/scan/stop`, { method: 'POST' }),
        fetch(`${this._apiBase}/api/explore/stop`, { method: 'POST' }),
        fetch(`${this._apiBase}/api/dock/stop`, { method: 'POST' }),
      ]);
      this._setStatus('Arrêt demandé');
    } catch (e) {
      this._setStatus(`Erreur d'envoi: ${e.message}`);
    }
  }

  async _runDiagnose() {
    // ~15s bloquant côté conteneur (4 commandes de lecture, PAS de
    // mouvement - SetMotor n'est jamais testé depuis ce bouton).
    const btn = this.shadowRoot.getElementById('btnDiagnose');
    btn.disabled = true;
    this._setStatus('Diagnostic en cours (~15s, aucun mouvement du robot)...');
    try {
      const res = await fetch(`${this._apiBase}/api/diagnose/run`, { method: 'POST' });
      if (!res.ok) {
        this._setStatus(`Erreur diagnostic: HTTP ${res.status}`);
        return;
      }
      const blob = await res.blob();
      const disposition = res.headers.get('Content-Disposition') || '';
      const match = disposition.match(/filename="?([^"]+)"?/);
      const filename = match ? match[1] : 'diagnostic_neato.md';

      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = filename;
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(url);

      this._setStatus(`Rapport téléchargé: ${filename}`);
    } catch (e) {
      this._setStatus(`Erreur diagnostic: ${e.message}`);
    } finally {
      btn.disabled = false;
    }
  }

  // -------------------------------------------------------------- Interaction
  _onCanvasClick(evt) {
    if (!this._mapInfo) return;
    const rect = this._canvas.getBoundingClientRect();
    const scaleX = this._canvas.width / rect.width;
    const scaleY = this._canvas.height / rect.height;
    const px = (evt.clientX - rect.left) * scaleX;
    const py = (evt.clientY - rect.top) * scaleY;

    const world = this._canvasToWorld(px, py);
    this._polygon.push(world);
    this._draw();
    this._updateButtons();
  }

  _updateButtons() {
    const btnZone = this.shadowRoot.getElementById('btnCleanZone');
    btnZone.disabled = this._polygon.length < 3 || this._safetyStop;
    this.shadowRoot.getElementById('btnCleanAll').disabled = this._safetyStop;

    const form = this.shadowRoot.getElementById('saveForm');
    form.classList.toggle('visible', this._polygon.length >= 3);
  }

  _setStatus(text) {
    this._statusText = text;
    const el = this.shadowRoot.getElementById('statusText');
    if (el) el.textContent = text;
  }

  // ------------------------------------------------------------ Conversions
  // Coordonnées monde (mètres, repère "map") <-> pixels canvas.
  // Convention : ligne 0 de la grille en bas de l'image (comme une carte),
  // donc on inverse l'axe Y à l'affichage.
  _worldToCanvas(x, y) {
    const info = this._mapInfo;
    const col = (x - info.origin.x) / info.resolution;
    const row = (y - info.origin.y) / info.resolution;
    const scale = this._canvas.width / info.width;
    return {
      x: col * scale,
      y: this._canvas.height - row * scale,
    };
  }

  _canvasToWorld(px, py) {
    const info = this._mapInfo;
    const scale = this._canvas.width / info.width;
    const col = px / scale;
    const row = (this._canvas.height - py) / scale;
    return [
      info.origin.x + col * info.resolution,
      info.origin.y + row * info.resolution,
    ];
  }

  // ------------------------------------------------------------------ Dessin
  _draw() {
    const ctx = this._ctx;
    const c = this._canvas;
    ctx.clearRect(0, 0, c.width, c.height);

    if (this._mapInfo) {
      this._drawGrid();
    }
    this._drawSavedZones();
    this._drawPolygon();
  }

  _drawSavedZones() {
    if (!this._mapInfo) return;
    const ctx = this._ctx;
    this._zones.forEach((zone) => {
      const color = ZONE_COLORS[zone.color] || ZONE_COLORS.gray;
      ctx.strokeStyle = color;
      ctx.fillStyle = color + '40'; // ~25% opacité
      ctx.lineWidth = 2;

      ctx.beginPath();
      zone.polygon.forEach(([x, y], i) => {
        const p = this._worldToCanvas(x, y);
        if (i === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      });
      ctx.closePath();
      ctx.stroke();
      ctx.fill();

      // Étiquette (icône + nom) au centroïde approximatif de la zone
      const cx = zone.polygon.reduce((s, p) => s + p[0], 0) / zone.polygon.length;
      const cy = zone.polygon.reduce((s, p) => s + p[1], 0) / zone.polygon.length;
      const center = this._worldToCanvas(cx, cy);
      ctx.font = '14px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillStyle = '#ffffff';
      ctx.fillText(`${ZONE_ICONS[zone.icon] || ''} ${zone.name}`, center.x, center.y);
    });
  }

  _drawGrid() {
    const info = this._mapInfo;
    const ctx = this._ctx;
    const scale = this._canvas.width / info.width;
    const data = info.data;

    // Rendu simple cellule par cellule. Pour de très grandes cartes,
    // ça peut devenir lent - suffisant pour un appartement classique.
    for (let row = 0; row < info.height; row++) {
      for (let col = 0; col < info.width; col++) {
        const v = data[row * info.width + col];
        let color;
        if (v < 0) {
          color = '#3a3a3a'; // inconnu
        } else if (v < 50) {
          color = '#f5f5f5'; // libre
        } else {
          color = '#111111'; // occupé
        }
        ctx.fillStyle = color;
        const px = col * scale;
        const py = this._canvas.height - (row + 1) * scale;
        ctx.fillRect(px, py, Math.ceil(scale), Math.ceil(scale));
      }
    }
  }

  _drawPolygon() {
    if (this._polygon.length === 0) return;
    const ctx = this._ctx;
    ctx.strokeStyle = '#03a9f4';
    ctx.fillStyle = 'rgba(3, 169, 244, 0.25)';
    ctx.lineWidth = 2;

    ctx.beginPath();
    this._polygon.forEach(([x, y], i) => {
      const p = this._worldToCanvas(x, y);
      if (i === 0) ctx.moveTo(p.x, p.y);
      else ctx.lineTo(p.x, p.y);
    });
    if (this._polygon.length >= 3) ctx.closePath();
    ctx.stroke();
    if (this._polygon.length >= 3) ctx.fill();

    // Points cliquables, plus visibles
    this._polygon.forEach(([x, y]) => {
      const p = this._worldToCanvas(x, y);
      ctx.beginPath();
      ctx.arc(p.x, p.y, 4, 0, 2 * Math.PI);
      ctx.fillStyle = '#03a9f4';
      ctx.fill();
    });
  }
}

customElements.define('neato-zone-map-card', NeatoZoneMapCard);

// Pour que la carte apparaisse dans le sélecteur de cartes du dashboard HA
window.customCards = window.customCards || [];
window.customCards.push({
  type: 'neato-zone-map-card',
  name: 'Neato Zone Map',
  description: "Carte SLAM interactive pour sélectionner une zone à nettoyer",
});
