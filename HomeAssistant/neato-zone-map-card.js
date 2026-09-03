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

class NeatoZoneMapCard extends HTMLElement {
  setConfig(config) {
    if (!config.api_base_url) {
      throw new Error('neato-zone-map-card: "api_base_url" est requis (ex: http://192.168.10.50:2000)');
    }
    this._config = config;
    this._apiBase = config.api_base_url.replace(/\/$/, '');
    this._refreshMs = config.refresh_ms || 4000;
    this._polygon = []; // points en coordonnées MONDE (mètres), pas pixels
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
        </div>
        <canvas id="mapCanvas" width="600" height="600"></canvas>
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

    this._fetchMap();
    this._fetchSafety();
  }

  _startPolling() {
    this._pollTimer = setInterval(() => {
      this._fetchMap();
      this._fetchSafety();
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

  async _stopCleaning() {
    try {
      await fetch(`${this._apiBase}/api/clean/stop`, { method: 'POST' });
      this._setStatus('Arrêt demandé');
    } catch (e) {
      this._setStatus(`Erreur d'envoi: ${e.message}`);
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
    this._drawPolygon();
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
