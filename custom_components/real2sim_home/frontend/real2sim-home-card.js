/*
 * Real2Sim Home floor-plan card + sidebar panel.
 *   type: custom:real2sim-home-card
 *   title: Home            (optional)
 *   rooms: [living_room]   (optional: only these rooms)
 *   show_objects: true, show_sensors: true, show_list: true, height: 520 (px, optional)
 * Data comes from the integration over the websocket (real2sim_home/subscribe): room maps (room_map.json,
 * HOME.md), their place on the home plan, sensor placements, and the fused state (people with position, pose,
 * heart and breathing rate; room occupancy; sensors).
 */
const OBJ = {
  tv: ["📺", "#5c6bc0"], toilet: ["🚽", "#26a69a"], chair: ["🪑", "#8d6e63"], sofa: ["🛋️", "#7e57c2"],
  bed: ["🛏️", "#42a5f5"], table: ["🍽️", "#a1887f"], refrigerator: ["🧊", "#4fc3f7"], oven: ["♨️", "#ef6c00"],
  stove: ["🔥", "#ef6c00"], microwave: ["♨️", "#ff8a65"], sink: ["🚰", "#29b6f6"], bathtub: ["🛁", "#4dd0e1"],
  plant: ["🪴", "#66bb6a"], laptop: ["💻", "#78909c"], storage: ["🗄️", "#90a4ae"], washer_dryer: ["🧺", "#80deea"],
  dishwasher: ["🍽️", "#80cbc4"], fireplace: ["🔥", "#d84315"], stairs: ["🪜", "#bcaaa4"], clock: ["🕒", "#9e9e9e"],
  bench: ["🪑", "#a1887f"], toaster: ["🍞", "#ffb74d"], bicycle: ["🚲", "#9e9e9e"],
};
const MARK = {
  motion: "🏃", presence: "👤", occupancy: "👤", camera: "📷", temperature: "🌡️", humidity: "💧", illuminance: "☀️",
  co2: "🫧", sound: "🎤", microphone: "🎙️", light: "💡", ruview: "📡", radar: "📶",
};
const POSE = { standing: "🧍", walking: "🚶", sitting: "🪑", lying: "🛌", fallen: "🆘" };
const esc = (s) => String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
const fmt = (v, d = 0) => (v === null || v === undefined || v === "" || isNaN(Number(v)) ? null : Number(v).toFixed(d));

class Real2SimHomeCard extends HTMLElement {
  static getStubConfig() { return { title: "Home" }; }
  static getConfigElement() { return null; }

  setConfig(config) {
    this._config = { show_objects: true, show_sensors: true, show_list: true, ...config };
    this._geoKey = null;
    this._stateKey = null;
    if (this.shadowRoot) this._render(true);
  }

  getCardSize() { return 8; }
  getGridOptions() { return { columns: 12, min_columns: 6, rows: 8 }; }

  set hass(hass) {
    this._hass = hass;
    if (!this.shadowRoot) this._init();
    if (!this._unsub && !this._subscribing) this._subscribe();
  }

  connectedCallback() { if (this._hass && !this._unsub && !this._subscribing) this._subscribe(); }

  disconnectedCallback() {
    if (this._unsub) { this._unsub.then((u) => u && u()).catch(() => {}); this._unsub = null; }
  }

  _init() {
    const root = this.attachShadow({ mode: "open" });
    root.innerHTML = `
      <style>
        :host { display: block; }
        ha-card, .card { padding: 12px; box-sizing: border-box; height: 100%; display: flex; flex-direction: column; }
        .head { display: flex; align-items: center; gap: 8px; margin-bottom: 6px; }
        .title { font-size: 1.2em; font-weight: 600; flex: 1; color: var(--primary-text-color); }
        button { background: var(--secondary-background-color, #eee); color: var(--primary-text-color); border: 0;
                 border-radius: 8px; padding: 4px 10px; cursor: pointer; font: inherit; font-size: 0.85em; }
        button.on { background: var(--primary-color); color: var(--text-primary-color, #fff); }
        .plan { position: relative; flex: 1; min-height: 120px; }
        svg { width: 100%; height: auto; max-height: 78vh; display: block; user-select: none; touch-action: none; }
        .msg { color: var(--secondary-text-color); padding: 24px 8px; line-height: 1.5; }
        .list { display: grid; grid-template-columns: repeat(auto-fill, minmax(210px, 1fr)); gap: 6px; margin-top: 8px; }
        .item { background: var(--secondary-background-color, #f3f3f3); border-radius: 10px; padding: 6px 9px;
                font-size: 0.85em; color: var(--primary-text-color); }
        .item b { font-weight: 600; }
        .muted { color: var(--secondary-text-color); }
        .alert { background: #c62828; color: #fff; }
        .heart { transform-box: fill-box; transform-origin: center; animation: beat var(--beat, 1s) infinite; }
        @keyframes beat { 0%,100% { transform: scale(1); } 15% { transform: scale(1.35); } 30% { transform: scale(1); } 45% { transform: scale(1.18); } }
        .pulse { animation: pulse 1.6s infinite; transform-box: fill-box; transform-origin: center; }
        @keyframes pulse { 0% { opacity: .7; transform: scale(.6); } 100% { opacity: 0; transform: scale(1.8); } }
        .clickable { cursor: pointer; }
        .drag { cursor: grab; }
      </style>
      <ha-card class="card">
        <div class="head"><div class="title"></div><button class="edit" title="Drag rooms and devices">Edit layout</button></div>
        <div class="plan"><div class="msg">Connecting…</div></div>
        <div class="list"></div>
      </ha-card>`;
    this._edit = false;
    root.querySelector(".edit").addEventListener("click", () => {
      this._edit = !this._edit;
      root.querySelector(".edit").classList.toggle("on", this._edit);
      this._render(true);
    });
  }

  _subscribe() {
    if (!this._hass || !this._hass.connection) return;
    this._subscribing = true;
    this._unsub = this._hass.connection.subscribeMessage((ev) => {
      if (ev.kind === "geometry") this._geo = ev.data;
      else if (ev.kind === "state") this._state = ev.data;
      this._render(false);
    }, { type: "real2sim_home/subscribe" });
    this._unsub.catch((err) => {
      this._subscribing = false; this._unsub = null;
      this._message(`Real2Sim Home is not available (${esc(err && (err.message || err.code))}). Add the integration under Settings › Devices & services.`);
    }).then(() => { this._subscribing = false; });
  }

  _message(html) {
    const plan = this.shadowRoot.querySelector(".plan");
    plan.innerHTML = `<div class="msg">${html}</div>`;
  }

  // ---------------------------------------------------------------- geometry helpers
  _rooms() {
    const all = (this._geo && this._geo.rooms) || [];
    const only = this._config.rooms;
    return only && only.length ? all.filter((r) => only.includes(r.id)) : all;
  }

  /** room-local (x, y) -> home plan (x, y), metres, y up */
  _toHome(room, x, y) {
    const m = room.map, b = m.bounds || [0, 0, 4, 4];
    const w = b[2] - b[0], h = b[3] - b[1];
    const lx = x - b[0], ly = y - b[1];
    const rot = ((Math.round((room.layout.rotation || 0) / 90) % 4) + 4) % 4;
    let px = lx, py = ly;
    if (rot === 1) { px = h - ly; py = lx; }
    else if (rot === 2) { px = w - lx; py = h - ly; }
    else if (rot === 3) { px = ly; py = w - lx; }
    return [room.layout.x + px, room.layout.y + py];
  }

  _fromHome(room, hx, hy) {
    const m = room.map, b = m.bounds || [0, 0, 4, 4];
    const w = b[2] - b[0], h = b[3] - b[1];
    const px = hx - room.layout.x, py = hy - room.layout.y;
    const rot = ((Math.round((room.layout.rotation || 0) / 90) % 4) + 4) % 4;
    let lx = px, ly = py;
    if (rot === 1) { lx = py; ly = h - px; }
    else if (rot === 2) { lx = w - px; ly = h - py; }
    else if (rot === 3) { lx = w - py; ly = px; }
    return [lx + b[0], ly + b[1]];
  }

  _extent(rooms) {
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    for (const r of rooms) {
      const b = r.map.bounds || [0, 0, 4, 4];
      for (const [x, y] of [[b[0], b[1]], [b[2], b[1]], [b[2], b[3]], [b[0], b[3]]]) {
        const [hx, hy] = this._toHome(r, x, y);
        x0 = Math.min(x0, hx); y0 = Math.min(y0, hy); x1 = Math.max(x1, hx); y1 = Math.max(y1, hy);
      }
    }
    if (!isFinite(x0)) return [0, 0, 4, 4];
    return [x0 - 0.6, y0 - 0.6, x1 + 0.6, y1 + 0.6];
  }

  // ---------------------------------------------------------------- render
  _render(force) {
    if (!this.shadowRoot) return;
    this.shadowRoot.querySelector(".title").textContent = this._config.title || "Home map";
    if (!this._geo) return;
    const rooms = this._rooms();
    if (!rooms.length) {
      this._message("No rooms yet. Scan a room with the R2S Capture app (Android: mode A with “Map the room”; iPhone: mode B RoomPlan) and send it to Home Assistant, or call the <code>real2sim_home.upload_room_map</code> service.");
      this.shadowRoot.querySelector(".list").innerHTML = this._listHtml([]);
      return;
    }
    const geoKey = JSON.stringify(this._geo) + this._edit;
    const stateKey = JSON.stringify(this._state || {});
    if (!force && geoKey === this._geoKey && stateKey === this._stateKey) return;
    const geoChanged = force || geoKey !== this._geoKey;
    this._geoKey = geoKey; this._stateKey = stateKey;
    const [x0, y0, x1, y1] = this._extent(rooms);
    this._ext = [x0, y0, x1, y1];
    const H = y1;
    this._Y = (hy) => H - hy + y0;   // flip: svg y grows down
    const plan = this.shadowRoot.querySelector(".plan");
    if (geoChanged || !plan.querySelector("svg")) {
      const height = this._config.height ? `style="max-height:${Number(this._config.height)}px"` : "";
      plan.innerHTML = `<svg viewBox="${x0} ${y0} ${x1 - x0} ${y1 - y0}" preserveAspectRatio="xMidYMid meet" ${height}>
        <g class="static">${rooms.map((r) => this._roomSvg(r)).join("")}</g><g class="dyn"></g></svg>`;
      this._bindStatic(plan.querySelector("svg"));
    }
    plan.querySelector("g.dyn").innerHTML = this._dynSvg(rooms);
    this._bindDyn(plan.querySelector("svg"));
    if (this._config.show_list !== false) this.shadowRoot.querySelector(".list").innerHTML = this._listHtml(rooms);
  }

  _pt(room, x, y) { const [hx, hy] = this._toHome(room, x, y); return [hx, this._Y(hy)]; }

  _roomSvg(r) {
    const m = r.map;
    const st = ((this._state && this._state.rooms) || []).find((s) => s.id === r.id) || {};
    const floor = (m.floor || []).map(([x, y]) => this._pt(r, x, y).join(",")).join(" ");
    const walls = (m.walls || []).map((w) => {
      const [ax, ay] = this._pt(r, w.a[0], w.a[1]); const [bx, by] = this._pt(r, w.b[0], w.b[1]);
      return `<line x1="${ax}" y1="${ay}" x2="${bx}" y2="${by}" stroke="var(--primary-text-color,#333)" stroke-width="0.09" stroke-linecap="round"/>`;
    }).join("");
    const seg = (list, color, width, dash) => (list || []).map((w) => {
      const [ax, ay] = this._pt(r, w.a[0], w.a[1]); const [bx, by] = this._pt(r, w.b[0], w.b[1]);
      return `<line x1="${ax}" y1="${ay}" x2="${bx}" y2="${by}" stroke="${color}" stroke-width="${width}" ${dash ? `stroke-dasharray="${dash}"` : ""}/>`;
    }).join("");
    let objs = "";
    if (this._config.show_objects !== false) {
      objs = (m.objects || []).map((o) => {
        const [ic, col] = OBJ[o.category] || ["▫️", "#9e9e9e"];
        const [cx, cy] = this._pt(r, o.center[0], o.center[1]);
        const rot = -((o.yaw || 0) * 180) / Math.PI - (r.layout.rotation || 0);
        const w = Math.max(0.15, o.size[0]), d = Math.max(0.1, o.size[1]);
        const fs = Math.max(0.16, Math.min(0.42, Math.min(w, d) * 0.7));
        return `<g transform="translate(${cx} ${cy})"><g transform="rotate(${-rot === 0 ? 0 : rot})">
            <rect x="${-w / 2}" y="${-d / 2}" width="${w}" height="${d}" rx="0.05" fill="${col}" fill-opacity="0.22" stroke="${col}" stroke-width="0.025"/></g>
            <text y="${fs * 0.35}" font-size="${fs}" text-anchor="middle">${ic}</text>
            <title>${esc(o.label || o.category)} · ${o.size.map((v) => v.toFixed(2)).join(" × ")} m</title></g>`;
      }).join("");
    }
    const b = m.bounds || [0, 0, 4, 4];
    const [lx, ly] = this._pt(r, b[0], b[3]);
    return `<g class="room" data-room="${esc(r.id)}">
      <polygon class="floor ${this._edit ? "drag" : ""}" points="${floor}" fill="${st.occupied ? "rgba(255,152,0,0.13)" : "var(--card-background-color,#fff)"}"
        stroke="var(--divider-color,#ccc)" stroke-width="0.03"/>
      ${walls}${seg(m.windows, "#4fc3f7", 0.07)}${seg(m.doors, "var(--card-background-color,#fff)", 0.1)}${seg(m.doors, "#8d6e63", 0.03, "0.08 0.05")}
      ${seg(m.openings, "var(--card-background-color,#fff)", 0.1)}
      ${objs}
      <text x="${lx}" y="${ly - 0.12}" font-size="0.26" font-weight="600" fill="var(--primary-text-color,#222)">${esc(r.name)}</text>
    </g>`;
  }

  _dynSvg(rooms) {
    const s = this._state || {};
    const byId = Object.fromEntries(rooms.map((r) => [r.id, r]));
    let out = "";
    // room chips: temperature, humidity, people
    for (const st of s.rooms || []) {
      const r = byId[st.id]; if (!r) continue;
      const b = r.map.bounds || [0, 0, 4, 4];
      const [lx, ly] = this._pt(r, b[0], b[3]);
      const chips = [];
      if (st.temperature !== null && st.temperature !== undefined) chips.push(`🌡️${st.temperature}°`);
      if (st.humidity !== null && st.humidity !== undefined) chips.push(`💧${Math.round(st.humidity)}%`);
      if (st.count) chips.push(`👥${st.count}`);
      const nameW = 0.16 * (r.name || "").length + 0.25;
      if (chips.length) out += `<text x="${lx + nameW}" y="${ly - 0.12}" font-size="0.2" fill="var(--secondary-text-color,#555)">${chips.join("  ")}</text>`;
      if (st.fall) out += `<text x="${lx + 0.12}" y="${ly + 0.32}" font-size="0.24" font-weight="700" fill="#c62828">⚠ FALL DETECTED</text>`;
    }
    if (this._config.show_sensors !== false) {
      for (const mk of s.markers || []) {
        const r = byId[mk.room_id]; if (!r) continue;
        const [x, y] = this._pt(r, mk.x, mk.y);
        const icon = MARK[mk.kind] || "•";
        const val = mk.kind === "camera" || mk.kind === "microphone" || mk.kind === "light" || mk.kind === "motion" || mk.kind === "presence" || mk.kind === "ruview"
          ? "" : (fmt(mk.state, mk.kind === "temperature" ? 1 : 0) ?? "") + (mk.unit && fmt(mk.state) !== null ? (mk.unit === "°C" || mk.unit === "°F" ? "°" : mk.unit === "%" ? "%" : "") : "");
        const active = mk.on;
        const ring = active ? `<circle r="0.2" fill="${mk.kind === "light" ? "#ffd54f" : "#ff9800"}" class="pulse"/>` : "";
        const ent = mk.entity_id.includes(".") ? mk.entity_id : "";
        out += `<g class="marker ${ent ? "clickable" : ""} ${this._edit ? "drag" : ""}" data-entity="${esc(ent)}" data-item="${esc(mk.entity_id)}" data-room="${esc(mk.room_id)}" transform="translate(${x} ${y})">
          ${ring}<circle r="0.16" fill="var(--card-background-color,#fff)" stroke="${active ? "#ff9800" : "var(--divider-color,#bbb)"}" stroke-width="0.025"/>
          <text y="0.065" font-size="0.18" text-anchor="middle">${icon}</text>
          ${val ? `<text y="0.36" font-size="0.15" text-anchor="middle" fill="var(--primary-text-color,#333)">${esc(val)}</text>` : ""}
          <title>${esc(mk.name)}: ${esc(mk.state)}${mk.unit ? " " + esc(mk.unit) : ""}</title></g>`;
      }
    }
    for (const p of s.people || []) {
      const r = byId[p.room_id]; if (!r) continue;
      const [x, y] = this._pt(r, p.x, p.y);
      const fallen = p.pose === "fallen";
      const col = fallen ? "#c62828" : p.known ? "var(--primary-color,#03a9f4)" : "#78909c";
      const br = Number(p.breathing_rate) || 0, hr = Number(p.heart_rate) || 0;
      const breath = br ? `<circle r="0.32" fill="none" stroke="${col}" stroke-width="0.03" opacity="0.6">
          <animate attributeName="r" values="0.3;0.46;0.3" dur="${(60 / br).toFixed(2)}s" repeatCount="indefinite"/>
          <animate attributeName="opacity" values="0.7;0.15;0.7" dur="${(60 / br).toFixed(2)}s" repeatCount="indefinite"/></circle>` : "";
      const initials = esc((p.name || "?").split(/\s+/).map((w) => w[0]).join("").slice(0, 2).toUpperCase());
      const vit = [];
      if (hr) vit.push(`<g transform="translate(0.34 -0.3)"><text class="heart" style="--beat:${(60 / hr).toFixed(2)}s" font-size="0.22" text-anchor="middle" y="0.07">❤️</text>
          <text x="0.15" y="0.07" font-size="0.19" font-weight="700" fill="#e53935">${hr}</text></g>`);
      if (br) vit.push(`<text x="0.2" y="0.5" font-size="0.16" fill="${col}">🫁 ${fmt(br, 0)}</text>`);
      out += `<g class="person ${p.known && p.id.includes(".") ? "clickable" : ""}" data-entity="${p.known ? esc(p.id) : ""}" transform="translate(${x} ${y})">
        ${breath}
        <circle r="0.24" fill="${col}" fill-opacity="${fallen ? 0.9 : 0.85}" stroke="#fff" stroke-width="0.03"/>
        <text y="0.08" font-size="0.2" text-anchor="middle" fill="#fff" font-weight="700">${p.known ? initials : "?"}</text>
        <text x="-0.32" y="-0.18" font-size="0.22" text-anchor="middle">${POSE[p.pose] || ""}</text>
        ${vit.join("")}
        <text y="-0.36" font-size="0.17" text-anchor="middle" fill="var(--primary-text-color,#222)" font-weight="600">${esc(p.name)}</text>
        <title>${esc(p.name)} · ${esc(p.pose)}${p.on ? " on " + esc(p.on) : ""}${hr ? ` · ❤ ${hr} bpm (${esc(p.heart_rate_source)})` : ""}${br ? ` · breathing ${br}/min (${esc(p.breathing_rate_source)})` : ""} · position: ${esc(p.position_source)}</title></g>`;
    }
    return out;
  }

  _listHtml(rooms) {
    const s = this._state || {};
    const roomName = (id) => (rooms.find((r) => r.id === id) || {}).name || id;
    const items = [];
    for (const p of s.people || []) {
      const ph = p.phone ? ` · 📱${p.phone.battery !== null && p.phone.battery !== undefined ? Math.round(p.phone.battery) + "%" : ""}${p.phone.charging ? "⚡" : ""}` : "";
      items.push(`<div class="item ${p.pose === "fallen" ? "alert" : ""}"><b>${esc(p.name)}</b> · ${esc(roomName(p.room_id))}<br>
        ${POSE[p.pose] || ""} ${esc(p.pose)}${p.on ? ` (${esc(p.on)})` : ""}${p.heart_rate ? ` · ❤️ ${p.heart_rate}` : ""}${p.breathing_rate ? ` · 🫁 ${p.breathing_rate}` : ""}${ph}
        <div class="muted">position: ${esc(p.position_source)}${p.heart_rate_source ? ` · heart: ${esc(p.heart_rate_source)}` : ""}</div></div>`);
    }
    for (const p of s.elsewhere || []) {
      const ph = p.phone && p.phone.battery !== null && p.phone.battery !== undefined ? ` · 📱${Math.round(p.phone.battery)}%` : "";
      items.push(`<div class="item"><b>${esc(p.name)}</b> · <span class="muted">${p.home ? "home, room unknown" : "away"}</span>${p.heart_rate ? ` · ❤️ ${p.heart_rate}` : ""}${ph}</div>`);
    }
    for (const n of s.nodes || []) {
      items.push(`<div class="item">📡 <b>RuView node ${n.node_id}</b> · ${esc(n.room_id ? roomName(n.room_id) : "no room")}<br>
        ${n.presence ? "presence" : "clear"}${n.fall ? " · <b>fall</b>" : ""}${n.heart_rate ? ` · ❤️ ${fmt(n.heart_rate)}` : ""}${n.breathing_rate ? ` · 🫁 ${fmt(n.breathing_rate, 1)}` : ""}
        <span class="muted">${n.vitals_age !== null ? ` · ${n.vitals_age}s ago` : " · no vitals"}</span></div>`);
    }
    if (this._edit) items.push(`<div class="item muted">Edit layout: drag a room by its floor to move it (double-click / double-tap to rotate 90°), drag a device marker to where it is in the room.</div>`);
    return items.join("");
  }

  // ---------------------------------------------------------------- interaction
  _svgPoint(svg, ev) {
    const pt = svg.createSVGPoint(); pt.x = ev.clientX; pt.y = ev.clientY;
    const p = pt.matrixTransform(svg.getScreenCTM().inverse());
    return [p.x, this._ext[1] + this._ext[3] - p.y];  // svg y -> home y (inverse of _Y)
  }

  _bindStatic(svg) {
    svg.querySelectorAll("g.room").forEach((g) => {
      const floor = g.querySelector("polygon.floor");
      const room = this._rooms().find((r) => r.id === g.dataset.room);
      floor.addEventListener("pointerdown", (ev) => {
        if (!this._edit) return;
        ev.preventDefault();
        const start = this._svgPoint(svg, ev); const lay0 = { ...room.layout };
        floor.setPointerCapture(ev.pointerId);
        const move = (e) => {
          const p = this._svgPoint(svg, e);
          room.layout = { ...lay0, x: lay0.x + p[0] - start[0], y: lay0.y + p[1] - start[1] };
          g.setAttribute("transform", `translate(${room.layout.x - lay0.x} ${-(room.layout.y - lay0.y)})`);
        };
        const up = () => {
          floor.removeEventListener("pointermove", move); floor.removeEventListener("pointerup", up);
          const snap = (v) => Math.round(v * 20) / 20;
          this._send({ item: "room", room_id: room.id, x: snap(room.layout.x), y: snap(room.layout.y), rotation: room.layout.rotation || 0 });
        };
        floor.addEventListener("pointermove", move); floor.addEventListener("pointerup", up);
      });
      floor.addEventListener("dblclick", () => {
        if (!this._edit) return;
        this._send({ item: "room", room_id: room.id, x: room.layout.x, y: room.layout.y, rotation: ((room.layout.rotation || 0) + 90) % 360 });
      });
    });
  }

  _bindDyn(svg) {
    svg.querySelectorAll("g.marker, g.person").forEach((g) => {
      g.addEventListener("click", () => {
        if (this._edit || this._dragged) return;
        const eid = g.dataset.entity;
        if (eid) this.dispatchEvent(new CustomEvent("hass-more-info", { detail: { entityId: eid }, bubbles: true, composed: true }));
      });
    });
    svg.querySelectorAll("g.marker").forEach((g) => {
      g.addEventListener("pointerdown", (ev) => {
        if (!this._edit) return;
        ev.preventDefault(); ev.stopPropagation();
        g.setPointerCapture(ev.pointerId);
        let last = null;
        const move = (e) => {
          const p = this._svgPoint(svg, e); last = p;
          g.setAttribute("transform", `translate(${p[0]} ${this._Y(p[1])})`);
        };
        const up = () => {
          g.removeEventListener("pointermove", move); g.removeEventListener("pointerup", up);
          if (!last) return;
          // the room under the drop point
          const room = this._rooms().find((r) => {
            const [lx, ly] = this._fromHome(r, last[0], last[1]); const b = r.map.bounds || [0, 0, 4, 4];
            return lx >= b[0] && lx <= b[2] && ly >= b[1] && ly <= b[3];
          }) || this._rooms().find((r) => r.id === g.dataset.room);
          const [lx, ly] = this._fromHome(room, last[0], last[1]);
          const item = g.dataset.item.startsWith("sensor.") && /_target_\d+_x$/.test(g.dataset.item) ? g.dataset.item : g.dataset.item;
          this._send({ item, room_id: room.id, x: Math.round(lx * 20) / 20, y: Math.round(ly * 20) / 20, rotation: 0 });
        };
        g.addEventListener("pointermove", move); g.addEventListener("pointerup", up);
      });
    });
  }

  _send(msg) {
    if (!this._hass) return;
    this._hass.callWS({ type: "real2sim_home/place", ...msg }).catch((err) => {
      this._message(`Could not save the layout: ${esc(err && (err.message || err.code))}`);
    });
  }
}

/** Sidebar panel ("Home map"): the card, full size. */
class Real2SimHomePanel extends HTMLElement {
  set hass(h) {
    if (!this._card) {
      this.style.display = "block"; this.style.height = "100%"; this.style.padding = "8px"; this.style.boxSizing = "border-box";
      this.style.background = "var(--primary-background-color)";
      this._card = document.createElement("real2sim-home-card");
      this._card.setConfig({ title: "Home map", height: Math.max(320, window.innerHeight - 220) });
      this.appendChild(this._card);
    }
    this._card.hass = h;
  }
  set panel(_p) {}
  set narrow(_n) {}
  set route(_r) {}
}

if (!customElements.get("real2sim-home-card")) customElements.define("real2sim-home-card", Real2SimHomeCard);
if (!customElements.get("real2sim-home-panel")) customElements.define("real2sim-home-panel", Real2SimHomePanel);
window.customCards = window.customCards || [];
if (!window.customCards.find((c) => c.type === "real2sim-home-card")) {
  window.customCards.push({ type: "real2sim-home-card", name: "Real2Sim Home map",
    description: "Rooms scanned with R2S Capture, people with pose and heart / breathing rate, sensors in place." });
}
