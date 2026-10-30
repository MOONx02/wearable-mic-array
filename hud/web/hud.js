// Browser version of hud_render.py + the `view` command in hud.py.
(() => {
  const D = window.HUD_DATA;
  const W = 1920, H = 1080;
  const CAM = { x: 24, y: 86, w: 1120, h: 840 };
  const TIMELINE = { x: 24, y: 944, w: 1120, h: 112 };
  const RADAR_BOX = { x: 1170, y: 86, w: 726, h: 600 };
  const METER_BOX = { x: 1170, y: 702, w: 726, h: 150 };
  const EVENT_BOX = { x: 1170, y: 868, w: 726, h: 188 };
  const RADAR_CY = 86 + 292, RADAR_R = 222;
  const BEAM_HALF_WIDTH = 6.0;
  const INK = "#e6edf3", DIM = "#8b949e", PANEL = "#161b22", EDGE = "#30363d", BEAM = "#58a6ff";
  const ALERT = "#ff7b72", SIM_BADGE = "#d29922", BG = "#0d1117";
  const CMAP = [[0, 0, 4], [40, 11, 84], [101, 21, 110], [159, 42, 99], [212, 72, 66],
    [245, 125, 21], [250, 193, 39], [252, 255, 164]];

  const canvas = document.getElementById("hud");
  const ctx = canvas.getContext("2d");
  const video = document.getElementById("cam");
  const $ = (id) => document.getElementById(id);
  const contains = (r, x, y) => x >= r.x && x <= r.x + r.w && y >= r.y && y <= r.y + r.h;
  const center = (r) => [r.x + r.w / 2, r.y + r.h / 2];
  const rad = (d) => (d * Math.PI) / 180;

  function heatRGB(v) {
    v = Math.min(Math.max(v, 0), 1) * (CMAP.length - 1);
    const i = Math.min(Math.floor(v), CMAP.length - 2), f = v - i;
    return CMAP[i].map((c, k) => Math.round(c + (CMAP[i + 1][k] - c) * f));
  }
  const heat = (v, a = 1) => { const [r, g, b] = heatRGB(v); return `rgba(${r},${g},${b},${a})`; };
  const withAlpha = (hex, a) => {
    const n = parseInt(hex.slice(1), 16);
    return `rgba(${(n >> 16) & 255},${(n >> 8) & 255},${n & 255},${a})`;
  };
  const wrap = (a) => ((((a + 180) % 360) + 360) % 360) - 180;

  function sampleMap(m, az) {
    const n = m.length, pos = (wrap(az) + 180) / 2.5;
    const i = ((Math.floor(pos) % n) + n) % n, frac = pos - Math.floor(pos);
    return m[i] * (1 - frac) + m[(i + 1) % n] * frac;
  }

  function directionName(az) {
    const a = wrap(az);
    if (Math.abs(a) <= 22.5) return "front";
    if (Math.abs(a) >= 157.5) return "behind";
    const side = a > 0 ? "left" : "right";
    if (Math.abs(a) < 67.5) return `front-${side}`;
    if (Math.abs(a) <= 112.5) return side;
    return `behind-${side}`;
  }

  const cam = {
    hfov: D.hfov, yaw: D.yaw,
    get f() { return CAM.w / 2 / Math.tan(rad(this.hfov / 2)); },
    xOf(az) {
      const rel = wrap(az - this.yaw);
      if (Math.abs(rel) >= 89) return null;
      return center(CAM)[0] - this.f * Math.tan(rad(rel));
    },
    azOf(x) { return wrap(this.yaw + (Math.atan(-(x - center(CAM)[0]) / this.f) * 180) / Math.PI); },
    visible(az) { return Math.abs(wrap(az - this.yaw)) <= this.hfov / 2; },
  };

  // ---------------------------------------------------------------- drawing helpers

  function setFont(size, bold, mono) {
    ctx.font = `${bold ? "bold " : ""}${size}px ${mono ? "Menlo, monospace" : "Helvetica, Arial, sans-serif"}`;
  }

  function text(x, y, s, size = 18, color = INK, bold = false, align = "left", mono = false) {
    setFont(size, bold, mono);
    ctx.fillStyle = color;
    const w = ctx.measureText(s).width;
    ctx.fillText(s, x + (align === "center" ? -w / 2 : align === "right" ? -w : 0), y);
    return w;
  }

  function rrect(x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
  }

  function panel(r, title) {
    rrect(r.x, r.y, r.w, r.h, 10);
    ctx.fillStyle = PANEL;
    ctx.fill();
    ctx.strokeStyle = EDGE;
    ctx.lineWidth = 1.5;
    ctx.stroke();
    if (title) text(r.x + 16, r.y + 28, title, 17, DIM, true);
  }

  function line(x0, y0, x1, y1, color, width, dash) {
    ctx.beginPath();
    ctx.moveTo(x0, y0);
    ctx.lineTo(x1, y1);
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.setLineDash(dash || []);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  function radarPoint(cx, cy, r, az) {
    return [cx - r * Math.sin(rad(az)), cy - r * Math.cos(rad(az))];
  }

  // ---------------------------------------------------------------- data and state

  const nB = D.n_blocks, BLOCK = D.block_s;
  const disp = [];
  for (let b = 0; b < nB; b++) {
    disp.push(b === 0 ? D.srp[0].slice() : D.srp[b].map((v, i) => Math.max(v, 0.7 * disp[b - 1][i])));
  }

  const state = {
    b: 0, track: Float64Array.from(D.auto_track), mode: "auto-follow", kind: "mvdr", listen: "beam",
    beamDb: new Float64Array(nB).fill(-120), locks: [], ready: false,
  };

  function beamLocks(track, minJump = 10) {
    const out = [];
    for (let b = 1; b < track.length; b++) {
      if (Math.abs(wrap(track[b] - track[b - 1])) >= minJump) out.push([b * BLOCK, track[b]]);
    }
    return out;
  }

  // ---------------------------------------------------------------- camera view

  function drawSyntheticScene(t) {
    const r = CAM, [cx, cy] = center(r);
    const g = ctx.createLinearGradient(0, r.y, 0, r.y + r.h);
    g.addColorStop(0, "#20262e");
    g.addColorStop(0.5, "#2b323b");
    g.addColorStop(0.5001, "#3a4048");
    g.addColorStop(1, "#22272e");
    ctx.fillStyle = g;
    ctx.fillRect(r.x, r.y, r.w, r.h);
    for (const side of [-1, 1]) {
      for (let k = 0; k < 6; k++) {
        const near = 1 / (1 + k * 0.55), far = 1 / (1 + (k + 1) * 0.55);
        const x0 = cx + side * r.w * 0.62 * near, x1 = cx + side * r.w * 0.62 * far;
        const top0 = cy - 330 * near, top1 = cy - 330 * far, bot0 = cy + 300 * near, bot1 = cy + 300 * far;
        ctx.beginPath();
        ctx.moveTo(x0, top0); ctx.lineTo(x1, top1); ctx.lineTo(x1, bot1); ctx.lineTo(x0, bot0);
        ctx.closePath();
        ctx.fillStyle = k % 2 === 0 ? "#2d343d" : "#283038";
        ctx.fill();
        ctx.strokeStyle = "#11151a";
        ctx.lineWidth = 2;
        ctx.stroke();
        for (let j = 0; j < 5; j++) {
          const yy = top0 + (bot0 - top0) * (0.15 + 0.17 * j), xx = x0 + (x1 - x0) * 0.3;
          ctx.beginPath();
          ctx.arc(xx, yy, 3 * near + 1, 0, 2 * Math.PI);
          ctx.fillStyle = (j + k) % 3 ? "#3fb950" : "#58a6ff";
          ctx.fill();
        }
      }
    }
    if (!D.scene) return;
    for (const s of D.scene.sources) {
      if (s.kind !== "talker") continue;
      const x = cam.xOf(s.az);
      if (x === null) continue;
      const depth = s.dist * Math.cos(rad(wrap(s.az - cam.yaw))), f = cam.f;
      const headY = cy - (f * (s.height - 1.3)) / depth, headR = (f * 0.1) / depth, sh = (f * 0.2) / depth;
      ctx.beginPath();
      ctx.moveTo(x - sh, r.y + r.h);
      ctx.lineTo(x - sh, headY + headR * 2.2);
      ctx.quadraticCurveTo(x - sh, headY + headR * 1.4, x - sh * 0.5, headY + headR * 1.4);
      ctx.lineTo(x + sh * 0.5, headY + headR * 1.4);
      ctx.quadraticCurveTo(x + sh, headY + headR * 1.4, x + sh, headY + headR * 2.2);
      ctx.lineTo(x + sh, r.y + r.h);
      ctx.closePath();
      ctx.fillStyle = "#4d5866";
      ctx.fill();
      ctx.beginPath();
      ctx.ellipse(x, headY, headR * 0.85, headR, 0, 0, 2 * Math.PI);
      ctx.fillStyle = "#6e7a88";
      ctx.fill();
      const speaking = s.active.some(([a, b]) => a <= t && t <= b);
      text(x, headY - headR - 16, s.name + (speaking ? "  (talking)" : ""), 17, speaking ? INK : DIM, speaking, "center");
    }
  }

  function drawVideo() {
    const vw = video.videoWidth, vh = video.videoHeight;
    const target = CAM.w / CAM.h;
    let sw = vw, sh = vh;
    if (vw / vh > target) sw = vh * target; else sh = vw / target;
    ctx.drawImage(video, (vw - sw) / 2, (vh - sh) / 2, sw, sh, CAM.x, CAM.y, CAM.w, CAM.h);
  }

  function drawCamera(b) {
    const t = b * BLOCK;
    const useVideo = video && video.readyState >= 2;
    ctx.save();
    ctx.beginPath();
    ctx.rect(CAM.x, CAM.y, CAM.w, CAM.h);
    ctx.clip();
    if (useVideo) drawVideo(); else drawSyntheticScene(t);
    const m = disp[b], top = CAM.y + 60, bot = CAM.y + CAM.h - 120, step = 4;
    for (let x = CAM.x; x < CAM.x + CAM.w; x += step) {
      const v = sampleMap(m, cam.azOf(x + step / 2));
      if (v <= 0.2) continue;
      const [r, g, bl] = heatRGB(0.45 + 0.55 * v), a = (200 * Math.pow((v - 0.2) / 0.8, 1.2)) / 255;
      const gr = ctx.createLinearGradient(0, top, 0, bot);
      for (const [p, k] of [[0, 0], [0.25, 1], [0.6, 0.8], [1, 0]]) gr.addColorStop(p, `rgba(${r},${g},${bl},${a * k})`);
      ctx.fillStyle = gr;
      ctx.fillRect(x, top, step, bot - top);
    }
    // compass tape
    ctx.fillStyle = "rgba(13,17,23,0.78)";
    ctx.fillRect(CAM.x, CAM.y, CAM.w, 44);
    const tapeBot = CAM.y + 44;
    for (let x = CAM.x; x < CAM.x + CAM.w; x += 4) {
      ctx.fillStyle = heat(sampleMap(m, cam.azOf(x + 2)));
      ctx.fillRect(x, tapeBot - 10, 4.5, 10);
    }
    for (let a = -90; a <= 90; a += 10) {
      const tx = cam.xOf(a);
      if (tx === null || !(CAM.x + 10 < tx && tx < CAM.x + CAM.w - 10)) continue;
      line(tx, tapeBot - 10, tx, tapeBot - (a % 30 === 0 ? 18 : 14), DIM, 1.5);
      if (a % 30 === 0) text(tx, CAM.y + 20, a ? `${a > 0 ? "+" : ""}${a}°` : "0°", 14, DIM, false, "center");
    }
    // beam marker
    const az = state.track[b];
    let bx = cam.xOf(az);
    if (bx !== null && cam.visible(az)) {
      const xl = cam.xOf(az + BEAM_HALF_WIDTH) ?? CAM.x, xr = cam.xOf(az - BEAM_HALF_WIDTH) ?? CAM.x + CAM.w;
      ctx.fillStyle = withAlpha(BEAM, 45 / 255);
      ctx.fillRect(xl, CAM.y, xr - xl, CAM.h);
      const y0 = CAM.y + 150, y1 = CAM.y + 470;
      for (const [xx, sgn] of [[xl, 1], [xr, -1]]) {
        line(xx, y0, xx + sgn * 24, y0, BEAM, 3);
        line(xx, y0, xx, y0 + 40, BEAM, 3);
        line(xx, y1, xx + sgn * 24, y1, BEAM, 3);
        line(xx, y1, xx, y1 - 40, BEAM, 3);
      }
      bx = Math.min(Math.max(bx, CAM.x + 80), CAM.x + CAM.w - 80);
      rrect(bx - 70, CAM.y + 54, 140, 32, 6);
      ctx.fillStyle = "rgba(13,17,23,0.82)";
      ctx.fill();
      ctx.strokeStyle = BEAM;
      ctx.lineWidth = 1.5;
      ctx.stroke();
      text(bx, CAM.y + 77, `BEAM  ${az >= 0 ? "+" : ""}${Math.round(az)}°`, 18, BEAM, true, "center");
    }
    // sounds outside the view
    const marks = [];
    if (D.activity[b] > 0) {
      for (const [a] of D.peaks[b]) {
        if (!cam.visible(a)) marks.push([a, DIM, `${directionName(a)} ${a >= 0 ? "+" : ""}${Math.round(a)}°`, false]);
      }
    }
    for (const e of D.events) {
      if (t - e.t >= 0 && t - e.t <= 3 && !cam.visible(e.az)) marks.push([e.az, ALERT, `${e.label} · ${directionName(e.az)}`, true]);
    }
    marks.sort((p, q) => (q[3] ? 1 : 0) - (p[3] ? 1 : 0));
    const used = [];
    for (const [a, col, label, strong] of marks) {
      const rel = wrap(a - cam.yaw), left = rel > 0;
      const y = CAM.y + 140 + ((Math.abs(rel) - cam.hfov / 2) / (180 - cam.hfov / 2)) * (CAM.h - 300);
      if (used.some(([uy, ul]) => Math.abs(y - uy) < 46 && ul === left)) continue;
      used.push([y, left]);
      const x = left ? CAM.x + 18 : CAM.x + CAM.w - 18, sgn = left ? -1 : 1, size = strong ? 22 : 15;
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(x - sgn * size * 1.6, y - size);
      ctx.lineTo(x - sgn * size * 1.6, y + size);
      ctx.closePath();
      ctx.fillStyle = col;
      ctx.fill();
      const tx = x - sgn * (size * 1.6 + 10), fs = strong ? 19 : 16;
      setFont(fs, strong);
      const w = ctx.measureText(label).width;
      rrect(tx - (left ? 0 : w) - 6, y - 15, w + 12, 30, 6);
      ctx.fillStyle = "rgba(13,17,23,0.78)";
      ctx.fill();
      text(tx, y + 7, label, fs, col, strong, left ? "left" : "right");
    }
    ctx.restore();
    rrect(CAM.x, CAM.y, CAM.w, CAM.h, 8);
    ctx.strokeStyle = EDGE;
    ctx.lineWidth = 2;
    ctx.stroke();
    text(CAM.x + 14, CAM.y + CAM.h - 14, `${useVideo ? "camera" : "simulated camera view"} · ${Math.round(cam.hfov)}° wide`, 15, DIM);
  }

  // ---------------------------------------------------------------- radar

  function drawRadar(b) {
    panel(RADAR_BOX, "All around the wearer (top view)");
    const cx = center(RADAR_BOX)[0], cy = RADAR_CY, R = RADAR_R, t = b * BLOCK;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    for (let i = 0; i < 30; i++) {
      const a = cam.yaw - cam.hfov / 2 + (cam.hfov * i) / 29;
      ctx.lineTo(...radarPoint(cx, cy, R, a));
    }
    ctx.closePath();
    ctx.fillStyle = "rgba(255,255,255,0.07)";
    ctx.fill();
    for (const rr of [R / 3, (2 * R) / 3, R]) {
      ctx.beginPath();
      ctx.arc(cx, cy, rr, 0, 2 * Math.PI);
      ctx.strokeStyle = EDGE;
      ctx.lineWidth = 1;
      ctx.stroke();
    }
    for (const [a, lab] of [[0, "front"], [90, "left"], [-90, "right"], [180, "behind"]]) {
      const [qx, qy] = radarPoint(cx, cy, R + 22, a);
      text(qx, qy + 6, lab, 15, DIM, false, "center");
    }
    const poly = (fn, stepDeg) => {
      ctx.beginPath();
      for (let a = -180; a < 180; a += stepDeg) {
        const [x, y] = radarPoint(cx, cy, fn(a), a);
        a === -180 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
      }
      ctx.closePath();
    };
    poly((a) => R * (0.15 + 0.6 * sampleMap(D.steady, a)), 5);
    ctx.strokeStyle = DIM;
    ctx.lineWidth = 1.5;
    ctx.setLineDash([6, 4]);
    ctx.stroke();
    ctx.setLineDash([]);
    const m = disp[b];
    poly((a) => R * (0.1 + 0.85 * Math.pow(sampleMap(m, a), 2)), 2.5);
    ctx.fillStyle = heat(0.75, 110 / 255);
    ctx.fill();
    ctx.strokeStyle = heat(0.85);
    ctx.lineWidth = 2;
    ctx.stroke();
    const az = state.track[b];
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    for (let i = 0; i < 8; i++) ctx.lineTo(...radarPoint(cx, cy, R, az - BEAM_HALF_WIDTH + (2 * BEAM_HALF_WIDTH * i) / 7));
    ctx.closePath();
    ctx.fillStyle = withAlpha(BEAM, 90 / 255);
    ctx.fill();
    ctx.strokeStyle = BEAM;
    ctx.lineWidth = 2;
    ctx.stroke();
    if (D.scene) {
      for (const s of D.scene.sources) {
        const [qx, qy] = radarPoint(cx, cy, R * Math.min(1, 0.35 + s.dist / 6), s.az);
        const active = s.active.some(([a, bb]) => a <= t && t <= bb);
        const col = active && s.kind !== "noise" ? INK : DIM;
        ctx.strokeStyle = col;
        ctx.lineWidth = 1.5;
        ctx.strokeRect(qx - 6, qy - 6, 12, 12);
        text(qx + 10, qy - 8, s.name.replace("Rack fan", "fan"), 13, col);
      }
    }
    for (const e of D.events) {
      if (t - e.t >= 0 && t - e.t <= 3) {
        const [qx, qy] = radarPoint(cx, cy, R * 0.92, e.az);
        ctx.beginPath();
        ctx.arc(qx, qy, 9, 0, 2 * Math.PI);
        ctx.fillStyle = ALERT;
        ctx.fill();
      }
    }
    ctx.beginPath();
    ctx.ellipse(cx, cy, 34, 20, 0, 0, 2 * Math.PI);
    ctx.fillStyle = EDGE;
    ctx.fill();
    ctx.strokeStyle = INK;
    ctx.lineWidth = 2;
    ctx.stroke();
    line(cx, cy - 20, cx, cy - 30, INK, 2);
    const legY = RADAR_BOX.y + RADAR_BOX.h - 14;
    let x = RADAR_BOX.x + 18;
    for (const [col, lab, dash] of [[heat(0.85), "new sounds", false], [DIM, "steady noise", true], [BEAM, "beam", false], [ALERT, "event", false]]) {
      line(x, legY - 6, x + 22, legY - 6, col, 3, dash ? [6, 4] : null);
      x += 30 + text(x + 30, legY, lab, 14, DIM) + 18;
    }
    if (D.scene) {
      ctx.strokeStyle = DIM;
      ctx.lineWidth = 1.5;
      ctx.strokeRect(x, legY - 12, 12, 12);
      text(x + 18, legY, "true position (sim)", 14, DIM);
    }
  }

  // ---------------------------------------------------------------- meters, events, timeline

  function drawMeters(b) {
    panel(METER_BOX, "Level");
    const rows = [[`one mic (${D.names[D.ref]})`, D.level_db[b], DIM], ["beam output", state.beamDb[b], BEAM]];
    rows.forEach(([lab, db, col], i) => {
      const y = METER_BOX.y + 62 + i * 44, x0 = METER_BOX.x + 190, w = 400;
      text(METER_BOX.x + 18, y + 6, lab, 16, INK);
      rrect(x0, y - 12, w, 22, 4);
      ctx.fillStyle = "#21262d";
      ctx.fill();
      const frac = Math.min(Math.max((db + 60) / 60, 0), 1);
      if (frac > 0) {
        rrect(x0, y - 12, Math.max(w * frac, 8), 22, 4);
        ctx.fillStyle = col;
        ctx.fill();
      }
      text(x0 + w + 12, y + 6, `${db.toFixed(1).padStart(5)} dB`, 16, INK, false, "left", true);
    });
    const lo = Math.max(0, b - 10);
    let diff = 0;
    for (let i = lo; i <= b; i++) diff += state.beamDb[i] - D.level_db[i];
    diff /= b - lo + 1;
    text(METER_BOX.x + METER_BOX.w - 18, METER_BOX.y + 28, `beam vs one mic: ${diff >= 0 ? "+" : ""}${diff.toFixed(1)} dB`, 16, BEAM, true, "right");
  }

  function drawEvents(b) {
    panel(EVENT_BOX, "Event log");
    const t = b * BLOCK, sgn = (a) => `${a >= 0 ? "+" : ""}${Math.round(a)}°`;
    const rows = D.events.filter((e) => e.t <= t).map((e) => [e.t, e.label, `${directionName(e.az)} ${sgn(e.az)}`, ALERT]);
    for (const [lt, la] of state.locks) if (lt <= t) rows.push([lt, "beam re-aimed", `${directionName(la)} ${sgn(la)}`, BEAM]);
    rows.sort((p, q) => p[0] - q[0]);
    rows.slice(-4).forEach(([et, lab, where, col], i) => {
      const y = EVENT_BOX.y + 62 + i * 31;
      text(EVENT_BOX.x + 18, y, `${et.toFixed(1).padStart(5)} s`, 16, DIM, false, "left", true);
      text(EVENT_BOX.x + 120, y, lab, 17, col, true);
      text(EVENT_BOX.x + 360, y, where, 17, INK);
    });
    if (!rows.length) text(EVENT_BOX.x + 18, EVENT_BOX.y + 62, "nothing yet", 16, DIM);
  }

  const timelineImg = (() => {
    const nA = D.grid.length, c = document.createElement("canvas");
    c.width = nB;
    c.height = nA;
    const g = c.getContext("2d"), img = g.createImageData(nB, nA);
    for (let b = 0; b < nB; b++) {
      for (let a = 0; a < nA; a++) {
        const [r, gg, bl] = heatRGB(Math.pow(Math.min(1, D.srp[b][a]), 1.3) * 0.95);
        const o = ((nA - 1 - a) * nB + b) * 4;
        img.data[o] = r; img.data[o + 1] = gg; img.data[o + 2] = bl; img.data[o + 3] = 255;
      }
    }
    g.putImageData(img, 0, 0);
    return c;
  })();

  function drawTimeline(b) {
    const r = TIMELINE;
    rrect(r.x - 2, r.y - 2, r.w + 4, r.h + 4, 6);
    ctx.fillStyle = PANEL;
    ctx.fill();
    ctx.strokeStyle = EDGE;
    ctx.lineWidth = 1.5;
    ctx.stroke();
    const inner = { x: r.x + 70, y: r.y + 6, w: r.w - 80, h: r.h - 12 };
    ctx.imageSmoothingEnabled = true;
    ctx.drawImage(timelineImg, inner.x, inner.y, inner.w, inner.h);
    const ty = (az) => inner.y + ((180 - az) / 360) * inner.h;
    const tx = (t) => inner.x + (t / D.duration) * inner.w;
    for (const a of [cam.yaw - cam.hfov / 2, cam.yaw + cam.hfov / 2]) line(inner.x, ty(a), inner.x + inner.w, ty(a), "rgba(255,255,255,0.27)", 1, [4, 4]);
    ctx.beginPath();
    state.track.forEach((a, i) => (i === 0 ? ctx.moveTo(tx(i * BLOCK), ty(a)) : ctx.lineTo(tx(i * BLOCK), ty(a))));
    ctx.strokeStyle = BG;
    ctx.lineWidth = 5;
    ctx.stroke();
    ctx.strokeStyle = "#a5d6ff";
    ctx.lineWidth = 2.5;
    ctx.stroke();
    for (const e of D.events) {
      ctx.beginPath();
      ctx.arc(tx(e.t), ty(e.az), 5, 0, 2 * Math.PI);
      ctx.fillStyle = ALERT;
      ctx.fill();
    }
    line(tx(b * BLOCK), inner.y, tx(b * BLOCK), inner.y + inner.h, INK, 2);
    text(r.x + 8, ty(150) + 5, "left", 13, DIM);
    text(r.x + 8, ty(0) + 5, "front", 13, DIM);
    text(r.x + 8, ty(-150) + 5, "right", 13, DIM);
  }

  function draw(b) {
    state.b = b;
    ctx.fillStyle = BG;
    ctx.fillRect(0, 0, W, H);
    text(24, 52, "Mic-array HUD", 30, INK, true);
    let x = 260;
    if (D.simulated) {
      rrect(x, 26, 230, 34, 8);
      ctx.fillStyle = SIM_BADGE;
      ctx.fill();
      text(x + 115, 50, "SIMULATED RECORDING", 16, BG, true, "center");
      x += 250;
    }
    const listen = state.listen === "mic" ? `  ·  listening to one mic (${D.names[D.ref]})` : "";
    text(x, 50, `Beam: ${state.mode} (${state.kind.toUpperCase()})${listen}`, 18, DIM);
    text(W - 24, 52, `t = ${(b * BLOCK).toFixed(1).padStart(5)} s / ${D.duration.toFixed(1)} s`, 22, INK, true, "right", true);
    drawCamera(b);
    drawTimeline(b);
    drawRadar(b);
    drawMeters(b);
    drawEvents(b);
    $("slider").value = b;
    $("time").textContent = `${(b * BLOCK).toFixed(1)} s`;
  }

  // ---------------------------------------------------------------- audio

  const pcm = (() => {
    const bin = atob(D.pcm_b64), bytes = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
    const dv = new DataView(bytes.buffer), n = D.samples, M = D.names.length, out = [];
    for (let m = 0; m < M; m++) {
      const ch = new Float32Array(n);
      for (let i = 0; i < n; i++) ch[i] = dv.getInt16(2 * (m * n + i), true) / 32768;
      out.push(ch);
    }
    return out;
  })();

  let S = null, gammaInv = null, actx = null, source = null;
  let playing = false, ctxStart = 0, offsetStart = 0, pausedAt = 0;
  const buffers = { beam: null, mic: null };
  const signals = { beam: null, mic: pcm[D.ref] };

  function audioCtx() {
    if (!actx) actx = new (window.AudioContext || window.webkitAudioContext)();
    return actx;
  }

  function toBuffer(sig) {
    const a = audioCtx();
    let peak = 1e-12;
    for (const v of sig) peak = Math.max(peak, Math.abs(v));
    const g = 0.9 / peak;
    let buf;
    try {
      buf = a.createBuffer(1, sig.length, D.fs);
      const ch = buf.getChannelData(0);
      for (let i = 0; i < sig.length; i++) ch[i] = sig[i] * g;
    } catch (e) {
      const ratio = a.sampleRate / D.fs, n = Math.floor(sig.length * ratio);
      buf = a.createBuffer(1, n, a.sampleRate);
      const ch = buf.getChannelData(0);
      for (let i = 0; i < n; i++) {
        const p = i / ratio, k = Math.floor(p), f = p - k;
        ch[i] = ((sig[k] || 0) * (1 - f) + (sig[k + 1] || 0) * f) * g;
      }
    }
    return buf;
  }

  const now = () => (playing ? offsetStart + (actx.currentTime - ctxStart) : pausedAt);

  function stopSource() {
    if (source) {
      source.onended = null;
      try { source.stop(); } catch (e) { /* already stopped */ }
      source.disconnect();
      source = null;
    }
  }

  function startAt(t) {
    const a = audioCtx();
    if (a.state === "suspended") a.resume();
    const which = state.listen;
    if (!buffers[which]) buffers[which] = toBuffer(signals[which]);
    stopSource();
    source = a.createBufferSource();
    source.buffer = buffers[which];
    source.connect(a.destination);
    source.start(0, Math.min(t, D.duration - 0.01));
    source.onended = () => { if (playing) pause(0); };
    ctxStart = a.currentTime;
    offsetStart = t;
    playing = true;
    if (video) syncVideo(t, true);
    $("play").textContent = "Pause";
    requestAnimationFrame(tick);
  }

  function pause(at) {
    pausedAt = at ?? now();
    playing = false;
    stopSource();
    if (video) video.pause();
    $("play").textContent = "Play";
    seek(pausedAt);
  }

  function seek(t) {
    t = Math.min(Math.max(t, 0), D.duration - BLOCK);
    if (playing) startAt(t); else pausedAt = t;
    if (video && !playing) syncVideo(t, false);
    draw(Math.min(Math.floor(t / BLOCK + 1e-6), nB - 1));
  }

  function syncVideo(t, play) {
    const vt = t + (D.video_offset || 0);
    if (vt < 0) return;
    if (Math.abs(video.currentTime - vt) > 0.15) video.currentTime = vt;
    if (play && video.paused) video.play().catch(() => {});
  }

  function tick() {
    if (!playing) return;
    const t = now();
    if (t >= D.duration - 0.01) { pause(0); return; }
    if (video) syncVideo(t, true);
    const b = Math.min(Math.floor(t / BLOCK), nB - 1);
    if (b !== state.b || video) draw(b);
    requestAnimationFrame(tick);
  }

  // ---------------------------------------------------------------- steering

  function steer(track, mode) {
    state.track = Float64Array.from(track);
    state.mode = mode;
    const y = DSP.beamform(S, D.pos, D.fs, D.nfft, D.hop, D.frames_per_block, state.track, state.kind, gammaInv, D.samples);
    signals.beam = y;
    buffers.beam = null;
    state.beamDb = DSP.blockDb(y, D.fs, nB, BLOCK);
    state.locks = beamLocks(state.track);
    if (playing && state.listen === "beam") startAt(now());
    draw(state.b);
  }

  function setListen(which) {
    state.listen = which;
    $("listenBeam").classList.toggle("on", which === "beam");
    $("listenMic").classList.toggle("on", which === "mic");
    if (playing) startAt(now()); else draw(state.b);
  }

  canvas.addEventListener("click", (ev) => {
    if (!state.ready) return;
    const r = canvas.getBoundingClientRect();
    const x = ((ev.clientX - r.left) * W) / r.width, y = ((ev.clientY - r.top) * H) / r.height;
    let az;
    if (contains(CAM, x, y)) az = cam.azOf(x);
    else if (contains(RADAR_BOX, x, y)) az = (Math.atan2(-(x - center(RADAR_BOX)[0]), -(y - RADAR_CY)) * 180) / Math.PI;
    else return;
    steer(new Array(nB).fill(az), `fixed ${az >= 0 ? "+" : ""}${Math.round(az)}° (clicked)`);
  });

  $("play").addEventListener("click", () => (playing ? pause() : startAt(pausedAt)));
  $("auto").addEventListener("click", () => steer(D.auto_track, "auto-follow"));
  $("listenBeam").addEventListener("click", () => setListen("beam"));
  $("listenMic").addEventListener("click", () => setListen("mic"));
  $("kind").addEventListener("change", (e) => { state.kind = e.target.value; steer(state.track, state.mode); });
  $("slider").max = nB - 1;
  $("slider").addEventListener("input", (e) => seek(Number(e.target.value) * BLOCK));
  document.addEventListener("keydown", (e) => {
    if (!state.ready || e.target.tagName === "SELECT") return;
    if (e.code === "Space") { e.preventDefault(); playing ? pause() : startAt(pausedAt); }
    else if (e.key === "ArrowRight" || e.key === "ArrowLeft") {
      e.preventDefault();
      const step = (e.shiftKey ? 10 : 1) * BLOCK * (e.key === "ArrowRight" ? 1 : -1);
      seek(state.b * BLOCK + step);
    } else if (e.key === "a" || e.key === "A") steer(D.auto_track, "auto-follow");
    else if (e.key === "m" || e.key === "M") setListen(state.listen === "beam" ? "mic" : "beam");
  });
  if (video) video.addEventListener("seeked", () => { if (!playing) draw(state.b); });

  draw(0);
  setTimeout(() => {
    S = DSP.stft(pcm, D.nfft, D.hop);
    gammaInv = DSP.diffuseInverses(D.pos, D.fs, D.nfft, 0.05);
    steer(D.auto_track, "auto-follow");
    state.ready = true;
    $("play").disabled = false;
    $("play").textContent = "Play";
    window.HUD_READY = true;
  }, 30);

  window.HUD = { draw, seek, steer, setListen, state };
})();
