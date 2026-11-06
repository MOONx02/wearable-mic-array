// Mic array 360 in the browser: reads the ESP32-S3 (cam_mic_record8) USB stream, finds the direction of
// sounds with SRP-PHAT over every mic pair (same maths as mic_camera_view360.py), draws the camera
// picture with a glow / edge arrows, a 360° radar, mic levels, and live subtitles: the array's voice focus
// (MVDR + Wiener, as in mic_caption_view.py) feeds an offline recognizer (Vosk), placed by the talker's direction.
"use strict";

const FS = 16000, CH = 8, C = 343, SEG = 1024, NFFT = 2048, INTERP = 8;
const BAND = [300, 6000], HOLD_S = 2.5, HISTORY_S = 10, BEHIND_DEG = 135, FB_SURE = 0.25;
const DEAD_PTP = 8, STUCK_FRACTION = 0.5, CAPTION_HOLD_S = 9;
// front/back with a body between the chest and back plates (Model.plateScore), as in mic_camera_view360.py:
// plates more than PLATE_GAP m apart; the longest way round a body (samples); the band that bends round to the
// far plate and the band the body shadows (Hz); the far-plate drop that counts as a full vote (dB); how far
// the score raises the front or back half of the azimuth curve
const PLATE_GAP = 0.08, PLATE_MAX_LAG = 24, PLATE_TIME_BAND = [300, 3000], PLATE_LEVEL_BAND = [1000, 4000];
const PLATE_SHADOW_DB = 6, PLATE_TILT = 0.6;
// a clap's direction: its first 12 ms (before echoes), then nothing new until it's 6 dB louder again or,
// 0.4 s on, has stopped fading
const ONSET_GATE_S = 0.012, RETRIGGER_DB = 6, STEADY_S = 0.4;
// 150 Hz Butterworth high-pass (b0, b1, b2, a1, a2) for the levels and the trigger
const HP = (() => {
  const w = (2 * Math.PI * 150) / FS, cw = Math.cos(w), al = Math.sin(w) / Math.SQRT2, a0 = 1 + al;
  return [(1 + cw) / 2 / a0, -(1 + cw) / a0, (1 + cw) / 2 / a0, (-2 * cw) / a0, (1 - al) / a0];
})();
// elevations searched when the mics aren't all at one height (degrees; talkers' faces are above the chest)
const ELEV = [-30, -20, -10, 0, 10, 20, 30, 40, 50, 60];
const VIDS = [0x303a, 0x2886];
const COLORS = ["#58a6ff", "#f0883e", "#3fb950", "#d2a8ff", "#ff7b72", "#e3b341", "#79c0ff", "#56d4dd"];
const BG = "#0d1117", PANEL = "#161b22", EDGE = "#30363d", INK = "#e6edf3", DIM = "#8b949e";
const GLOW = "#ffa657", LOUD = "#ff7b72", SPEAKER = "#58a6ff";

// channel: [name, x mm toward the wearer's left, y mm toward the front, z mm up (optional)]
const FRONT = { 0: ["front outer L", 68.6, 22], 1: ["front outer R", -68.6, 22],
                2: ["front inner L", 22.9, 22], 3: ["front inner R", -22.9, 22] };
// cam_mic_vest6: the chest plate's row of four with a pair 60 mm above the inner two
const VEST6 = { 0: ["inner R", -22.9, 0, 0], 1: ["inner L", 22.9, 0, 0], 2: ["outer R", -68.6, 0, 0],
                3: ["outer L", 68.6, 0, 0], 4: ["top R", -22.9, 0, 60], 5: ["top L", 22.9, 0, 60] };
// cam_mic_vest8: the chest plate plus the back plate's pair, level with the row, chest to back as worn
const BACK_DEPTH_MM = 150;
const VEST8 = { ...VEST6, 6: ["back R", -68.6, -BACK_DEPTH_MM, 0], 7: ["back L", 68.6, -BACK_DEPTH_MM, 0] };
const PRESETS = {
  vest8: { name: "8 mics: vest chest 6 + back pair", geom: VEST8 },
  vest6: { name: "6 mics: vest chest, 4 in a row + 2 above", geom: VEST6 },
  rear5: { name: "5 mics: 4 front + rear centre", geom: { ...FRONT, 4: ["rear centre", 0, -22] } },
  six: { name: "6 mics: 4 front + 2 back", geom: { ...FRONT, 5: ["back R", -68.6, -22], 7: ["back L", 68.6, -22] } },
  front4: { name: "4 front mics (no front/back)", geom: { ...FRONT } },
};

const S = Object.assign({ layout: "vest8", hfov: 60, offset: 0, tilt: 0, swap: false, swapfb: false, rotate: 0,
                          mirror: false, thr: 12, cc: true, focus: true, find: false, findMode: "follow",
                          roll: 0, kel: 1, camX: 0, camZ: 0, dist: 1.5, calDist: 1, cal: [], calFine: true, camOff: false },
                        JSON.parse(localStorage.getItem("mic360") || "{}"));
if (S.find) S.cc = false;
if (!["follow", "alarm", "evac"].includes(S.findMode)) S.findMode = "follow";
if (!PRESETS[S.layout]) S.layout = "vest8";
if (!S.vest8) {  // once: tablets that saved the six-mic vest move to the eight-mic one
  if (S.layout === "vest6") S.layout = "vest8";
  S.vest8 = true;
}
const save = () => localStorage.setItem("mic360", JSON.stringify(S));
const wrap = (a) => ((((a + 180) % 360) + 360) % 360) - 180;
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const $ = (id) => document.getElementById(id);

// ------------------------------------------------------------------ FFT (radix 2, in place)

function makeFFT(n) {
  const bits = Math.log2(n), rev = new Uint32Array(n);
  for (let i = 0; i < n; i++) {
    let r = 0;
    for (let b = 0, x = i; b < bits; b++, x >>= 1) r = (r << 1) | (x & 1);
    rev[i] = r;
  }
  const wr = new Float64Array(n / 2), wi = new Float64Array(n / 2);
  for (let i = 0; i < n / 2; i++) {
    wr[i] = Math.cos((2 * Math.PI * i) / n);
    wi[i] = -Math.sin((2 * Math.PI * i) / n);
  }
  return (re, im) => {
    for (let i = 0; i < n; i++) {
      const j = rev[i];
      if (j > i) {
        let t = re[i]; re[i] = re[j]; re[j] = t;
        t = im[i]; im[i] = im[j]; im[j] = t;
      }
    }
    for (let size = 2; size <= n; size <<= 1) {
      const half = size >> 1, step = n / size;
      for (let i = 0; i < n; i += size) {
        for (let k = 0; k < half; k++) {
          const c = wr[k * step], s = wi[k * step], a = i + k, b = a + half;
          const tr = re[b] * c - im[b] * s, ti = re[b] * s + im[b] * c;
          re[b] = re[a] - tr; im[b] = im[a] - ti;
          re[a] += tr; im[a] += ti;
        }
      }
    }
  };
}
const fft = makeFFT(NFFT);

// ------------------------------------------------------------------ analysis

class Model {
  constructor() {
    this.N = FS * 2;
    this.x = Array.from({ length: CH }, () => new Float32Array(this.N));
    // the same above 150 Hz, for the levels and the trigger: body thumps, footsteps and room rumble sit
    // below that and would otherwise move every mic at once (direction finding only looks above 300 Hz)
    this.xh = Array.from({ length: CH }, () => new Float32Array(this.N));
    this.hp = Array.from({ length: CH }, () => new Float64Array(4));
    this.wp = 0;
    this.total = 0;
    this.dc = new Float64Array(CH);
    this.level = new Float64Array(CH).fill(-120);
    this.floor = new Float64Array(CH).fill(-70);
    this.levHist = Array.from({ length: CH }, () => []);
    this.dead = new Array(CH).fill(true);
    this.dets = [];
    this.last = null;
    this.steer = new Map();
    this.blk = new Float32Array(800);
    this.half = new Float32Array(FS / 2);
    this.seg = new Float32Array(SEG);
    this.setGeom(PRESETS[S.layout].geom);
  }
  setGeom(g) {
    this.geom = g;
    this.chans = Object.keys(g).map(Number).sort((a, b) => a - b);
    this.steer.clear();
  }
  get now() { return this.total / FS; }

  push(s, frames) {
    for (let c = 0; c < CH; c++) {
      let m = 0;
      for (let i = 0; i < frames; i++) m += s[i * CH + c];
      m /= frames;
      this.dc[c] = this.total === 0 ? m : 0.95 * this.dc[c] + 0.05 * m;
      const d = this.dc[c], buf = this.x[c], hb = this.xh[c], z = this.hp[c];
      let w = this.wp, [x1, x2, y1, y2] = z;
      for (let i = 0; i < frames; i++) {
        const v = s[i * CH + c] - d;
        buf[w] = v;
        const y = HP[0] * v + HP[1] * x1 + HP[2] * x2 - HP[3] * y1 - HP[4] * y2;
        x2 = x1; x1 = v; y2 = y1; y1 = y;
        hb[w] = y;
        if (++w === this.N) w = 0;
      }
      z[0] = x1; z[1] = x2; z[2] = y1; z[3] = y2;
    }
    this.wp = (this.wp + frames) % this.N;
    this.total += frames;
  }
  tail(c, n, out, src = this.x) {
    const buf = src[c];
    let r = (this.wp - n + this.N) % this.N;
    for (let i = 0; i < n; i++) {
      out[i] = buf[r];
      if (++r === this.N) r = 0;
    }
    return out;
  }

  tick(thr, voice = false) {
    if (this.total < FS / 10) return;
    this.ticks = (this.ticks || 0) + 1;
    for (let c = 0; c < CH; c++) {
      const blk = this.tail(c, 800, this.blk, this.xh);
      let m = 0;
      for (let i = 0; i < 800; i++) m += blk[i];
      m /= 800;
      let v = 0;
      for (let i = 0; i < 800; i++) v += (blk[i] - m) * (blk[i] - m);
      const lv = 10 * Math.log10(v / 800 / (32768 * 32768) + 1e-12);
      this.level[c] = lv;
      // floor = the quiet level of the last ~5 s (10th percentile): settles on a steady fan within a second,
      // and pauses between words keep it at the real background while someone talks
      const hist = this.levHist[c];
      hist.push(lv);
      if (hist.length > 150) hist.shift();
      const sorted = hist.slice().sort((a, b) => a - b);
      this.floor[c] = sorted[Math.floor(sorted.length * 0.1)];
      if (this.total < FS / 2 || this.ticks % 15) continue;  // the dead-line check looks at 0.5 s anyway
      const h = this.tail(c, FS / 2, this.half);
      let lo = Infinity, hi = -Infinity, same = 0;
      for (let i = 0; i < h.length; i++) {
        const x = h[i];
        if (x < lo) lo = x;
        if (x > hi) hi = x;
        if (i && x === h[i - 1]) same++;
      }
      this.dead[c] = hi - lo < DEAD_PTP || same > STUCK_FRACTION * h.length;
    }
    if (!S.find && !cal.on) this.direction(thr, voice);  // in Find mode and calibration only chirps are shown
  }

  // Channel that carries the mic at geometry slot ch: itself, or its data-line partner when the mic is
  // actually on the other SEL slot.
  source(ch) {
    if (!this.dead[ch]) return ch;
    const other = ch ^ 1;
    if (!this.dead[other] && !(other in this.geom)) return other;
    return null;
  }
  active() {
    const out = [];
    for (const ch of this.chans) {
      const s = this.source(ch);
      if (s !== null) out.push([ch, s]);
    }
    return out;
  }

  steering(chans, top = BAND[1], lo = BAND[0]) {
    const key = chans.join(",") + "@" + lo + "-" + top;
    let st = this.steer.get(key);
    if (st) return st;
    const pos = chans.map((c) => [this.geom[c][1] / 1000, this.geom[c][2] / 1000, (this.geom[c][3] || 0) / 1000]);
    const ptp = (i, ps = pos) => ps.length < 2 ? 0 : Math.max(...ps.map((p) => p[i])) - Math.min(...ps.map((p) => p[i]));
    const ptpy = ptp(1);
    // chest and back plates with a body between: sound bends round it, so pairs across the body don't have
    // the straight-line delays. They only vote on front/back (plateScore); the angle comes from pairs on one plate.
    let plates = null, spread = Math.hypot(ptp(0), ptpy, ptp(2));
    const ptpz = ptp(2);
    if (ptpy > PLATE_GAP) {
      const ys = pos.map((p) => p[1]), mid = (Math.max(...ys) + Math.min(...ys)) / 2;
      const front = [], back = [];
      pos.forEach((p, i) => (p[1] > mid ? front : back).push(i));
      const reach = (ix) => { const ps = ix.map((i) => pos[i]); return Math.hypot(ptp(0, ps), ptp(1, ps), ptp(2, ps)); };
      spread = Math.max(reach(front), reach(back));
      const band = (b) => { const r = [Math.max(lo, b[0]), Math.min(top, b[1])]; return r[1] - r[0] < 500 ? [lo, top] : r; };
      const tb = band(PLATE_TIME_BAND), lb = band(PLATE_LEVEL_BAND);
      const tk0 = Math.ceil((tb[0] * NFFT) / FS), tnb = Math.floor((tb[1] * NFFT) / FS) - tk0 + 1, TL = 2 * PLATE_MAX_LAG + 1;
      const tcos = new Float32Array(TL * tnb), tsin = new Float32Array(TL * tnb);
      for (let l = 0; l < TL; l++) {
        for (let b = 0; b < tnb; b++) {
          const ph = (2 * Math.PI * (tk0 + b) * (l - PLATE_MAX_LAG)) / NFFT;
          tcos[l * tnb + b] = Math.cos(ph);
          tsin[l * tnb + b] = Math.sin(ph);
        }
      }
      plates = { front, back, tk0, tnb, tcos, tsin, lk0: Math.ceil((lb[0] * NFFT) / FS), lk1: Math.floor((lb[1] * NFFT) / FS) };
    }
    const m = (Math.ceil((spread / C) * FS) + 2) * INTERP;
    // directions: 360 azimuths × the elevations (just 0° for a flat array), index k * nel + e
    const els = ptpz > 0.01 ? ELEV : [0], nel = els.length, G = 360 * nel;
    const t = pos.map(([x, y, z]) => {
      const a = new Float64Array(G);
      for (let k = 0; k < 360; k++) {
        const th = ((k - 180) * Math.PI) / 180;
        for (let e = 0; e < nel; e++) {
          const ph = (els[e] * Math.PI) / 180;
          a[k * nel + e] = -((x * Math.sin(th) + y * Math.cos(th)) * Math.cos(ph) + z * Math.sin(ph)) / C;
        }
      }
      return a;
    });
    const pairs = [];
    for (let i = 0; i < chans.length; i++) {
      for (let j = i + 1; j < chans.length; j++) {
        const onFront = plates ? plates.front.includes(i) : true;
        if (plates && onFront !== plates.front.includes(j)) continue;
        const idx = new Int32Array(G);
        for (let g = 0; g < G; g++) {
          idx[g] = Math.min(2 * m, Math.max(0, Math.round((t[j][g] - t[i][g]) * FS * INTERP) + m));
        }
        pairs.push([i, j, idx, onFront]);
      }
    }
    if (plates) {
      plates.nFront = pairs.filter((p) => p[3]).length;
      plates.nBack = pairs.length - plates.nFront;
    }
    // GCC-PHAT is only needed at the lags the array can produce, so evaluate those directly instead of
    // an 8x zero-padded inverse FFT per pair.
    const k0 = Math.ceil((lo * NFFT) / FS), k1 = Math.floor((top * NFFT) / FS);
    const nb = k1 - k0 + 1, L = 2 * m + 1, N = NFFT * INTERP;
    const cosT = new Float32Array(L * nb), sinT = new Float32Array(L * nb);
    for (let l = 0; l < L; l++) {
      for (let b = 0; b < nb; b++) {
        const ph = (2 * Math.PI * (k0 + b) * (l - m)) / N;
        cosT[l * nb + b] = Math.cos(ph);
        sinT[l * nb + b] = Math.sin(ph);
      }
    }
    st = { pairs, m, L, k0, nb, cosT, sinT, hasBack: ptpy > 0.01, els, nel, plates };
    this.steer.set(key, st);
    return st;
  }

  // samples [start, start + n) of channel c on the stream clock (must still be in the 2 s buffer)
  at(c, start, n, out, src = this.x) {
    const buf = src[c];
    let r = (((this.wp - (this.total - start)) % this.N) + this.N) % this.N;
    for (let i = 0; i < n; i++) {
      out[i] = buf[r];
      if (++r === this.N) r = 0;
    }
    return out;
  }

  // over the latest SEG samples, or the SEG from stream sample `start`; len < SEG keeps only its first len
  // samples, with tapered ends
  srp(srcs, st, start = null, len = SEG) {
    const re = [], im = [], edge = Math.min(16, len >> 2);
    for (const s of srcs) {
      const r = new Float64Array(NFFT), i = new Float64Array(NFFT);
      const seg = start === null ? this.tail(s, SEG, this.seg) : this.at(s, start, SEG, this.seg);
      for (let n = 0; n < len; n++) {
        const k = Math.min(n, len - 1 - n);
        r[n] = len < SEG && k < edge ? seg[n] * (0.5 - 0.5 * Math.cos((Math.PI * (k + 0.5)) / edge)) : seg[n];
      }
      fft(r, i);
      re.push(r);
      im.push(i);
    }
    const grid = new Float64Array(360 * st.nel), cc = new Float64Array(st.L);
    const gr = new Float64Array(st.nb), gi = new Float64Array(st.nb);
    // with two plates, the angle comes from the plate facing the sound: the far one hears it only after it
    // bends round the body, which pulls its delays toward the side
    const P = st.plates, score = P ? this.plateScore(re, im, P) : 0;
    const [wFront, wBack] = plateWeights(P, score);
    for (const [i, j, idx, onFront] of st.pairs) {
      const w = onFront ? wFront : wBack;
      for (let b = 0; b < st.nb; b++) {
        const k = st.k0 + b;
        const ar = re[j][k], ai = im[j][k], br = re[i][k], bi = -im[i][k];
        const xr = ar * br - ai * bi, xi = ar * bi + ai * br, mag = Math.hypot(xr, xi) + 1e-12;
        gr[b] = xr / mag;
        gi[b] = xi / mag;
      }
      for (let l = 0; l < st.L; l++) {
        const o = l * st.nb;
        let s = 0;
        for (let b = 0; b < st.nb; b++) s += gr[b] * st.cosT[o + b] - gi[b] * st.sinT[o + b];
        cc[l] = s;
      }
      for (let g = 0; g < grid.length; g++) grid[g] += w * cc[idx[g]];
    }
    const curve = toAzimuth(grid, st);
    if (P) tilt(curve, score);
    return curve;
  }

  // Front (+1) to back (-1) from the chest and back plates: which plate hears the sound first (GCC-PHAT over
  // the delays sound bending round a body can have) and which is louder in the band the body shadows.
  plateScore(re, im, P) {
    if (!P.front.length || !P.back.length) return 0;
    const LW = PLATE_MAX_LAG, cc = new Float64Array(2 * LW + 1);
    const gr = new Float64Array(P.tnb), gi = new Float64Array(P.tnb);
    for (const i of P.front) {
      for (const j of P.back) {
        for (let b = 0; b < P.tnb; b++) {
          const k = P.tk0 + b, ar = re[j][k], ai = im[j][k], br = re[i][k], bi = -im[i][k];
          const xr = ar * br - ai * bi, xi = ar * bi + ai * br, mag = Math.hypot(xr, xi) + 1e-12;
          gr[b] = xr / mag;
          gi[b] = xi / mag;
        }
        for (let l = 0; l < cc.length; l++) {
          const o = l * P.tnb;
          let s = 0;
          for (let b = 0; b < P.tnb; b++) s += gr[b] * P.tcos[o + b] - gi[b] * P.tsin[o + b];
          cc[l] += s;
        }
      }
    }
    let later = 0, earlier = 0;  // the back plate later = in front
    for (let l = 1; l <= LW; l++) {
      later = Math.max(later, cc[LW + l]);
      earlier = Math.max(earlier, cc[LW - l]);
    }
    const timing = (later - earlier) / (later + earlier + 1e-9);
    const level = (ix) => {
      let sum = 0;
      for (const c of ix) {
        let p = 0;
        for (let k = P.lk0; k <= P.lk1; k++) p += re[c][k] ** 2 + im[c][k] ** 2;
        sum += 10 * Math.log10(p + 1e-12);
      }
      return sum / ix.length;
    };
    const shadow = level(P.back) - level(P.front);
    return 0.5 * timing + 0.5 * Math.tanh(-shadow / PLATE_SHADOW_DB);
  }

  // voice: someone is talking (voice focus says so), so search only the voice band, where the talker
  // beats a fan; a clap keeps the full band
  direction(thr, voice = false) {
    const act = this.active();
    if (this.total < FS / 2 || act.length < 2) return;
    const srcs = act.map((a) => a[1]);
    let strength = -Infinity;
    for (const s of srcs) strength = Math.max(strength, this.level[s] - this.floor[s]);
    if (strength < thr) { this.ev = null; return; }
    // where missing packets were joined up, every mic jumps at the same instant: not a sound
    if (this.gapAt !== undefined && this.total - this.gapAt < SEG + 800) return;
    if (!voice) return this.transient(act, srcs, strength);
    if (this.last && this.now - this.last.t < 0.08) return;
    const st = this.steering(act.map((a) => a[0]), voice ? 3400 : BAND[1]);
    // while someone talks, show the whole phrase's direction (as the subtitles do), not one 64 ms snapshot
    const phrase = voice ? speechCurve() : null;
    const curve = phrase && phrase.n >= 5 ? phrase.sum : this.srp(srcs, st);
    this.add({ t: this.now, ...pick(curve, st), strength, curve, n: act.length, voice });
  }

  // A clap or snap: its direction comes from its first ONSET_GATE_S. After that the room's echoes arrive
  // from every side, so while it only dies away no new direction is taken. It counts again when it gets
  // louder (a new sound on top) or stops fading (a steady sound, where the latest SEG is fine).
  transient(act, srcs, strength) {
    let ev = this.ev;
    if (ev && strength > ev.low + RETRIGGER_DB && this.now - ev.t > 0.1) ev = null;
    if (ev) {
      ev.low = Math.min(ev.low, strength);
      if (this.now - ev.ref[0] >= 0.2) {  // an echo tail keeps falling; even a big hall's loses 3 dB in 0.2 s
        ev.holding = strength > ev.ref[1] - 3;
        ev.ref = [this.now, strength];
      }
      if (ev.at === null) {
        const steady = this.now - ev.t > STEADY_S && ev.holding;
        if (!steady || (this.last && this.now - this.last.t < 0.08)) return;
        const st = this.steering(act.map((a) => a[0]));
        const curve = this.srp(srcs, st);
        this.add({ t: this.now, ...pick(curve, st), strength, curve, n: act.length, voice: false });
        return;
      }
    } else {
      ev = this.ev = { t: this.now, low: strength, ref: [this.now, strength], holding: false, at: this.onset(srcs) };
    }
    const pre = Math.round(0.002 * FS), gate = Math.round(ONSET_GATE_S * FS);
    if (this.total < ev.at + gate) return;  // the rest of its first ms arrive with the next tick
    const st = this.steering(act.map((a) => a[0]));
    const curve = this.srp(srcs, st, ev.at - pre, pre + gate);
    ev.at = null;
    this.add({ t: this.now, ...pick(curve, st), strength, curve, n: act.length, voice: false });
  }

  // stream sample where the newest sound starts: the first in the level window (800 samples) where the
  // mics' summed 1 ms energy climbs 35% of the way from where the window starts to its peak
  onset(srcs) {
    const n = 800, w = 16, p = new Float64Array(n), e = new Float64Array(n);
    for (const s of srcs) {
      const h = this.tail(s, n, this.blk, this.xh);
      for (let i = 0; i < n; i++) p[i] += h[i] * h[i];
    }
    let acc = 0, peak = 0;
    for (let i = 0; i < n; i++) {
      acc += p[i] - (i >= w ? p[i - w] : 0);
      e[i] = acc;
      if (acc > peak) peak = acc;
    }
    let base = 0;
    for (let i = w; i < w + 64; i++) base += e[i] / 64;
    const cut = base + 0.35 * (peak - base);
    let i = w;
    while (i < n - 1 && e[i] < cut) i++;
    return this.total - n + i - (w >> 1);
  }

  add(det) {
    this.dets.push(det);
    this.last = det;
    while (this.dets.length && this.now - this.dets[0].t > HISTORY_S) this.dets.shift();
  }
}

// SRP over azimuth × elevation down to an azimuth curve: the cut through the peak's elevation, kept in
// curve.el (degrees) when the array has height. (Each azimuth's best elevation instead would smear the
// peak sideways, since tilting up and turning toward the row's end change the row's delays alike.)
function toAzimuth(grid, st) {
  if (st.nel === 1) return grid;
  let g = 0;
  for (let i = 1; i < grid.length; i++) if (grid[i] > grid[g]) g = i;
  const e = g % st.nel, curve = new Float64Array(360);
  for (let k = 0; k < 360; k++) curve[k] = grid[k * st.nel + e];
  curve.el = st.els[e];
  if (e > 0 && e < st.nel - 1) {  // parabola through the peak and its neighbours, between grid steps
    const a = grid[g - 1], b = grid[g], c = grid[g + 1], den = a - 2 * b + c;
    if (den < 0) curve.el += (0.5 * (a - c) / den) * (st.els[e + 1] - st.els[e]);
  }
  return curve;
}

// Per-pair weights [chest plate, back plate] for the SRP sum: the plate facing the sound (by the plate score)
// sets the angle, each plate's pairs normalised so the chest's many pairs don't outvote the back's one.
function plateWeights(P, score) {
  if (!P) return [1, 1];
  if (!P.nBack) return [1, 0];
  if (!P.nFront) return [0, 1];
  return [(1 + score) / 2 / P.nFront, (1 - score) / 2 / P.nBack];
}

// Raises the front or back half of a front/back-mirrored azimuth curve by the plate score, so the peak and
// pick's front/back margin follow it; at the sides it barely matters, so it fades. In place (keeps curve.el).
function tilt(curve, score) {
  const sorted = Array.from(curve).sort((a, b) => a - b), med = (sorted[179] + sorted[180]) / 2;
  for (let k = 0; k < 360; k++) {
    const above = curve[k] - med;
    if (above > 0) curve[k] += above * PLATE_TILT * score * Math.cos(((k - 180) * Math.PI) / 180);
  }
}

// Peak of an SRP curve: angle, spread (sigma), elevation (null for a flat array) and how sure front vs
// back is (null without a back mic, and then the angle is folded into the front half).
function pick(curve, st) {
  let k = 0;
  for (let i = 1; i < 360; i++) if (curve[i] > curve[k]) k = i;
  const elev = curve.el ?? null;
  let angle = k - 180;
  const peak = curve[k], sorted = Array.from(curve).sort((a, b) => a - b);
  const base = (sorted[179] + sorted[180]) / 2, span = Math.max(peak - base, 1e-9);
  const cut = peak - 0.3 * span;
  let lo = k, hi = k;
  while (curve[(lo - 1 + 360) % 360] >= cut && k - lo < 179) lo--;
  while (curve[(hi + 1) % 360] >= cut && hi - k < 179) hi++;
  const sigma = Math.min(60, Math.max(3, (hi - lo) / 2));
  let fb = null;
  if (st.hasBack) fb = (peak - curve[(((Math.round(wrap(180 - angle)) + 180) % 360) + 360) % 360]) / span;
  else if (Math.abs(angle) > 90) angle = Math.sign(angle) * (180 - Math.abs(angle));
  return { angle, sigma, fb, elev };
}

// ------------------------------------------------------------------ Find mode: beacons and alarms

// Beacon codes: each is a 150 ms sweep through 2.5–6.5 kHz with its own shape (beacon.html plays them; keep
// the two in step). Rising, falling and up-then-down sweeps match each other at only ~10%, so matched
// filters tell them apart even when they overlap, and voices, fans and claps don't look like any of them.
// SRP-PHAT over just their band gives the direction.
const CHIRP_LO = 2500, CHIRP_HI = 6500, CHIRP_S = 0.15;
const BLEN = Math.round(CHIRP_S * FS), BN = 8192, BEACON_SCORE = 0.3;
const FOX = "#3fb950", DANGER = "#f85149", ALARM_COL = "#e3b341";
const CODES = {
  leader: { name: "Leader", shape: [[0, CHIRP_LO], [1, CHIRP_HI]], period: 0.5, color: FOX },
  exit: { name: "Exit", shape: [[0, CHIRP_HI], [1, CHIRP_LO]], period: 0.5, color: FOX },
  danger: { name: "Danger", shape: [[0, CHIRP_LO], [0.5, CHIRP_HI], [1, CHIRP_LO]], period: 0.3, color: DANGER },
  calib: { name: "Calibration", shape: [[0, CHIRP_HI], [0.5, CHIRP_LO], [1, CHIRP_HI]], period: 0.4, color: "#56d4dd" },
};
const KINDS = { ...CODES, alarm: { name: "Alarm", color: ALARM_COL } };
const FIND_MODES = {
  follow: { name: "Follow a leader", codes: ["leader"] },
  alarm: { name: "Where's the alarm", codes: [] },
  evac: { name: "Evacuate", codes: ["exit", "danger"] },
};
const kindColor = (d) => (d.kind ? KINDS[d.kind].color : d.strength > 30 ? LOUD : GLOW);

// A code's sweep at sample rate sr: frequency piecewise linear through its shape (time as a fraction of
// the chirp), 10 ms raised-cosine fades.
function codeChirp(code, sr = FS) {
  const shape = CODES[code].shape, len = Math.round(CHIRP_S * sr), fade = 0.01 * sr, out = new Float64Array(len);
  let ph = 0, k = 0;
  for (let i = 0; i < len; i++) {
    const u = i / len;
    while (k < shape.length - 2 && u > shape[k + 1][0]) k++;
    const [u0, f0] = shape[k], [u1, f1] = shape[k + 1];
    out[i] = Math.sin(ph) * (0.5 - 0.5 * Math.cos(Math.PI * Math.min(1, i / fade, (len - 1 - i) / fade)));
    ph += (2 * Math.PI * (f0 + ((f1 - f0) * (u - u0)) / (u1 - u0))) / sr;
  }
  return out;
}

class Beacon {
  constructor() {
    this.fft = makeFFT(BN);
    this.k0 = Math.floor(((CHIRP_LO - 300) * BN) / FS);
    this.k1 = Math.ceil(((CHIRP_HI + 300) * BN) / FS);
    this.tmpl = {};
    for (const code of Object.keys(CODES)) {
      const re = new Float64Array(BN), im = new Float64Array(BN);
      re.set(codeChirp(code));
      this.fft(re, im);
      // analytic template (positive frequencies only, doubled), so the filter output's size is its envelope
      const nb = this.k1 - this.k0 + 1, tRe = new Float64Array(nb), tIm = new Float64Array(nb);
      let e = 0;
      for (let b = 0; b < nb; b++) {
        tRe[b] = 2 * re[this.k0 + b];
        tIm[b] = 2 * im[this.k0 + b];
        e += tRe[b] ** 2 + tIm[b] ** 2;
      }
      this.tmpl[code] = { tRe, tIm, norm: Math.sqrt(e / BN) };
    }
    this.done = 0;  // stream sample up to which chirp starts have been checked
    this.dets = [];
    this.buf = new Float32Array(BN);
    this.outs = new Map();  // filter outputs per code and mic, reused between calls
  }

  reset() { this.done = model.total; this.dets = []; }

  out(code, i) {
    const key = code + i;
    if (!this.outs.has(key)) this.outs.set(key, [new Float64Array(BN), new Float64Array(BN)]);
    return this.outs.get(key);
  }

  // Matched filters for the given codes over the chirp starts since the last call; each hit gets its
  // direction from the chirp itself. Returns the new hits.
  process(codes) {
    const act = model.active(), found = [];
    if (!codes.length || act.length < 2 || model.total < FS) return found;
    let adv = model.total - this.done;
    if (adv < 2048) return found;
    if (adv > BN - BLEN) adv = BN - BLEN;
    const W = adv + BLEN, start = model.total - W, srcs = act.map((a) => a[1]), n = srcs.length;
    const energy = new Float64Array(adv), scores = codes.map(() => new Float64Array(adv)), den = new Float64Array(adv);
    const re = new Float64Array(BN), im = new Float64Array(BN), yr = new Float64Array(BN), yi = new Float64Array(BN);
    const cum = new Float64Array(W + 1);
    srcs.forEach((s, si) => {
      model.at(s, start, W, this.buf);
      re.fill(0);
      im.fill(0);
      for (let i = 0; i < W; i++) re[i] = this.buf[i];
      this.fft(re, im);
      // inverse FFTs (forward FFT of the conjugate): the band-limited signal whose energy normalises the
      // filters, then each code's filter output
      yr.fill(0);
      yi.fill(0);
      for (let k = this.k0; k <= this.k1; k++) { yr[k] = 2 * re[k]; yi[k] = -2 * im[k]; }
      this.fft(yr, yi);
      for (let i = 0; i < W; i++) cum[i + 1] = cum[i] + (yr[i] * yr[i] + yi[i] * yi[i]) / (BN * BN);
      for (let p = 0; p < adv; p++) {
        const e = cum[p + BLEN] - cum[p];
        energy[p] += e;
        den[p] = Math.sqrt(e);
      }
      codes.forEach((code, ci) => {
        const { tRe, tIm, norm } = this.tmpl[code], [cr, cim] = this.out(code, si), sc = scores[ci];
        cr.fill(0);
        cim.fill(0);
        for (let k = this.k0, b = 0; k <= this.k1; k++, b++) {
          const xr = 2 * re[k], xi = 2 * im[k];
          cr[k] = xr * tRe[b] + xi * tIm[b];
          cim[k] = -(xi * tRe[b] - xr * tIm[b]);
        }
        this.fft(cr, cim);
        for (let p = 0; p < adv; p++) sc[p] += Math.hypot(cr[p], cim[p]) / BN / (den[p] * norm + 1e-9);
      });
    });
    this.done = model.total;
    codes.forEach((code, ci) => {
      const sc = scores[ci], peaks = [];
      for (let p = 0; p < adv; p++) {
        if (sc[p] / n >= BEACON_SCORE && !(p && sc[p] < sc[p - 1]) && !(p + 1 < adv && sc[p] < sc[p + 1])) peaks.push(p);
      }
      peaks.sort((a, b) => sc[b] - sc[a]);
      const kept = [];
      for (const p of peaks) {
        // the strongest copy within 50 ms wins (the rest are echoes); two phones with one code still show
        if (kept.length >= 3 || kept.some((q) => Math.abs(q - p) < 0.05 * FS)) continue;
        const at = (start + p) / FS;
        if (this.dets.some((d) => d.kind === code && Math.abs(d.at - at) < 0.05)) continue;
        kept.push(p);
      }
      if (!kept.length) return;
      const st = model.steering(act.map((a) => a[0]), CHIRP_HI, CHIRP_LO), outs = srcs.map((_, si) => this.out(code, si));
      for (const p of kept) {
        const curve = this.pulseSRP(outs, p, st);
        // the chirp's own energy (score² × window energy), so a sound overlapping it doesn't make it seem closer
        const level = 10 * Math.log10(((sc[p] / n) ** 2 * energy[p]) / n / BLEN / (32768 * 32768) + 1e-12);
        found.push({ t: model.now, at: (start + p) / FS, ...pick(curve, st), strength: 0, curve, n, kind: code, score: sc[p] / n, level });
      }
    });
    found.sort((a, b) => a.at - b.at);
    for (const d of found) {
      this.dets.push(d);
      model.add(d);
    }
    while (this.dets.length && model.now - this.dets[0].t > 12) this.dets.shift();
    return found;
  }

  // Direction from the matched-filter outputs: there the chirp is a pulse about 4 samples wide, so a short
  // window around it holds the chirp and very little of the fan. Each channel's window is upsampled 8x
  // and every mic pair cross-correlated at the lags the array can produce, then summed per angle.
  pulseSRP(outs, p, st) {
    const G = 64, U = G * INTERP, ups = [];
    if (!this.f64) { this.f64 = makeFFT(G); this.fU = makeFFT(U); }
    for (const [cr, ci] of outs) {
      const r = new Float64Array(G), i = new Float64Array(G);
      for (let n = 0; n < G; n++) {
        const q = p - G / 2 + n;
        if (q < 0) continue;
        const w = 0.5 - 0.5 * Math.cos((2 * Math.PI * (n + 0.5)) / G);
        r[n] = (cr[q] / BN) * w;
        i[n] = (-ci[q] / BN) * w;  // the inverse FFT left it conjugated
      }
      this.f64(r, i);
      const ur = new Float64Array(U), ui = new Float64Array(U);
      for (let k = 0; k < G / 2; k++) {
        ur[k] = r[k]; ui[k] = -i[k];
        ur[U - G / 2 + k] = r[G / 2 + k]; ui[U - G / 2 + k] = -i[G / 2 + k];
      }
      this.fU(ur, ui);  // conjugated input: an inverse transform, result conjugated again below
      for (let n = 0; n < U; n++) ui[n] = -ui[n];
      ups.push([ur, ui]);
    }
    const xcorr = (a, b, lag) => {  // b against a, b later by lag (upsampled samples)
      const [ar, ai] = ups[a], [br, bi] = ups[b];
      let s = 0;
      for (let q = Math.max(0, -lag); q < Math.min(U, U - lag); q++) s += br[q + lag] * ar[q] + bi[q + lag] * ai[q];
      return s;
    };
    const P = st.plates;
    let score = 0;
    if (P && P.front.length && P.back.length) {  // which plate's pulse comes first, and which is stronger
      let later = 0, earlier = 0;
      for (let lag = INTERP; lag <= PLATE_MAX_LAG * INTERP; lag += INTERP) {
        let sl = 0, se = 0;
        for (const i of P.front) for (const j of P.back) { sl += xcorr(i, j, lag); se += xcorr(i, j, -lag); }
        later = Math.max(later, sl);
        earlier = Math.max(earlier, se);
      }
      const energy = (ix) => ix.reduce((s, c) => s + 10 * Math.log10(ups[c][0].reduce((e, v, n) => e + v * v + ups[c][1][n] ** 2, 0) + 1e-30), 0) / ix.length;
      score = 0.5 * ((later - earlier) / (later + earlier + 1e-30)) + 0.5 * Math.tanh(-(energy(P.back) - energy(P.front)) / PLATE_SHADOW_DB);
    }
    const [wFront, wBack] = plateWeights(P, score);
    const grid = new Float64Array(360 * st.nel), cc = new Float64Array(st.L);
    for (const [a, b, idx, onFront] of st.pairs) {
      for (let l = 0; l < st.L; l++) cc[l] = xcorr(a, b, l - st.m);
      const w = onFront ? wFront : wBack;
      for (let g = 0; g < grid.length; g++) grid[g] += w * cc[idx[g]];
    }
    const curve = toAzimuth(grid, st);
    if (P) tilt(curve, score);
    return curve;
  }

}

// Hits of one kind from the last `win` seconds grouped by direction (two exits show as two): per group
// the circular mean of its last few hits, how loud it is, and whether it's getting louder (closer) than
// it was `older` seconds ago (from the last `avg` hits). Loudest first.
function tracks(dets, kind, win, older = [2, 5], avg = 4) {
  const now = model.now, groups = [];
  for (let i = dets.length - 1; i >= 0; i--) {
    const d = dets[i];
    if (d.kind !== kind || now - d.t > win) continue;
    let g = groups.find((o) => Math.abs(wrap(o.angle - d.angle)) < 25);
    if (!g) groups.push((g = { angle: d.angle, hits: [] }));
    g.hits.push(d);
  }
  const mean = (a) => a.reduce((s, d) => s + d.level, 0) / a.length;
  return groups.map((g) => {
    const lastFew = g.hits.slice(0, avg);
    let x = 0, y = 0;
    for (const d of lastFew) { x += Math.cos((d.angle * Math.PI) / 180); y += Math.sin((d.angle * Math.PI) / 180); }
    const angle = (Math.atan2(y, x) * 180) / Math.PI, level = mean(lastFew.slice(0, 3));
    const before = dets.filter((d) => d.kind === kind && now - d.t >= older[0] && now - d.t < older[1] && Math.abs(wrap(d.angle - angle)) < 25);
    const trend = before.length >= 2 && g.hits.length >= 2 ? level - mean(before) : 0;
    return { angle, level, trend, last: g.hits[0], hits: g.hits.length, age: now - g.hits[0].t };
  }).sort((a, b) => b.level - a.level);
}

// Alarm finder. Smoke and fire alarms are loud steady tones (almost all between 400 Hz and 4.5 kHz) that
// beep on and off. A frame is "tone" when one frequency stands far above the rest of the band, and a run of
// those at one unwavering pitch is a beep; a voice's pitch slides. Short beeps (CO alarms, 0.1 s) are high
// piezo tones, and low sounders (520 Hz) beep for half a second, so a beep under 1 kHz must last 0.25 s:
// that's longer than a voice holds one pitch. When a beep ends, its direction comes from its first ~150 ms,
// before the room's echoes build up, with GCC-PHAT on just the bins around the tone. A beep only counts with
// two more at the same pitch in the last 10 s (or once a tone has held for 1.5 s, located every half
// second), so talk, a whistle or a sung note doesn't.
const ALARM = { lo: 400, hi: 4500, tonal: 18, share: 0.2, hop: 256, gap: 2, confirm: 10, beeps: 3, steady: 1.5 };
const samePitch = (a, b) => Math.abs(a - b) <= Math.max(6, 0.012 * b);
const minBeep = (f) => (f < 1000 ? 0.25 : 0.08) * FS;

class Alarm {
  constructor() {
    this.k0 = Math.ceil((ALARM.lo * NFFT) / FS);
    this.k1 = Math.floor((ALARM.hi * NFFT) / FS);
    this.win = Float64Array.from({ length: SEG }, (_, i) => 0.5 - 0.5 * Math.cos((2 * Math.PI * (i + 0.5)) / SEG));
    this.buf = new Float32Array(SEG);
    this.dets = [];
    this.beeps = [];
    this.run = null;
    this.done = 0;
  }

  reset() { this.done = model.total; this.dets = []; this.beeps = []; this.run = null; }

  process() {
    const act = model.active(), found = [];
    if (act.length < 2 || model.total < FS) return found;
    const srcs = act.map((a) => a[1]), nb = this.k1 - this.k0 + 1;
    if (model.total - this.done > FS) { this.done = model.total - FS; this.run = null; }
    const re = new Float64Array(NFFT), im = new Float64Array(NFFT), P = new Float64Array(nb);
    while (this.done + SEG <= model.total) {
      const start = this.done;
      this.done += ALARM.hop;
      P.fill(0);
      for (const s of srcs) {
        model.at(s, start, SEG, this.buf);
        re.fill(0);
        im.fill(0);
        for (let i = 0; i < SEG; i++) re[i] = this.buf[i] * this.win[i];
        fft(re, im);
        for (let b = 0; b < nb; b++) P[b] += re[this.k0 + b] ** 2 + im[this.k0 + b] ** 2;
      }
      let kp = 0, total = 0, tone = 0;
      for (let b = 0; b < nb; b++) { total += P[b]; if (P[b] > P[kp]) kp = b; }
      for (let b = Math.max(0, kp - 3); b <= Math.min(nb - 1, kp + 3); b++) tone += P[b];
      const med = Float64Array.from(P).sort()[nb >> 1];
      let dk = 0;  // the peak between bins: a parabola through the log powers
      if (kp > 0 && kp < nb - 1) {
        const a = Math.log(P[kp - 1] + 1e-30), b = Math.log(P[kp] + 1e-30), c = Math.log(P[kp + 1] + 1e-30);
        dk = a - 2 * b + c < 0 ? (0.5 * (a - c)) / (a - 2 * b + c) : 0;
      }
      const f = ((this.k0 + kp + dk) * FS) / NFFT;
      // the tone's mean square per mic (Hann window, positive frequencies only), in dBFS
      const level = 10 * Math.log10((2 * tone) / srcs.length / (NFFT * SEG * 0.375) / (32768 * 32768) + 1e-12);
      const on = P[kp] >= med * 10 ** (ALARM.tonal / 10) && tone >= ALARM.share * total;
      const r = this.run;
      if (on && r && samePitch(f, r.f)) {
        r.miss = 0;
        r.last = start;
        r.f = 0.8 * r.f + 0.2 * f;
        r.lo = Math.min(r.lo, f);
        r.hi = Math.max(r.hi, f);
        r.level = Math.max(r.level, level);
        if (start - r.start >= ALARM.steady * FS && samePitch(r.hi, r.lo) && start - r.located >= FS / 2) {  // a horn that doesn't beep
          r.located = start;
          found.push(this.locate(act, start - 3 * 512 + 384, start, r.f, r.level));
        }
        continue;
      }
      if (r && ++r.miss <= ALARM.gap) continue;  // a word or a clap over the beep doesn't split it
      if (r && r.last - r.start + SEG / 2 >= minBeep(r.f) && samePitch(r.hi, r.lo)) {
        this.beeps.push({ f: r.f, dur: (r.last - r.start + SEG / 2) / FS, end: start });
        const recent = this.beeps.filter((b) => samePitch(b.f, r.f) && start - b.end < ALARM.confirm * FS).length;
        if (recent >= ALARM.beeps && r.located < 0 && model.total - r.start < model.N - SEG) {
          found.push(this.locate(act, r.start + 384, r.last, r.f, r.level));
        }
      }
      this.run = on ? { f, lo: f, hi: f, start, last: start, miss: 0, level, located: -Infinity } : null;
    }
    while (this.beeps.length && model.total - this.beeps[0].end > 20 * FS) this.beeps.shift();
    for (const d of found) {
      this.dets.push(d);
      model.add(d);
    }
    while (this.dets.length && model.now - this.dets[0].t > 20) this.dets.shift();
    return found;
  }

  // up to three frames 32 ms apart from `from`, none starting after `upto` (the last tone frame's start)
  locate(act, from, upto, f, level) {
    const fr = Math.round(f / 25) * 25;  // a handful of steering tables, not one per pitch
    const st = model.steering(act.map((a) => a[0]), fr + 30, fr - 30), srcs = act.map((a) => a[1]);
    const curve = new Float64Array(360);
    for (let k = 0, at = from; k < 3 && (k === 0 || at <= upto); k++, at += 512) {
      const c = model.srp(srcs, st, Math.min(at, upto));
      for (let i = 0; i < 360; i++) curve[i] += c[i];
      if (k === 0 && c.el !== undefined) curve.el = c.el;
    }
    const start = from;
    return { t: model.now, at: start / FS, ...pick(curve, st), strength: 0, curve, n: act.length, kind: "alarm", freq: f, level, score: 1 };
  }

  // how the alarm at about pitch f sounds: steady, or the length of its beeps
  pattern(f) {
    if (this.run && samePitch(this.run.f, f) && this.run.last - this.run.start >= ALARM.steady * FS) return "steady tone";
    const d = this.beeps.filter((b) => samePitch(b.f, f)).map((b) => b.dur);
    if (!d.length) return "beeping";
    const m = Math.max(...d);  // talk over a beep can split it, never lengthen it
    return m < 0.25 ? "short beeps (CO-alarm style)" : m < 0.8 ? "long beeps (fire-alarm style)" : `beeps ${m.toFixed(1)} s long`;
  }
}

// ------------------------------------------------------------------ packet parser (same as the USB protocol)

class Parser {
  constructor(on) {
    this.on = on;
    this.td = new TextDecoder();
    this.reset();
  }
  reset() {
    this.buf = new Uint8Array(0);
    this.esc = false;       // between the board's "escape: on" and "escape: off": FE 00 = 0xFE, FE 01 = 0xFF
    this.escHalf = false;   // the last read ended inside an FE pair
  }
  unescape(c) {
    if (!this.escHalf && c.indexOf(0xfe) < 0) return c;
    const out = new Uint8Array(c.length);
    let o = 0, i = 0;
    if (this.escHalf && c.length) { out[o++] = 0xfe + c[0]; i = 1; this.escHalf = false; }
    for (; i < c.length; i++) {
      if (c[i] !== 0xfe) out[o++] = c[i];
      else if (i + 1 < c.length) out[o++] = 0xfe + c[++i];
      else this.escHalf = true;
    }
    return out.subarray(0, o);
  }
  escape(c) {  // back to the bytes as they came, for the part of a read after "escape: off"
    const out = [];
    for (const v of c) { if (v >= 0xfe) out.push(0xfe, v - 0xfe); else out.push(v); }
    if (this.escHalf) out.push(0xfe);
    this.escHalf = false;
    return new Uint8Array(out);
  }
  feed(chunk) {
    const c = this.esc ? this.unescape(chunk) : chunk;
    let b = c;
    if (this.buf.length) {
      b = new Uint8Array(this.buf.length + c.length);
      b.set(this.buf);
      b.set(c, this.buf.length);
    }
    const n = b.length, dv = new DataView(b.buffer, b.byteOffset, b.byteLength);
    let p = 0;
    for (;;) {
      while (p + 1 < n && !(b[p] === 0xa5 && b[p + 1] === 0x5a)) p++;
      if (p + 4 > n) break;
      const k0 = b[p + 2], k1 = b[p + 3];
      if (k0 === 77 && k1 === 56) {  // 'M8'
        if (p + 10 > n) break;
        const seq = dv.getUint32(p + 4, true), frames = dv.getUint16(p + 8, true);
        if (frames === 0 || frames > 4096) { p += 2; continue; }
        const end = p + 10 + frames * CH * 2;
        if (end > n) break;
        const s = new Int16Array(b.buffer.slice(b.byteOffset + p + 10, b.byteOffset + end));  // little-endian host
        this.on.mic(seq, frames, s, 0xff);
        p = end;
      } else if (k0 === 77 && k1 === 75) {  // 'MK': only the channels in the mask (asked for with 'k')
        if (p + 12 > n) break;
        const seq = dv.getUint32(p + 4, true), frames = dv.getUint16(p + 8, true), mask = b[p + 10];
        const chans = [];
        for (let c = 0; c < CH; c++) if (mask & (1 << c)) chans.push(c);
        if (frames === 0 || frames > 4096 || !chans.length) { p += 2; continue; }
        const nc = chans.length, end = p + 12 + frames * nc * 2;
        if (end > n) break;
        const src = new Int16Array(b.buffer.slice(b.byteOffset + p + 12, b.byteOffset + end));
        const s = new Int16Array(frames * CH);
        for (let f = 0; f < frames; f++) for (let i = 0; i < nc; i++) s[f * CH + chans[i]] = src[f * nc + i];
        this.on.mic(seq, frames, s, mask);
        p = end;
      } else if (k0 === 77 && k1 === 52) {  // 'M4': the 4-mic firmware
        this.on.wrong();
        p += 2;
      } else if (k0 === 74 && k1 === 80) {  // 'JP'
        if (p + 16 > n) break;
        const len = dv.getUint32(p + 8, true), w = dv.getUint16(p + 12, true), h = dv.getUint16(p + 14, true);
        if (len > 400000 || !w || w > 2048 || !h || h > 2048) { p += 2; continue; }
        if (p + 16 + len > n) break;
        this.on.jpeg(b.slice(p + 16, p + 16 + len));
        p += 16 + len;
      } else if (k0 === 83 && k1 === 84) {  // 'ST'
        if (p + 6 > n) break;
        const len = dv.getUint16(p + 4, true);
        if (len > 512) { p += 2; continue; }
        if (p + 6 + len > n) break;
        const msg = this.td.decode(b.subarray(p + 6, p + 6 + len));
        p += 6 + len;
        if (msg === "escape: on" || msg === "escape: off") {
          const on = msg === "escape: on";
          if (on === this.esc) continue;
          // the rest of this read is in the other form: undo what was done to it and parse it again
          const rest = this.esc ? this.escape(b.subarray(p)) : b.slice(p);
          this.esc = on;
          this.buf = new Uint8Array(0);
          this.feed(rest);
          return;
        }
        this.on.status(msg);
      } else {
        p += 2;
      }
    }
    this.buf = p < n ? b.slice(p) : new Uint8Array(0);
    if (this.buf.length > 1 << 20) this.buf = new Uint8Array(0);
  }
}

// ------------------------------------------------------------------ connections

// ~4 s of stream: a busy moment on the page must not back up into the board, which then drops mic
// packets and camera frames
const SERIAL_BUFFER = 1 << 20;

class SerialConn {  // Web Serial (desktop Chrome / Edge, newer Android Chrome)
  constructor() { this.name = "USB serial"; }
  async open() {
    this.port = await navigator.serial.requestPort({ filters: VIDS.map((v) => ({ usbVendorId: v })) });
    await this.port.open({ baudRate: 115200, bufferSize: SERIAL_BUFFER });
  }
  async reopen() {
    for (;;) {
      await sleep(700);
      if (this.closed) return false;
      try {
        const ports = await navigator.serial.getPorts();
        const p = ports.find((x) => VIDS.includes(x.getInfo().usbVendorId)) || this.port;
        try { await p.close(); } catch (e) { /* not open */ }
        await p.open({ baudRate: 115200, bufferSize: SERIAL_BUFFER });
        this.port = p;
        return true;
      } catch (e) { /* board still resetting */ }
    }
  }
  async run(onData, onState) {
    while (!this.closed) {
      if (!this.port.readable) {
        onState("reconnecting…");
        if (!(await this.reopen())) return;
      }
      this.reader = this.port.readable.getReader();
      onState("streaming");
      try {
        for (;;) {
          const { value, done } = await this.reader.read();
          if (done) break;
          if (value) onData(value);
        }
      } catch (e) { /* unplugged or reset */ }
      try { this.reader.releaseLock(); } catch (e) { /* already released */ }
      if (this.closed) return;
      onState("reconnecting…");
      try { await this.port.close(); } catch (e) { /* gone */ }
      if (!(await this.reopen())) return;
    }
  }
  send(bytes) {  // one write at a time: a second getWriter() while one is locked throws, and that byte is lost
    this.sending = (this.sending || Promise.resolve()).then(async () => {
      if (!this.port?.writable) return;
      const w = this.port.writable.getWriter();
      try { await w.write(bytes); } finally { w.releaseLock(); }
    }).catch(() => {});
    return this.sending;
  }
  async close() {
    this.closed = true;
    try { await this.reader?.cancel(); } catch (e) { /* ignore */ }
    try { await this.port?.close(); } catch (e) { /* ignore */ }
  }
}

const APP_VERSION = "v24";
const MAX_USB_READ = 131072;  // about 1/3 s of stream; bigger only adds lag

class UsbConn {  // WebUSB, talking USB CDC directly (Android Chrome)
  constructor() { this.name = "USB"; }
  async open() {
    this.dev = await navigator.usb.requestDevice({ filters: VIDS.map((v) => ({ vendorId: v })) });
    await this.setup();
  }
  async setup() {
    const d = this.dev;
    if (!d.opened) await d.open();
    if (!d.configuration) await d.selectConfiguration(1);
    let data = null, comm = null;
    for (const itf of d.configuration.interfaces) {
      for (const alt of itf.alternates) {
        if (alt.interfaceClass === 10 && !data) {
          const i = alt.endpoints.find((e) => e.direction === "in" && e.type === "bulk");
          const o = alt.endpoints.find((e) => e.direction === "out" && e.type === "bulk");
          if (i && o) data = { n: itf.interfaceNumber, inEp: i.endpointNumber, outEp: o.endpointNumber };
        }
        if (alt.interfaceClass === 2 && comm === null) comm = itf.interfaceNumber;
      }
    }
    if (!data) throw new Error("this USB device has no serial interface");
    if (comm !== null) {
      try { await d.claimInterface(comm); } catch (e) { /* optional */ }
    }
    await d.claimInterface(data.n);
    this.data = data;
    if (comm !== null) {
      try {  // SET_LINE_CODING 115200 8N1; the ESP32-S3's USB serial ignores the rate but some hosts need it set
        await d.controlTransferOut({ requestType: "class", recipient: "interface", request: 0x20, value: 0, index: comm },
                                   new Uint8Array([0x00, 0xc2, 0x01, 0x00, 0x00, 0x00, 0x08]));
      } catch (e) { /* optional */ }
    }
  }
  async reopen() {
    for (;;) {
      await sleep(700);
      if (this.closed) return false;
      try {
        const devs = await navigator.usb.getDevices();
        const d = devs.find((x) => VIDS.includes(x.vendorId));
        if (!d) continue;
        this.dev = d;
        try { await d.close(); } catch (e) { /* not open */ }
        await this.setup();
        return true;
      } catch (e) { /* not back yet */ }
    }
  }
  async run(onData, onState) {
    while (!this.closed) {
      onState("streaming");
      const ep = this.data.inEp, q = [];
      // Android Chrome services reads one after another with a fixed cost each, so the link only keeps up
      // when each read is big: grow the read size while reads keep coming back full
      this.size = this.size || 65536;
      const read = () => { const p = this.dev.transferIn(ep, this.size); p.catch(() => {}); return p; };
      try {
        for (let i = 0; i < 16; i++) q.push(read());
        let full = 0;
        for (;;) {
          const r = await q.shift();
          if (this.closed) return;
          q.push(read());
          usbReads++;
          if (r.status === "ok" && r.data && r.data.byteLength) {
            full = r.data.byteLength >= this.size ? full + 1 : 0;
            if (full >= 8 && this.size < MAX_USB_READ) { this.size *= 2; full = 0; }
            onData(new Uint8Array(r.data.buffer, r.data.byteOffset, r.data.byteLength));
          } else if (r.status === "stall") {
            await this.dev.clearHalt("in", ep);
          }
        }
      } catch (e) {
        if (this.closed) return;
        if (this.dev.opened && this.size > 16384) {  // this device won't take reads that big
          this.size = 16384;
          continue;
        }
        onState("reconnecting…");
        if (!(await this.reopen())) return;
      }
    }
  }
  async send(bytes) {
    try { await this.dev.transferOut(this.data.outEp, bytes); } catch (e) { /* ignore */ }
  }
  async close() {
    this.closed = true;
    try { await this.dev.close(); } catch (e) { /* ignore */ }
  }
}

class WsConn {  // tools/ws_bridge.py, or a Wi-Fi firmware later
  constructor(url) { this.url = url; this.name = "Wi-Fi"; }
  async open() {}
  run(onData, onState) {
    return new Promise((resolve) => {
      const connect = () => {
        if (this.closed) return resolve();
        onState("connecting…");
        const ws = new WebSocket(this.url);
        ws.binaryType = "arraybuffer";
        ws.onopen = () => onState("streaming");
        ws.onmessage = (e) => onData(new Uint8Array(e.data));
        ws.onclose = () => { this.ws = null; if (!this.closed) setTimeout(connect, 1000); else resolve(); };
        this.ws = ws;
      };
      connect();
    });
  }
  async send(bytes) { if (this.ws && this.ws.readyState === 1) this.ws.send(bytes); }
  async close() { this.closed = true; this.ws?.close(); }
}

class DemoConn {  // no board: recorded speech and claps from all round the array over a fan behind, through the current layout
  constructor() { this.name = "demo"; }
  async open() {
    this.clips = {};
    for (const k of ["a1", "b1", "a2"]) {
      try { this.clips[k] = wavToFloat(await (await fetch(`demo/tts_${k}.wav`)).arrayBuffer()); } catch (e) { /* claps only */ }
    }
    this.script = [["clap", 0], ["say", "a1", 35], ["clap", 120], ["say", "b1", -110], ["clap", -45], ["say", "a2", 0], ["clap", 180]];
  }
  clap(ang) {
    const sig = new Float64Array(2048);
    for (let i = 0; i < sig.length; i++) sig[i] = gauss() * Math.exp(-i / 250) * Math.min(1, i / 40);
    return spatial(sig, ang, false, 12000, "peak");
  }
  next() {
    const ev = this.script[this.k++ % this.script.length];
    // talkers' mouths are about 20° above the chest at conversation distance
    if (ev[0] === "say" && this.clips[ev[1]]) return { sig: spatial(this.clips[ev[1]], ev[2], false, 2500, "rms", 20), gap: 0.9 * FS };
    return { sig: this.clap(ev[ev.length - 1]), gap: 1.0 * FS };
  }
  run(onData, onState) {
    onState("demo (no board)");
    this.k = 0;
    const white = Float64Array.from({ length: 65536 }, gauss);
    const fan = spatial(pinkish(white), 150, true, 500, "rms");
    let seq = 0, made = 0, ev = null, pos = 0, wait = FS;
    const t0 = performance.now();
    const cam = typeof OffscreenCanvas !== "undefined" ? new OffscreenCanvas(320, 240) : null;
    let camSeq = 0, camBusy = false, nextCam = 0;
    const finds = [], nextAt = {}, count = {};  // Find mode's beacons and alarm, playing
    return new Promise((resolve) => {
      this.timer = setInterval(() => {
        if (this.closed) { clearInterval(this.timer); return resolve(); }
        if (cam && !S.camOff && !camBusy && performance.now() >= nextCam) {  // 10 fps JPEG, like the board's camera
          nextCam = performance.now() + 100;
          camBusy = true;
          demoPicture(cam.getContext("2d"), made / FS);
          cam.convertToBlob({ type: "image/jpeg", quality: 0.6 })
            .then((blob) => blob.arrayBuffer())
            .then((ab) => {
              const j = new Uint8Array(ab), pkt = new Uint8Array(16 + j.length), dv = new DataView(pkt.buffer);
              pkt.set([0xa5, 0x5a, 74, 80]);
              dv.setUint32(4, camSeq++, true);
              dv.setUint32(8, j.length, true);
              dv.setUint16(12, 320, true);
              dv.setUint16(14, 240, true);
              pkt.set(j, 16);
              if (!this.closed) onData(pkt);
            })
            .finally(() => { camBusy = false; });
        }
        const due = Math.min(((performance.now() - t0) / 1000) * FS - made, 128 * 20);
        for (let done = 0; done + 128 <= due; done += 128) {
          const frames = 128, pkt = new Uint8Array(10 + frames * CH * 2), dv = new DataView(pkt.buffer);
          pkt.set([0xa5, 0x5a, 77, 56]);
          dv.setUint32(4, seq++, true);
          dv.setUint16(8, frames, true);
          for (let f = 0; f < frames; f++) {
            if (!ev && --wait <= 0) { ev = this.next(); pos = 0; }
            const n = made + f, fi = n & 65535;
            if (S.find) {
              for (const e of DEMO_FIND[S.findMode]) {
                nextAt[e.id] ??= n + (e.phase || 0) * FS;
                if (n < nextAt[e.id]) continue;
                const t = n / FS, k = (count[e.id] = (count[e.id] || 0) + 1);
                finds.push({ sig: spatial(e.sig(), e.ang(t), false, e.level(t), "peak"), pos: 0 });
                nextAt[e.id] = n + (e.gaps ? e.gaps[k % e.gaps.length] : e.period) * FS;
              }
            } else if (cal.on && model.now >= cal.since && n >= (nextAt.calib ?? 0)) {  // the phone on the calibration spot
              const [a, e] = demoPhoneDir();
              finds.push({ sig: spatial(codeChirp("calib"), a, false, 900 / S.calDist, "peak", e), pos: 0 });
              nextAt.calib = n + CODES.calib.period * FS;
            }
            for (let c = 0; c < CH; c++) {
              let v = -30935;
              if (c in model.geom) {
                v = gauss() * 25 + (fan[c] ? fan[c][fi] : 0);
                if (ev && ev.sig[c]) v += ev.sig[c][pos];
                for (const b of finds) if (b.sig[c]) v += b.sig[c][b.pos];
              }
              dv.setInt16(10 + (f * CH + c) * 2, Math.max(-32768, Math.min(32767, Math.round(v))), true);
            }
            if (ev && ++pos >= ev.sig.len) { wait = ev.gap; ev = null; }
            for (let i = finds.length - 1; i >= 0; i--) if (++finds[i].pos >= finds[i].sig.len) finds.splice(i, 1);
          }
          made += frames;
          onData(pkt);
        }
      }, 8);
    });
  }
  async send() {}
  async close() { this.closed = true; }
}

// Demo sources for each Find function, on top of the talkers and claps. The wearer turns slowly, so
// everything drifts round the array; the leader also walks nearer and farther.
function alarmBeep() {
  const n = FS / 2, fade = 0.005 * FS;
  return Float64Array.from({ length: n }, (_, i) => Math.sin((2 * Math.PI * 3100 * i) / FS) * Math.min(1, i / fade, (n - 1 - i) / fade));
}
const DEMO_FIND = {
  follow: [{ id: "leader", sig: () => codeChirp("leader"), period: CODES.leader.period, ang: (t) => wrap(t * 9 - 60), level: (t) => 700 + 600 * Math.sin(t * 0.25) }],
  evac: [
    { id: "exit", sig: () => codeChirp("exit"), period: CODES.exit.period, ang: (t) => wrap(40 - t * 4), level: () => 900 },
    { id: "exit2", sig: () => codeChirp("exit"), period: CODES.exit.period, phase: 0.23, ang: (t) => wrap(-150 - t * 4), level: () => 450 },
    { id: "danger", sig: () => codeChirp("danger"), period: CODES.danger.period, phase: 0.11, ang: (t) => wrap(-35 - t * 4), level: () => 1200 },
  ],
  // a fire alarm's three 0.5 s beeps, then a pause (the "temporal-3" pattern)
  alarm: [{ id: "alarm", sig: alarmBeep, gaps: [1, 1, 2], ang: (t) => wrap(70 - t * 4), level: () => 1500 }],
};

// The demo's made-up camera (aimed a little right and up, rolled, below the mics), which calibration should
// find, and the phone on the spot calibration asks for.
const DEMO_CAM = { offset: -6, tilt: 12, roll: 3, hfov: 68, kel: 1, camX: 0, camZ: -40 };
function demoPhoneDir() {
  const [, , X, Y] = calSpot();
  return camUnproj(X, Y, S.calDist, DEMO_CAM);
}

function demoPicture(g, t) {
  const sky = g.createLinearGradient(0, 0, 0, 240);
  sky.addColorStop(0, "#2d3b4e");
  sky.addColorStop(1, "#141a22");
  g.fillStyle = sky;
  g.fillRect(0, 0, 320, 240);
  g.strokeStyle = "#3a4656";
  for (let i = -6; i <= 6; i++) {
    g.beginPath();
    g.moveTo(160 + i * 12, 150);
    g.lineTo(160 + i * 60, 240);
    g.stroke();
  }
  const x = 160 + 120 * Math.sin(t * 0.8);
  g.fillStyle = "#e3b341";
  g.fillRect(x - 6, 70, 12, 80);
  g.fillStyle = "#e6edf3";
  g.font = "14px sans-serif";
  g.fillText(`demo camera  ${t.toFixed(1)} s`, 10, 20);
  if (cal.on && model.now >= cal.since) {  // not while it's being moved to the next spot
    const [fx, fy] = calSpot(), px = fx * 320, py = fy * 240, s = Math.max(0.5, 1.6 / S.calDist);
    g.fillStyle = "#0b0f14";
    g.strokeStyle = "#e6edf3";
    g.lineWidth = 1.5;
    g.fillRect(px - 7 * s, py - 24 * s, 14 * s, 28 * s);
    g.strokeRect(px - 7 * s, py - 24 * s, 14 * s, 28 * s);
    g.fillStyle = "#56d4dd";
    g.beginPath();
    g.arc(px, py, 2 * s, 0, 2 * Math.PI);  // the speaker, at the bottom
    g.fill();
  }
}

function gauss() {
  return Math.sqrt(-2 * Math.log(Math.random() + 1e-12)) * Math.cos(2 * Math.PI * Math.random());
}

const ffts = new Map();
const fftOf = (n) => ffts.get(n) || (ffts.set(n, makeFFT(n)), ffts.get(n));

// Mono signal arriving from angle ang (and elevation el) at every mic in the current layout (fractional
// delays in the frequency domain), scaled so its peak or active-speech RMS is `level`.
function spatial(sig, ang, circular, level, by, el = 0) {
  const pad = circular ? 0 : 32, n = 1 << Math.ceil(Math.log2(sig.length + 2 * pad)), f = fftOf(n);
  const re = new Float64Array(n), im = new Float64Array(n);
  re.set(sig, pad);
  let ref = 0, cnt = 0;
  for (const v of sig) {
    if (by === "peak") ref = Math.max(ref, Math.abs(v));
    else if (Math.abs(v) > 0.01) { ref += v * v; cnt++; }
  }
  ref = by === "peak" ? ref : Math.sqrt(ref / Math.max(1, cnt));
  const gain = level / Math.max(ref, 1e-9) / n;
  f(re, im);
  const th = (ang * Math.PI) / 180, ph = (el * Math.PI) / 180, out = { len: circular ? n : sig.length + pad };
  const ux = Math.sin(th) * Math.cos(ph), uy = Math.cos(th) * Math.cos(ph), uz = Math.sin(ph);
  for (const [ch, [, x, y, z = 0]] of Object.entries(model.geom)) {
    const delay = -((x / 1000) * ux + (y / 1000) * uy + (z / 1000) * uz) / C, r = new Float64Array(n), i = new Float64Array(n);
    for (let k = 0; k < n; k++) {
      const ph = -2 * Math.PI * ((k <= n / 2 ? k : k - n) * (FS / n)) * delay, c = Math.cos(ph), s = Math.sin(ph);
      r[k] = re[k] * c - im[k] * s;
      i[k] = -(re[k] * s + im[k] * c);  // conjugate, so the forward FFT acts as an inverse
    }
    f(r, i);
    for (let k = 0; k < n; k++) r[k] *= gain;
    if (!circular) r.fill(0, 0, pad - 8);  // fractional-delay ringing ahead of the onset
    out[ch] = r;
  }
  return out;
}

// fan-like rush: white noise tilted toward low frequencies
function pinkish(white) {
  const n = white.length, f = fftOf(n), re = Float64Array.from(white), im = new Float64Array(n);
  f(re, im);
  for (let k = 0; k < n; k++) {
    const hz = Math.max(60, Math.abs(k <= n / 2 ? k : k - n) * (FS / n)), g = Math.sqrt(60 / hz) * (hz > 6000 ? 0.3 : 1);
    re[k] *= g;
    im[k] = -im[k] * g;
  }
  f(re, im);
  return re.map((v) => v / n);
}

function wavToFloat(buf) {
  const dv = new DataView(buf);
  let p = 12;
  while (p + 8 <= dv.byteLength) {
    const id = String.fromCharCode(dv.getUint8(p), dv.getUint8(p + 1), dv.getUint8(p + 2), dv.getUint8(p + 3));
    const len = dv.getUint32(p + 4, true);
    if (id === "data") {
      const out = new Float64Array(len >> 1);
      for (let i = 0; i < out.length; i++) out[i] = dv.getInt16(p + 8 + 2 * i, true) / 32768;
      return out;
    }
    p += 8 + len + (len & 1);
  }
  throw new Error("no audio in wav");
}

// ------------------------------------------------------------------ app state

const model = new Model();
const beacon = new Beacon(), alarm = new Alarm();
let conn = null, connState = "", wrongFirmware = false, boardStats = "", boardMsg = "", boardFw = "", fwLayoutDone = false;
let image = null, decoding = 0, lastSeq = null, gaps = 0, decodeErrors = 0, decodeFails = 0, usbBytes = 0, usbReads = 0, pendingJpeg = null, streamMask = 0xff, maskAsks = 0;
const pktTimes = [], frameTimes = [], rxTimes = [];
const busy = { ms: 0 };  // main-thread time spent on our own work, for the load figure in the status line

const parser = new Parser({
  mic(seq, frames, s, mask) {
    streamMask = mask;
    if (lastSeq !== null && seq !== ((lastSeq + 1) >>> 0)) {
      gaps++;
      model.gapAt = model.total;
    }
    lastSeq = seq;
    model.push(s, frames);
    if (cap.on) runFocus(s, frames);
    pktTimes.push(performance.now());
  },
  wrong() { wrongFirmware = true; },
  jpeg(bytes) {
    const now = performance.now();
    rxTimes.push(now);
    if (S.camOff) return;  // a board without 'c' keeps sending: don't spend the tablet's time decoding
    // frames can arrive several at once; keep the newest and show it when the current decode is done
    if (decoding && now - decoding < 1000) { pendingJpeg = bytes; return; }  // a hung decode mustn't freeze it
    pendingJpeg = null;
    const mine = decoding = now;
    const blob = new Blob([bytes], { type: "image/jpeg" });
    createImageBitmap(blob)
      .catch(() => {  // some JPEGs the bitmap decoder rejects still decode as an <img>
        decodeErrors++;
        const img = new Image(), url = URL.createObjectURL(blob);
        img.src = url;
        return img.decode().then(() => img).finally(() => URL.revokeObjectURL(url));
      })
      .then((bm) => { image?.close?.(); image = bm; frameTimes.push(performance.now()); })
      .catch(() => { decodeFails++; })
      .finally(() => {
        if (decoding !== mine) return;
        decoding = 0;
        if (pendingJpeg) { rxTimes.pop(); parser.on.jpeg(pendingJpeg); }
      });
  },
  status(msg) {
    if (msg.startsWith("stats: ")) boardStats = msg.slice(7);
    else if (msg.startsWith("firmware: ")) {
      boardFw = msg.slice(10);
      // the board says which mics it has: switch to that layout once per connection, so it can still be changed
      const lay = /layout (\w+)/.exec(boardFw)?.[1];
      if (lay && PRESETS[lay] && !fwLayoutDone) {
        fwLayoutDone = true;
        if (S.layout !== lay) {
          S.layout = lay;
          model.setGeom(PRESETS[lay].geom);
          save();
          uiSync();
          hint(`Layout set from the board: ${PRESETS[lay].name}.`);
        }
      }
    } else boardMsg = msg;
  },
});

const orient = (a) => {
  const b = S.swap ? -a : a;
  return S.swapfb ? wrap(180 - b) : wrap(b);
};
const sideText = (rel) => Math.abs(rel) < 2 ? "straight ahead" : Math.abs(rel) > 178 ? "directly behind"
  : `${Math.abs(rel).toFixed(0)}° ${rel > 0 ? "left" : "right"}`;
const fbText = (d) => d.fb === null ? "front/back unknown (no back mic)" : d.fb >= FB_SURE ? "front/back sure" : "front/back unsure";
// height from the raised mics, relative to the camera's centre (Set centre / Calibrate); null for a flat array
const upText = (d) => {
  if (d.elev === null || d.elev === undefined) return null;
  const e = d.elev * S.kel - S.tilt;
  return Math.abs(e) < 5 ? "level" : `${Math.abs(e).toFixed(0)}° ${e > 0 ? "up" : "down"}`;
};

// ------------------------------------------------------------------ sound → picture, and calibrating it

// Where a sound lands in the picture. The array's direction (azimuth a, + = left; elevation e, scaled by kel,
// or the camera's own tilt when the array is flat and e is null) to a point `dist` m away; seen from the
// camera, which sits camX / camZ mm from the array's centre (+ = left / up) and is turned by yaw (offset),
// pitch (tilt) and roll; through a lens hfov° across. [X, Y] in half picture widths from the centre (X right,
// Y up), or null when it's behind the camera.
const CAM_KEYS = ["offset", "tilt", "roll", "hfov", "kel", "camX", "camZ"];
function camProj(a, e, dist, P = S) {
  const r = Math.PI / 180, el = (e === null || e === undefined ? P.tilt : e * P.kel) * r, az = a * r, d = dist * 1000;
  const x = d * Math.sin(az) * Math.cos(el) - P.camX, y = d * Math.cos(az) * Math.cos(el), z = d * Math.sin(el) - P.camZ;
  const cy = Math.cos(P.offset * r), sy = Math.sin(P.offset * r), ct = Math.cos(P.tilt * r), st = Math.sin(P.tilt * r);
  const x1 = x * cy - y * sy, y1 = x * sy + y * cy;
  const y2 = y1 * ct + z * st, z2 = z * ct - y1 * st;
  if (y2 < 0.05 * Math.hypot(x1, y2, z2)) return null;
  const fn = 1 / Math.tan((P.hfov / 2) * r), X = (-x1 / y2) * fn, Y = (z2 / y2) * fn;
  const cr = Math.cos(P.roll * r), sr = Math.sin(P.roll * r);
  return [X * cr - Y * sr, X * sr + Y * cr];
}

// The other way: the array direction [a, e] (e as the array would measure it) of the point `dist` m away
// that shows at picture spot [X, Y]. For the demo's phone.
function camUnproj(X, Y, dist, P) {
  const r = Math.PI / 180, fn = 1 / Math.tan((P.hfov / 2) * r), cr = Math.cos(P.roll * r), sr = Math.sin(P.roll * r);
  const X0 = X * cr + Y * sr, Y0 = Y * cr - X * sr;
  const x1 = -X0 / fn, y2 = 1, z2 = Y0 / fn;
  const cy = Math.cos(P.offset * r), sy = Math.sin(P.offset * r), ct = Math.cos(P.tilt * r), st = Math.sin(P.tilt * r);
  const y1 = y2 * ct - z2 * st, z = y2 * st + z2 * ct, x = x1 * cy + y1 * sy, y = y1 * cy - x1 * sy;
  const uu = x * x + y * y + z * z, cu = P.camX * x + P.camZ * z, cc = P.camX ** 2 + P.camZ ** 2, D = dist * 1000;
  const s = (-cu + Math.sqrt(Math.max(0, cu * cu - uu * (cc - D * D)))) / uu;
  const px = P.camX + s * x, py = s * y, pz = P.camZ + s * z;
  return [Math.atan2(px, py) / r, Math.atan2(pz, Math.hypot(px, py)) / r / P.kel];
}

// What's left of each captured point's error after the fit, blended in near that point (fine-tune), so the
// spots that were measured land exactly and the fit's shape holds in between.
const FINE_DEG = 8;
function calFix(a, e) {
  if (!S.calFine || S.cal.length < 3) return [0, 0];
  let wx = 0, wy = 0, ws = 0;
  for (const p of S.cal) {
    if (!p.res) continue;
    const da = wrap(a - p.oa), de = e === null || e === undefined || p.e === null ? 0 : e - p.e;
    const w = Math.exp(-(da * da + de * de) / (2 * FINE_DEG * FINE_DEG));
    wx += w * p.res[0];
    wy += w * p.res[1];
    ws += w;
  }
  return [wx / Math.max(1, ws), wy / Math.max(1, ws)];
}

// canvas point of a sound from (oriented) azimuth a and elevation e, or null when it's behind the camera
function toScreen(pic, a, e, dist = cal.on ? S.calDist : S.dist) {
  const q = camProj(a, e, dist);
  if (!q) return null;
  const f = calFix(a, e);
  return [pic.cx + ((q[0] + f[0]) * pic.w) / 2, pic.cy - ((q[1] + f[1]) * pic.w) / 2];
}

// Calibration: a phone plays the Calibration chirp (beacon.html?role=calib) from a known spot in the picture,
// at a known distance. Each capture is the median of the last few chirps' directions; the fit finds the
// camera's yaw, pitch, roll, view width, the elevation scale and (with more than one distance) where the
// camera sits on the array, by least squares (Levenberg–Marquardt), with soft priors keeping whatever the
// points can't pin down near sensible values.
const CAL_TARGETS = [["Centre", 0.5, 0.5], ["Right edge", 0.92, 0.5], ["Left edge", 0.08, 0.5], ["Top", 0.5, 0.1],
  ["Bottom", 0.5, 0.9], ["Top right", 0.9, 0.12], ["Top left", 0.1, 0.12], ["Bottom right", 0.9, 0.88], ["Bottom left", 0.1, 0.88]];
const CAL_PRIOR = { offset: [0, 90], tilt: [0, 90], roll: [0, 4], hfov: [60, 40], kel: [1, 0.25], camX: [0, 60], camZ: [0, 60] };
const CAL_STEP = { offset: 0.05, tilt: 0.05, roll: 0.05, hfov: 0.05, kel: 0.001, camX: 0.5, camZ: 0.5 };
const CAL_SIGMA = 0.02;  // a capture's usual scatter, in half picture widths (about 1°)
const CAL_HITS_S = 3;
// The guide: spots to visit in order, [CAL_TARGETS index, metres]. The second distance is what pins down
// where the camera sits on the array.
const CAL_GUIDE = [[0, 1], [1, 1], [2, 1], [3, 1], [4, 1], [0, 2], [1, 2], [2, 2]];
const CAL_MOVE_S = 2.5;   // after a capture, chirps are ignored this long while the phone is moved
const CAL_AUTO_N = 6;     // chirps in a row (about 2.4 s) that must agree before the guide captures a spot
const CAL_STEADY = 1.5;   // ... to within this many degrees (median distance from their median)
const CAL_OFF = 0.3;      // once the fit has 3 points: how far (half picture widths, about 10°) the sound may land from the cross
const CAL_FORCE_S = 6;    // steady but off the cross this long: captured anyway, since the fit may be what's off
const cal = { on: false, target: 0, custom: null, since: 0, pic: null, last: null,
              guide: true, step: 0, lastDir: null, offSince: null, status: "" };
const calRecent = () => model.dets.filter((d) => d.kind === "calib" && d.t > cal.since && model.now - d.t < CAL_HITS_S);
// the spot the phone should be on: picture fractions, and half picture widths from the centre (X right, Y up)
function calSpot() {
  const [fx, fy] = cal.custom || CAL_TARGETS[cal.target].slice(1), aspect = cal.pic ? cal.pic.h / cal.pic.w : 0.75;
  return [fx, fy, (fx - 0.5) * 2, (0.5 - fy) * 2 * aspect];
}

const calAngle = (p, swap) => {
  const b = swap ? -p.ang : p.ang;
  return S.swapfb ? wrap(180 - b) : wrap(b);
};
function calResid(P, pts, swap, priors = true) {
  const out = [];
  for (const p of pts) {
    const q = camProj(calAngle(p, swap), p.e, p.dist, P);
    if (!q) { out.push(50, 50); continue; }
    out.push((q[0] - p.X) / CAL_SIGMA);
    if (p.e !== null) out.push((q[1] - p.Y) / CAL_SIGMA);
  }
  if (priors) for (const k of CAM_KEYS) out.push((P[k] - CAL_PRIOR[k][0]) / CAL_PRIOR[k][1]);
  return out;
}
const sumSq = (v) => v.reduce((s, x) => s + x * x, 0);
function calClamp(P) {
  P.offset = wrap(P.offset);
  P.tilt = Math.max(-70, Math.min(70, P.tilt));
  P.roll = Math.max(-30, Math.min(30, P.roll));
  P.hfov = Math.max(20, Math.min(150, P.hfov));
  P.kel = Math.max(0.3, Math.min(2.5, P.kel));
  P.camX = Math.max(-300, Math.min(300, P.camX));
  P.camZ = Math.max(-300, Math.min(300, P.camZ));
  return P;
}
function solveLin(A, b) {  // Gaussian elimination with partial pivoting; A is n×n, both are overwritten
  const n = b.length;
  for (let i = 0; i < n; i++) {
    let p = i;
    for (let j = i + 1; j < n; j++) if (Math.abs(A[j][i]) > Math.abs(A[p][i])) p = j;
    [A[i], A[p]] = [A[p], A[i]];
    [b[i], b[p]] = [b[p], b[i]];
    const d = A[i][i] || 1e-12;
    for (let j = i + 1; j < n; j++) {
      const f = A[j][i] / d;
      for (let k = i; k < n; k++) A[j][k] -= f * A[i][k];
      b[j] -= f * b[i];
    }
  }
  const x = new Array(n).fill(0);
  for (let i = n - 1; i >= 0; i--) {
    let s = b[i];
    for (let k = i + 1; k < n; k++) s -= A[i][k] * x[k];
    x[i] = s / (A[i][i] || 1e-12);
  }
  return x;
}
function calLM(P0, pts, swap) {
  let P = calClamp({ ...P0 }), r = calResid(P, pts, swap), cost = sumSq(r), lam = 1e-3;
  const n = CAM_KEYS.length;
  for (let it = 0; it < 80; it++) {
    const J = CAM_KEYS.map((k) => {
      const r2 = calResid({ ...P, [k]: P[k] + CAL_STEP[k] }, pts, swap);
      return r2.map((v, i) => (v - r[i]) / CAL_STEP[k]);
    });
    const A = J.map((a) => J.map((b) => a.reduce((s, v, m) => s + v * b[m], 0)));
    const g = J.map((a) => a.reduce((s, v, m) => s + v * r[m], 0));
    let better = false;
    for (let tries = 0; tries < 10 && !better; tries++) {
      const M = A.map((row, i) => row.map((v, j) => (i === j ? v * (1 + lam) + 1e-9 : v)));
      const dx = solveLin(M, g.map((v) => -v));
      const Q = { ...P };
      CAM_KEYS.forEach((k, i) => { Q[k] += dx[i]; });
      calClamp(Q);
      const r2 = calResid(Q, pts, swap), c2 = sumSq(r2);
      if (c2 < cost) {
        better = cost - c2 > 1e-7 * cost;
        P = Q; r = r2; cost = c2;
        lam = Math.max(1e-9, lam / 4);
        if (!better) break;
      } else lam *= 5;
    }
    if (!better) break;
  }
  return { P, cost, data: sumSq(calResid(P, pts, swap, false)) };
}
// One fit from where things are now and one from the point nearest the picture's centre; the better wins.
function calSolve(pts, swap) {
  const now = Object.fromEntries(CAM_KEYS.map((k) => [k, S[k]]));
  const mid = pts.reduce((b, p) => (Math.hypot(p.X, p.Y) < Math.hypot(b.X, b.Y) ? p : b));
  const fresh = { offset: calAngle(mid, swap), tilt: mid.e ?? 0, roll: 0, hfov: S.hfov, kel: 1, camX: 0, camZ: 0 };
  return [calLM(now, pts, swap), calLM(fresh, pts, swap)].reduce((a, b) => (b.cost < a.cost ? b : a));
}
// Fit to every captured point and use it. Also notices left and right being the wrong way round.
function calFit() {
  const pts = S.cal;
  if (!pts.length) return null;
  let best = calSolve(pts, S.swap), flipped = false;
  if (pts.length >= 3 && new Set(pts.map((p) => Math.round(p.X * 4))).size >= 2) {
    const other = calSolve(pts, !S.swap);
    if (other.data < 0.3 * best.data && best.data / pts.length > 9) { best = other; S.swap = !S.swap; flipped = true; }
  }
  Object.assign(S, best.P);
  const fn = 1 / Math.tan(((S.hfov / 2) * Math.PI) / 180);
  let sq = 0;
  for (const p of pts) {
    p.oa = calAngle(p, S.swap);
    const q = camProj(p.oa, p.e, p.dist) || [0, 0];
    p.res = [p.X - q[0], p.e === null ? 0 : p.Y - q[1]];
    p.err = (Math.atan(Math.hypot(...p.res) / fn) * 180) / Math.PI;
    sq += p.err * p.err;
  }
  save();
  return { rms: Math.sqrt(sq / pts.length), flipped };
}

// ------------------------------------------------------------------ voice focus (StreamingFocus from mic_caption_view.py)

// MVDR beam toward the talker + direction filter + Wiener hiss filter on 8 ms frames. It learns the
// background from quiet frames and the talker from voiced speech frames as it goes, so it follows whoever
// is talking. The same as tools/vest6_process.py (detect, DirectionFilter, Versions).
const FN = 512, FH = 128, FB = FN / 2 + 1, FK0 = 10, FK1 = 108, FLO = 3;  // band 300–3400 Hz; bins under 70 Hz dropped
const fft512 = makeFFT(FN);
const FWIN = Float64Array.from({ length: FN }, (_, i) => Math.sqrt(0.5 - 0.5 * Math.cos((2 * Math.PI * i) / FN)));
// What the beam learns from: no bangs or taps (a jump in every band within a frame, or one mic far louder
// than the rest), only voiced speech (a pitch in the 300–2000 Hz harmonics, two frames running, plus 150 ms
// for consonants), and, once the talker stands out from the background, only frames from their direction.
const ONSET_DB = 12, SPREAD_DB = 12, HOLD = 15, HOLD_MAX = 40, VOICED = 0.6, HANG = 19, ON_TARGET = 0.3;
const POST_FLOOR = 0.18, TOTAL_FLOOR = 0.1;  // direction filter down to -15 dB; with the noise filter, -20 dB
const FW0 = 5, FHI = 64, FV0 = 10, FV1 = 64, LAG0 = 40, LAG1 = 200;  // 150 Hz, 2 kHz, 300–2000 Hz, 400–80 Hz pitch
const FRW = (() => {  // the window's own correlation, divided out of the pitch correlation
  const r = new Float64Array(LAG1);
  let r0 = 0;
  for (let i = 0; i < FN; i++) r0 += FWIN[i] * FWIN[i];
  for (let l = LAG0; l < LAG1; l++) {
    let s = 0;
    for (let i = 0; i + l < FN; i++) s += FWIN[i] * FWIN[i + l];
    r[l] = s / r0;
  }
  return r;
})();

function cSolve(Are, Aim, n, Bre, Bim, m) {  // complex A (n×n) X = B (n×m), row-major; Gaussian elimination
  const ar = Float64Array.from(Are), ai = Float64Array.from(Aim), br = Float64Array.from(Bre), bi = Float64Array.from(Bim);
  const swap = (a, w, r1, r2) => { for (let j = 0; j < w; j++) { const t = a[r1 * w + j]; a[r1 * w + j] = a[r2 * w + j]; a[r2 * w + j] = t; } };
  for (let c = 0; c < n; c++) {
    let p = c, best = -1;
    for (let r = c; r < n; r++) {
      const v = ar[r * n + c] ** 2 + ai[r * n + c] ** 2;
      if (v > best) { best = v; p = r; }
    }
    if (p !== c) { swap(ar, n, p, c); swap(ai, n, p, c); swap(br, m, p, c); swap(bi, m, p, c); }
    const pr = ar[c * n + c], pi = ai[c * n + c], den = pr * pr + pi * pi || 1e-30, ir = pr / den, ii = -pi / den;
    for (let r = c + 1; r < n; r++) {
      const xr = ar[r * n + c], xi = ai[r * n + c], fr = xr * ir - xi * ii, fi = xr * ii + xi * ir;
      for (let j = c; j < n; j++) {
        const yr = ar[c * n + j], yi = ai[c * n + j];
        ar[r * n + j] -= fr * yr - fi * yi;
        ai[r * n + j] -= fr * yi + fi * yr;
      }
      for (let j = 0; j < m; j++) {
        const yr = br[c * m + j], yi = bi[c * m + j];
        br[r * m + j] -= fr * yr - fi * yi;
        bi[r * m + j] -= fr * yi + fi * yr;
      }
    }
  }
  for (let c = n - 1; c >= 0; c--) {
    const pr = ar[c * n + c], pi = ai[c * n + c], den = pr * pr + pi * pi || 1e-30, ir = pr / den, ii = -pi / den;
    for (let j = 0; j < m; j++) {
      let sr = br[c * m + j], si = bi[c * m + j];
      for (let k = c + 1; k < n; k++) {
        const xr = br[k * m + j], xi = bi[k * m + j];
        sr -= ar[c * n + k] * xr - ai[c * n + k] * xi;
        si -= ar[c * n + k] * xi + ai[c * n + k] * xr;
      }
      br[c * m + j] = sr * ir - si * ii;
      bi[c * m + j] = sr * ii + si * ir;
    }
  }
  return [br, bi];
}

class Focus {
  constructor(srcs, pos) {  // pos: each mic's [x left, y front, z up] in m
    const n = (this.n = srcs.length), nn = n * n;
    this.srcs = srcs.slice();
    this.pos = pos;
    this.hist = [];
    this.hold = this.held = this.ref = 0;
    this.since = 99;
    this.wasPitched = false;
    this.pm = new Float64Array(n);
    this.acRe = new Float64Array(FN);
    this.acIm = new Float64Array(FN);
    this.target = null;
    this.tc = new Float64Array(FB * n);
    this.ts = new Float64Array(FB * n);
    this.sNow = new Float64Array(FB);
    this.sSm = new Float64Array(FB);
    this.sT = new Float64Array(FB).fill(1);  // how well speech lines up on the talker, per bin
    this.sN = new Float64Array(FB).fill(1);  // how well the background does
    this.gDir = new Float64Array(FB).fill(1);
    this.buf = srcs.map(() => new Float64Array(FN));
    this.pend = srcs.map(() => new Float64Array(FH));
    this.np = 0;
    this.fill = 0;
    this.Xre = srcs.map(() => new Float64Array(FN));
    this.Xim = srcs.map(() => new Float64Array(FN));
    this.Yre = new Float64Array(FN);
    this.Yim = new Float64Array(FN);
    this.ola = new Float64Array(FN);
    this.RnRe = new Float64Array(FB * nn);
    this.RnIm = new Float64Array(FB * nn);
    for (let k = 0; k < FB; k++) for (let a = 0; a < n; a++) this.RnRe[k * nn + a * n + a] = 1e-6;
    this.RsRe = Float64Array.from(this.RnRe);
    this.RsIm = new Float64Array(FB * nn);
    this.wRe = null;
    this.Pn = new Float64Array(FB).fill(1e-3);
    this.gPrev = new Float64Array(FB).fill(1);
    this.postPrev = new Float64Array(FB).fill(1);
    this.floor = null;
    this.nNoise = this.nSpeech = this.frames = 0;
    this.wide = [];
    this.enabled = true;
  }
  push(s, frames, cb) {  // s: interleaved int16 packet
    for (let i = 0; i < frames; i++) {
      for (let q = 0; q < this.n; q++) this.pend[q][this.np] = s[i * CH + this.srcs[q]];
      if (++this.np === FH) { this.np = 0; this.frame(cb); }
    }
  }
  // the talker's direction (azimuth + = left, elevation), from the caption tracker
  setTarget(az, el) {
    if (this.target && Math.abs(wrap(az - this.target[0])) < 2 && Math.abs(el - this.target[1]) < 2) return;
    this.target = [az, el];
    const th = (az * Math.PI) / 180, ph = (el * Math.PI) / 180, n = this.n;
    for (let q = 0; q < n; q++) {
      const [x, y, z] = this.pos[q], tau = -((x * Math.sin(th) + y * Math.cos(th)) * Math.cos(ph) + z * Math.sin(ph)) / C;
      for (let k = 0; k < FB; k++) {
        const a = (2 * Math.PI * k * FS * tau) / FN;
        this.tc[k * n + q] = Math.cos(a);
        this.ts[k * n + q] = Math.sin(a);
      }
    }
  }
  // Is anything a bang, tap or handling noise, and is there a voice? (loud: 8 dB over the floor)
  detect(e, w, h, loud, quiet) {
    const hist = this.hist;
    let med = w, hmed = h, prev = w;
    if (hist.length >= 12) {
      const a = hist.slice(0, 10).map((v) => v[0]).sort((x, y) => x - y), b = hist.slice(0, 10).map((v) => v[1]).sort((x, y) => x - y);
      med = (a[4] + a[5]) / 2;
      hmed = (b[4] + b[5]) / 2;
      prev = hist[11][0];
    }
    hist.push([w, h]);
    if (hist.length > 12) hist.shift();
    let hi = 0, n = this.n;
    const pm = Array.from(this.pm).sort((x, y) => x - y);
    for (let q = 0; q < n; q++) hi = Math.max(hi, this.pm[q]);
    const mid = n % 2 ? pm[(n - 1) / 2] : (pm[n / 2 - 1] + pm[n / 2]) / 2;
    // a knock arrives in one frame and in every band; a word builds up over several, mostly low
    const onset = w - prev > 8 && w - med > ONSET_DB && h - hmed > ONSET_DB && e - this.floor > 12;
    const handled = n > 2 && loud && 10 * Math.log10(hi / (mid + 1e-12)) > SPREAD_DB;
    if (onset || handled) {
      this.hold = HOLD;
      this.held = 0;
      this.ref = onset ? med : this.floor;
    } else if (this.hold) {
      this.held++;
      this.hold--;
      if (w > this.ref + 6 && this.held < HOLD_MAX) this.hold = Math.max(this.hold, 1);
    }
    // pitch: correlation of the summed mics' 300–2000 Hz harmonics at 80–400 Hz periods
    const re = this.acRe, im = this.acIm;
    re.fill(0);
    im.fill(0);
    for (let k = FV0; k <= FV1; k++) {
      let sr = 0, si = 0;
      for (let q = 0; q < n; q++) { sr += this.Xre[q][k]; si += this.Xim[q][k]; }
      re[k] = re[FN - k] = sr * sr + si * si;
    }
    fft512(re, im);
    let best = 0;
    for (let l = LAG0; l < LAG1; l++) best = Math.max(best, re[l] / FRW[l]);
    const pitched = re[0] > 0 && best > VOICED * re[0], voiced = pitched && this.wasPitched;
    this.wasPitched = pitched;
    this.since = voiced ? 0 : this.since + 1;
    return { speech: loud && this.since <= HANG && !this.hold, quiet: quiet && !this.hold && !voiced };
  }
  // per bin, how well the mics line up on the talker; returns how much this frame comes from them
  // (about 1 = all, 0 = no more than the background; null while they don't stand out from it)
  direction(speech, quiet) {
    const n = this.n, s = this.sNow, sT = this.sT, sN = this.sN, tc = this.tc, ts = this.ts;
    let sw = 0, swt = 0, num = 0, den = 0;
    for (let k = 0; k < FB; k++) {
      let lr = 0, li = 0, P = 0;
      for (let q = 0; q < n; q++) {
        const xr = this.Xre[q][k], xi = this.Xim[q][k], c = tc[k * n + q], si = ts[k * n + q];
        lr += xr * c - xi * si;
        li += xr * si + xi * c;
        P += xr * xr + xi * xi;
      }
      s[k] = (lr * lr + li * li) / (n * P + 1e-12);
      if (k >= FK0 && k <= FK1) {
        const span = sT[k] - sN[k], trust = Math.min(1, Math.max(0, (span - 0.1) / 0.2));
        sw += P;
        swt += P * trust;
        num += P * trust * (s[k] - sN[k]);
        den += P * trust * span;
      }
    }
    for (let k = 0; k < FB; k++) {
      const s3 = 0.5 * s[k] + 0.25 * (k > 0 ? s[k - 1] : 0) + 0.25 * (k < FB - 1 ? s[k + 1] : 0);
      this.sSm[k] = 0.5 * this.sSm[k] + 0.5 * s3;
      const span = sT[k] - sN[k], trust = Math.min(1, Math.max(0, (span - 0.1) / 0.2));
      const g = Math.min(1, Math.max(POST_FLOOR, (this.sSm[k] - sN[k]) / Math.max(span, 0.05)));
      this.gDir[k] = 1 - trust * (1 - g);
      if (speech) sT[k] = 0.99 * sT[k] + 0.01 * s[k];
      if (quiet) sN[k] = 0.99 * sN[k] + 0.01 * s[k];
    }
    return swt > 0.2 * sw ? num / den : null;
  }
  frame(cb) {
    const n = this.n, nn = n * n;
    for (let q = 0; q < n; q++) { this.buf[q].copyWithin(0, FH); this.buf[q].set(this.pend[q], FN - FH); }
    this.fill = Math.min(FN, this.fill + FH);
    if (this.fill < FN) return;
    let eb = 0, ew = 0, e150 = 0, ehi = 0;
    for (let q = 0; q < n; q++) {
      const re = this.Xre[q], im = this.Xim[q], b = this.buf[q];
      for (let i = 0; i < FN; i++) { re[i] = b[i] * FWIN[i]; im[i] = 0; }
      fft512(re, im);
      let mb = 0;
      for (let k = FLO; k < FB; k++) {
        const p = re[k] * re[k] + im[k] * im[k];
        ew += p;
        if (k >= FW0) e150 += p;
        if (k >= FHI) ehi += p;
        if (k >= FK0 && k <= FK1) mb += p;
      }
      eb += mb;
      this.pm[q] = mb;
    }
    const e = 10 * Math.log10(eb / (n * (FK1 - FK0 + 1)) + 1e-12), w = 10 * Math.log10(ew / (n * (FB - FLO)) + 1e-12);
    if (this.floor === null) this.floor = e;
    else if (e < this.floor) this.floor = 0.9 * this.floor + 0.1 * e;
    else this.floor += 0.002;
    const loud = e > this.floor + 8;
    const { speech, quiet } = this.detect(e, 10 * Math.log10(e150 / n + 1e-12), 10 * Math.log10(ehi / n + 1e-12),
                                          loud, e < this.floor + 3);
    let bang = false;
    if (this.wide.length === 40) {
      const old = this.wide.slice(0, 37).sort((a, b) => a - b);
      bang = w - old[18] > 22 && w - this.floor > 25;
      this.wide.shift();
    }
    this.wide.push(w);
    this.frames++;
    const Yre = this.Yre, Yim = this.Yim;
    if (this.enabled) {
      const score = this.target ? this.direction(speech, quiet) : null;
      if (!this.target) this.gDir.fill(1);
      const learn = speech && (score === null || score > ON_TARGET);
      if (quiet || learn) {
        const Rre = quiet ? this.RnRe : this.RsRe, Rim = quiet ? this.RnIm : this.RsIm, a1 = quiet ? 0.99 : 0.98, a2 = 1 - a1;
        for (let k = 0; k < FB; k++) {
          const o = k * nn;
          for (let a = 0; a < n; a++) {
            const xr = this.Xre[a][k], xi = this.Xim[a][k];
            for (let b = 0; b < n; b++) {
              const yr = this.Xre[b][k], yi = this.Xim[b][k], j = o + a * n + b;
              Rre[j] = a1 * Rre[j] + a2 * (xr * yr + xi * yi);
              Rim[j] = a1 * Rim[j] + a2 * (xi * yr - xr * yi);
            }
          }
        }
        if (quiet) this.nNoise++; else this.nSpeech++;
      }
      if (this.frames % 32 === 0 && this.nNoise > 100 && this.nSpeech > 30) this.updateWeights();
      for (let k = 0; k < FB; k++) {
        let yr = 0, yi = 0;
        for (let q = 0; q < n; q++) {
          const xr = this.Xre[q][k], xi = this.Xim[q][k];
          if (this.wRe) {
            const wr = this.wRe[k * n + q], wi = this.wIm[k * n + q];
            yr += wr * xr + wi * xi;
            yi += wr * xi - wi * xr;
          } else { yr += xr / n; yi += xi / n; }
        }
        if (k < FLO) yr = yi = 0;
        const py = yr * yr + yi * yi;
        if (quiet) this.Pn[k] = 0.98 * this.Pn[k] + 0.02 * py;
        const post = py / (this.Pn[k] + 1e-12);
        const prio = 0.98 * this.gPrev[k] ** 2 * this.postPrev[k] + 0.02 * Math.max(post - 1, 0);
        const g = Math.max(prio / (1 + prio), 0.12), gt = Math.max(g * this.gDir[k], TOTAL_FLOOR);
        this.gPrev[k] = g;
        this.postPrev[k] = post;
        Yre[k] = yr * gt;
        Yim[k] = yi * gt;
      }
    } else {
      for (let k = 0; k < FB; k++) {
        let yr = 0, yi = 0;
        for (let q = 0; q < n; q++) { yr += this.Xre[q][k] / n; yi += this.Xim[q][k] / n; }
        Yre[k] = k < FLO ? 0 : yr;
        Yim[k] = k < FLO ? 0 : yi;
      }
    }
    Yim[0] = Yim[FN / 2] = 0;
    for (let k = FB; k < FN; k++) { Yre[k] = Yre[FN - k]; Yim[k] = Yim[FN - k]; }  // conjugated already for the inverse
    for (let k = 1; k < FB - 1; k++) Yim[k] = -Yim[k];
    fft512(Yre, Yim);
    for (let i = 0; i < FN; i++) this.ola[i] += ((Yre[i] / FN) * FWIN[i]) / 2;  // sqrt-Hann in and out, 75% overlap
    const out = this.ola.slice(0, FH);
    this.ola.copyWithin(0, FH);
    this.ola.fill(0, FN - FH);
    cb(out, speech, bang, e - this.floor);
  }
  updateWeights() {
    const n = this.n, nn = n * n;
    if (!this.wRe) {
      this.wRe = new Float64Array(FB * n);
      this.wIm = new Float64Array(FB * n);
      this.vRe = new Float64Array(FB * n).fill(1);
      this.vIm = new Float64Array(FB * n);
    }
    const Nre = new Float64Array(nn), Nim = new Float64Array(nn);
    for (let k = 0; k < FB; k++) {
      const o = k * nn;
      let tr = 0;
      for (let a = 0; a < n; a++) tr += this.RnRe[o + a * n + a];
      const load = (1e-3 * tr) / n + 1e-12;
      for (let j = 0; j < nn; j++) { Nre[j] = this.RnRe[o + j]; Nim[j] = this.RnIm[o + j]; }
      for (let a = 0; a < n; a++) Nre[a * n + a] += load;
      // talker steering vector: principal generalized eigenvector of (Rs, Rn), by power iteration
      const [Mre, Mim] = cSolve(Nre, Nim, n, this.RsRe.subarray(o, o + nn), this.RsIm.subarray(o, o + nn), n);
      let vr = this.vRe.slice(k * n, k * n + n), vi = this.vIm.slice(k * n, k * n + n);
      for (let it = 0; it < 12; it++) {
        const ur = new Float64Array(n), ui = new Float64Array(n);
        let norm = 0;
        for (let a = 0; a < n; a++) {
          for (let b = 0; b < n; b++) {
            const mr = Mre[a * n + b], mi = Mim[a * n + b];
            ur[a] += mr * vr[b] - mi * vi[b];
            ui[a] += mr * vi[b] + mi * vr[b];
          }
          norm += ur[a] * ur[a] + ui[a] * ui[a];
        }
        norm = Math.sqrt(norm) || 1;
        for (let a = 0; a < n; a++) { ur[a] /= norm; ui[a] /= norm; }
        vr = ur;
        vi = ui;
      }
      this.vRe.set(vr, k * n);
      this.vIm.set(vi, k * n);
      const dr = new Float64Array(n), di = new Float64Array(n);
      for (let a = 0; a < n; a++) {
        for (let b = 0; b < n; b++) {
          dr[a] += Nre[a * n + b] * vr[b] - Nim[a * n + b] * vi[b];
          di[a] += Nre[a * n + b] * vi[b] + Nim[a * n + b] * vr[b];
        }
      }
      const d0 = dr[0] * dr[0] + di[0] * di[0];
      if (d0 > 1e-24) {
        const ir = dr[0] / d0, ii = -di[0] / d0;
        for (let a = n - 1; a >= 0; a--) {
          const xr = dr[a], xi = di[a];
          dr[a] = xr * ir - xi * ii;
          di[a] = xr * ii + xi * ir;
        }
      }
      const [Rr, Ri] = cSolve(Nre, Nim, n, dr, di, 1);
      let sr = 1e-12, si = 0;
      for (let a = 0; a < n; a++) { sr += dr[a] * Rr[a] + di[a] * Ri[a]; si += dr[a] * Ri[a] - di[a] * Rr[a]; }
      const den = sr * sr + si * si;
      for (let a = 0; a < n; a++) {
        this.wRe[k * n + a] = (Rr[a] * sr + Ri[a] * si) / den;
        this.wIm[k * n + a] = (Ri[a] * sr - Rr[a] * si) / den;
      }
    }
  }
}

// ------------------------------------------------------------------ captions (offline, from the focused array audio)

const MODEL_URL = "models/vosk-model-small-en-us-0.15.tar.gz";
const JUNK = new Set(["the", "a", "huh", "uh", "um", "hm", "oh"]);
const cap = { on: S.cc, lines: [], interim: null, gain: 20, lastBang: 0, voiceUntil: 0 };
const asr = { rec: null, loading: false, status: "", buf: new Float32Array(1024), n: 0, fed: 0 };
let focus = null;

// Talker direction: voice-band SRP on the frames voice focus marks as speech, each frame's curve
// normalised so one loud frame can't outvote the rest, stamped with the recognizer's audio clock.
// A caption takes the direction from exactly the time span of its words, so it stays right even when
// the recognizer runs behind. (Quiet speech under a fan never trips the clap-style detector, so this
// can't come from model.last.)
const utt = { log: [], tick: 0, skip: 0, lastEnd: 0 };
// the voice focus's aim: the direction curves of the last second or so of speech, added up
const aim = { acc: new Float64Array(360), el: 0, w: 0 };

function trackSpeechDir(weight) {
  const act = model.active();
  if (act.length < 2) return;
  const st = model.steering(act.map((a) => a[0]), 3400);
  const c = model.srp(act.map((a) => a[1]), st), sorted = Array.from(c).sort((a, b) => a - b);
  const med = (sorted[179] + sorted[180]) / 2, span = Math.max(sorted[359] - med, 1e-9);
  const curve = new Float32Array(360);
  for (let k = 0; k < 360; k++) curve[k] = (weight * (c[k] - med)) / span;
  let kb = 0;
  for (let k = 0; k < 360; k++) {
    aim.acc[k] = 0.9 * aim.acc[k] + curve[k];
    if (aim.acc[k] > aim.acc[kb]) kb = k;
  }
  if (c.el != null) {
    aim.el = 0.9 * aim.el + weight * c.el;
    aim.w = 0.9 * aim.w + weight;
  }
  focus?.setTarget(kb - 180, aim.w > 0 ? aim.el / aim.w : 0);
  const t = asr.fed / FS;
  utt.log.push({ t, curve, el: c.el ?? null, back: st.hasBack });
  while (utt.log.length && t - utt.log[0].t > 60) utt.log.shift();
}

// The phrase's direction curve; with a tall array, sum.el is the elevation of the frames that point the
// phrase's way, each weighted by how strongly it does.
function speechCurve(t0 = utt.lastEnd, t1 = Infinity) {
  const sum = new Float64Array(360), used = [];
  let n = 0, back = true;
  for (const e of utt.log) {
    if (e.t < t0 || e.t > t1) continue;
    for (let k = 0; k < 360; k++) sum[k] += e.curve[k];
    if (e.el !== null) used.push(e);
    back = e.back;
    n++;
  }
  if (used.length) {
    let k = 0;
    for (let i = 1; i < 360; i++) if (sum[i] > sum[k]) k = i;
    let es = 0, ws = 0;
    for (const e of used) {
      const w = Math.max(0, e.curve[k]);
      es += w * e.el;
      ws += w;
    }
    sum.el = ws > 0 ? es / ws : used[used.length - 1].el;
  }
  return { sum, n, back };
}

function speechAngle(t0, t1) {
  const { sum, n, back } = speechCurve(t0, t1);
  if (!n) return null;
  let k = 0;
  for (let i = 1; i < 360; i++) if (sum[i] > sum[k]) k = i;
  let a = k - 180;
  if (!back && Math.abs(a) > 90) a = Math.sign(a) * (180 - Math.abs(a));
  return a;
}

const tidy = (t) => {
  t = t.replace(/\bi\b/g, "I").replace(/\bi'(m|ll|ve|d)\b/g, "I'$1");
  return t.charAt(0).toUpperCase() + t.slice(1);
};

// t0/t1: the words' times on the recognizer's audio clock, so a short pause mid-sentence joins the line
// even when results arrive late and bunched up. Returns the piece, so Moonshine can swap its words later.
function addLine(text, angle, kind = "speech", t0 = null, t1 = null) {
  const now = performance.now() / 1000, last = cap.lines[cap.lines.length - 1];
  const sameTalker = last && (angle === null || last.angle === null || Math.abs(wrap(angle - last.angle)) < 25);
  let line;
  if (kind === "speech" && last && last.kind === "speech" && sameTalker && t0 !== null && last.t1 !== null
      && t0 - last.t1 < 1.5 && last.text.length + text.length < 160) {
    line = last;
    line.wall = now;
    line.t1 = t1;
  } else {
    line = { wall: now, text: "", parts: [], angle, kind, t1 };
    cap.lines.push(line);
    if (cap.lines.length > 200) cap.lines.shift();
  }
  const part = { text, line };
  line.parts.push(part);
  line.text = line.parts.map((p) => p.text).join(" ");
  return part;
}

function setPart(part, text) {
  part.text = text;
  part.line.text = part.line.parts.map((p) => p.text).join(" ");
}

// Better words: Vosk shows text instantly, then a bigger model re-reads each finished sentence from the
// same focused audio and replaces it. Which one depends on what the device can run (S.stt, More menu).
const HQ_URL = "asr_worker.js";
const hq = { worker: null, ready: false, status: "", queue: [], busy: null, id: 0, ring: new Float32Array(FS * 40), last: "" };
const STT = {
  light: "Light: instant words only (old phones)",
  standard: "Standard: Moonshine base (tablets)",
  better: "Better: Whisper small, 400 MB (laptops)",
  best: "Best: Whisper large-v3 turbo, 1.5 GB (strong graphics chip)",
};

// Whisper only runs well on a graphics chip, and phones/tablets would rather keep their battery.
// (Chrome reports at most 8 GB of memory, so cores and the GPU's maker are the better hints.)
const sttAuto = (async () => {
  const cores = navigator.hardwareConcurrency || 2, mem = navigator.deviceMemory || 4;
  if (/Android|iPhone|iPad/.test(navigator.userAgent)) return ["standard", "a phone or tablet"];
  if (cores <= 2 || mem < 2) return ["light", "a slow device"];
  let gpu = null;
  try { gpu = await navigator.gpu?.requestAdapter({ powerPreference: "high-performance" }); } catch (e) { /* no WebGPU */ }
  if (!gpu || gpu.isFallbackAdapter || cores < 8) return ["standard", cores < 8 ? `${cores} cores` : "no usable graphics chip"];
  const vendor = (gpu.info?.vendor || "").toLowerCase();
  if (mem >= 8 && (/nvidia|amd|ati/.test(vendor) || cores >= 12)) return ["best", `${vendor || "a"} graphics chip, ${cores} cores`];
  return ["better", `${vendor ? vendor + " " : ""}graphics chip, ${cores} cores`];
})();

async function hqLoad() {
  if (hq.worker || hq.picking || typeof Worker === "undefined") return;
  hq.picking = true;
  const key = S.stt in STT ? S.stt : (await sttAuto)[0];
  hq.picking = false;
  if (key === "light" || hq.worker || !asr.rec) return;
  try {
    hq.worker = new Worker(HQ_URL, { type: "module" });
  } catch (e) {
    hq.status = "better words unavailable";
    return;
  }
  const worker = hq.worker;
  hq.status = "loading better words…";
  worker.onmessage = ({ data: m }) => {
    if (hq.failed || worker !== hq.worker) return;
    if (m.type === "progress") hq.status = `loading better words ${m.mb.toFixed(0)}/${m.total.toFixed(0)} MB (first time only)`;
    else if (m.type === "stage") hq.status = m.text;
    else if (m.type === "ready") {
      Object.assign(hq, { ready: true, status: "", threads: m.threads, name: m.name + (m.device === "webgpu" ? " (GPU)" : "") });
      hqNext();
    }
    else if (m.type === "error") {
      console.warn("better words:", m.message);
      if (!hq.ready) {
        hq.failed = true;
        hq.status = /WebGPU/.test(m.message) ? m.message : "better words didn't load";
        hint(`${STT[key].split(":")[0]} speech model didn't load (${m.message}); captions keep the instant words.`);
      }
    }
    else if (m.type === "done") {
      const job = hq.busy, text = unbracket(m.text);
      hq.busy = null;
      if (job && job.id === m.id && plausible(text, m.sec) && !(FILLER.test(text) && !FILLER.test(job.part.text))) setPart(job.part, text);
      hq.last = `${(m.ms / 1000).toFixed(1)} s for ${m.sec.toFixed(1)} s`;
      hqNext();
    }
  };
  worker.onerror = (e) => { hq.status = "better words didn't load"; console.warn("better words worker:", e.message); };
  worker.postMessage({ type: "load", model: key });
}

function hqUnload() {
  hq.worker?.terminate();
  Object.assign(hq, { worker: null, ready: false, failed: false, status: "", queue: [], busy: null, last: "", name: "" });
}

// Whisper writes "[BLANK_AUDIO]", "(wind blowing)" or a lone "Thank you." for noise; keep Vosk's words then
const unbracket = (t) => t.replace(/\[[^\]]*\]|\([^)]*\)|♪/g, " ").replace(/\s+/g, " ").trim();
const FILLER = /^(thank you|thanks for watching|you|bye)[.!]?$/i;

// Moonshine can loop on noise ("the the the …"); keep Vosk's words then
function plausible(text, sec) {
  const w = text.toLowerCase().replace(/[^a-z0-9' ]/g, " ").split(/\s+/).filter(Boolean);
  if (!w.length || w.length > 5 * sec + 4) return false;
  const seen = new Map();
  for (let i = 0; i + 2 < w.length; i++) {
    const k = w.slice(i, i + 3).join(" ");
    seen.set(k, (seen.get(k) || 0) + 1);
    if (seen.get(k) >= 3) return false;
  }
  return true;
}

function hqSubmit(part, t0, t1) {
  if (!hq.worker) return;
  const ring = hq.ring, L = ring.length;
  const s0 = Math.max(Math.round((t0 - 0.3) * FS), asr.fed - L + 1, 0), s1 = Math.min(Math.round((t1 + 0.3) * FS), asr.fed);
  if (s1 - s0 < FS * 0.25) return;
  const pad = Math.max(0, Math.ceil((FS - (s1 - s0)) / 2));  // at least 1 s, padded with silence
  const audio = new Float32Array(s1 - s0 + 2 * pad);
  for (let i = s0; i < s1; i++) audio[pad + i - s0] = ring[i % L];
  hq.queue.push({ id: ++hq.id, part, audio });
  while (hq.queue.length > 4) hq.queue.shift();  // falling behind: the oldest keeps Vosk's words
  hqNext();
}

function hqNext() {
  if (!hq.ready || hq.busy || !hq.queue.length) return;
  hq.busy = hq.queue.shift();
  hq.worker.postMessage({ type: "run", id: hq.busy.id, audio: hq.busy.audio });
}

function loadScript(src) {
  return new Promise((resolve, reject) => {
    const s = document.createElement("script");
    s.src = src;
    s.onload = resolve;
    s.onerror = () => reject(new Error(`couldn't load ${src}`));
    document.head.appendChild(s);
  });
}

async function asrLoad() {
  if (asr.rec || asr.loading) return;
  asr.loading = true;
  asr.status = "loading the speech model (41 MB the first time, then kept on this device)…";
  try {
    if (!window.Vosk) await loadScript("lib/vosk.js");
    // the model is cached under its URL inside the recognizer's worker, so the URL has to stay the same
    const vm = await Vosk.createModel(new URL(MODEL_URL, location.href).href, -1);
    if (!cap.on) { vm.terminate?.(); asr.loading = false; asr.status = ""; return; }  // turned off while loading
    asr.model = vm;
    const rec = new vm.KaldiRecognizer(FS);
    rec.on("partialresult", (m) => {
      const t = (m.result.partial || "").trim();
      cap.interim = t && !JUNK.has(t) ? { text: tidy(t), angle: speechAngle() } : null;
    });
    rec.on("result", (m) => {
      const t = (m.result.text || "").trim(), words = m.result.result || [];
      cap.interim = null;
      if (!words.length) return;
      const t0 = words[0].start, t1 = words[words.length - 1].end;
      // directions are stamped when a frame leaves voice focus, but measured on the newest input, about
      // one focus window (32 ms) ahead of it
      if (t && !JUNK.has(t)) hqSubmit(addLine(tidy(t), speechAngle(t0 - 0.1 - FN / FS, t1 + 0.1), "speech", t0, t1), t0, t1);
      utt.lastEnd = t1;
    });
    rec.setWords(true);
    asr.rec = rec;
    asr.status = "";
    hqLoad();
  } catch (e) {
    asr.status = `speech model didn't load: ${e.message || e}`;
  }
  asr.loading = false;
}

function onFocusFrame(out, speech, bang, overFloor) {
  if (speech) {
    let s = 0;
    for (let i = 0; i < FH; i++) s += out[i] * out[i];
    const target = Math.min(400, Math.max(1, 3000 / (Math.sqrt(s / FH) + 1e-9)));
    cap.gain = 0.98 * cap.gain + 0.02 * target;
  }
  cap.voiceUntil = bang || utt.skip > 0 ? 0 : speech ? performance.now() + 150 : cap.voiceUntil;
  if (bang) utt.skip = 30;  // a clap is loud enough to count as speech; keep it out of the talker's direction
  if (utt.skip > 0) utt.skip--;
  else if (speech && asr.rec && ++utt.tick % 8 === 0) trackSpeechDir(Math.min(1, Math.max(0.1, (overFloor - 8) / 12)));
  const now = performance.now();
  if (bang && now - cap.lastBang > 600) {
    cap.lastBang = now;
    setTimeout(() => {  // let the direction finder catch it first
      const d = model.last;
      addLine("[BANG]", d && model.now - d.t < 0.5 ? d.angle : null, "sound");
    }, 150);
  }
  if (!asr.rec) return;
  const g = cap.gain / 32768, ring = hq.ring, base = asr.fed;
  asr.fed += FH;
  for (let i = 0; i < FH; i++) {
    const x = Math.max(-1, Math.min(1, out[i] * g));
    ring[(base + i) % ring.length] = x;
    asr.buf[asr.n++] = x;
    if (asr.n === asr.buf.length) {
      asr.rec.acceptWaveformFloat(asr.buf, FS);
      asr.n = 0;
    }
  }
}

function runFocus(s, frames) {
  const act = model.active(), srcs = act.map((a) => a[1]);
  if (!srcs.length) return;
  if (!focus || focus.srcs.join() !== srcs.join() || focus.geom !== model.geom) {
    focus = new Focus(srcs, act.map(([c]) => [model.geom[c][1] / 1000, model.geom[c][2] / 1000, (model.geom[c][3] || 0) / 1000]));
    focus.geom = model.geom;
  }
  focus.enabled = S.focus;
  focus.push(s, frames, onFocusFrame);
}

// ------------------------------------------------------------------ drawing

const cv = $("cv"), ctx = cv.getContext("2d");
let W = 0, H = 0;

function resize() {
  const r = cv.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  W = r.width;
  H = r.height;
  cv.width = Math.round(W * dpr);
  cv.height = Math.round(H * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
window.addEventListener("resize", resize);

const font = (px, bold = false, mono = false) =>
  `${bold ? "600 " : ""}${px}px ${mono ? "ui-monospace, Menlo, monospace" : "-apple-system, system-ui, Roboto, sans-serif"}`;
const alpha = (hex, a) => {
  const n = parseInt(hex.slice(1), 16);
  return `rgba(${n >> 16}, ${(n >> 8) & 255}, ${n & 255}, ${Math.max(0, Math.min(1, a))})`;
};
const R = (x, y, w, h) => ({ x, y, w, h, r: x + w, b: y + h, cx: x + w / 2, cy: y + h / 2 });

function rrect(x, y, w, h, rad) {
  ctx.beginPath();
  ctx.moveTo(x + rad, y);
  ctx.arcTo(x + w, y, x + w, y + h, rad);
  ctx.arcTo(x + w, y + h, x, y + h, rad);
  ctx.arcTo(x, y + h, x, y, rad);
  ctx.arcTo(x, y, x + w, y, rad);
  ctx.closePath();
}

function panel(r, title) {
  rrect(r.x, r.y, r.w, r.h, 10);
  ctx.fillStyle = PANEL;
  ctx.fill();
  ctx.strokeStyle = EDGE;
  ctx.lineWidth = 1;
  ctx.stroke();
  ctx.font = font(13, true);
  ctx.fillStyle = DIM;
  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";
  ctx.fillText(title, r.x + 14, r.y + 22);
}

function text(t, x, y, px, color, align = "left", bold = false, mono = false) {
  ctx.font = font(px, bold, mono);
  ctx.fillStyle = color;
  ctx.textAlign = align;
  ctx.textBaseline = "middle";
  ctx.fillText(t, x, y);
}

function wrapLines(t, maxW) {
  const words = t.split(/\s+/), out = [];
  let line = "";
  for (const w of words) {
    const test = line ? line + " " + w : w;
    if (ctx.measureText(test).width > maxW && line) { out.push(line); line = w; } else line = test;
  }
  if (line) out.push(line);
  return out;
}

function labelBox(x, y, lines, color, pic) {
  ctx.font = font(13, true);
  const w = Math.max(...lines.map((l, i) => { ctx.font = font(i ? 12 : 14, !i); return ctx.measureText(l).width; })) + 22;
  const h = 14 + lines.length * 19;
  if (x + w > pic.r - 4) x = x - w - 28;
  x = Math.max(pic.x + 4, x);
  y = Math.max(pic.y + 4, Math.min(pic.b - h - 4, y));
  rrect(x, y, w, h, 8);
  ctx.fillStyle = "rgba(13, 17, 23, 0.86)";
  ctx.fill();
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.stroke();
  lines.forEach((l, i) => text(l, x + 11, y + 16 + i * 19, i ? 12 : 14, i ? INK : INK, "left", !i));
}

function pictureRect(r) {
  const area = R(r.x + 12, r.y + 34, r.w - 24, r.h - 64);
  let aspect = 4 / 3;
  if (image) aspect = S.rotate % 180 ? image.height / image.width : image.width / image.height;
  const pw = Math.min(area.w, area.h * aspect), ph = pw / aspect;
  return R(area.cx - pw / 2, area.cy - ph / 2, pw, ph);
}

function drawCamera(r) {
  panel(r, "Camera + sound direction (360°)");
  const pic = pictureRect(r);
  if (image) {
    ctx.save();
    ctx.translate(pic.cx, pic.cy);
    ctx.rotate((S.rotate * Math.PI) / 180);
    if (S.mirror) ctx.scale(-1, 1);
    const w = S.rotate % 180 ? pic.h : pic.w, h = S.rotate % 180 ? pic.w : pic.h;
    ctx.drawImage(image, -w / 2, -h / 2, w, h);
    ctx.restore();
  } else {
    ctx.setLineDash([6, 5]);
    ctx.strokeStyle = EDGE;
    ctx.lineWidth = 1.5;
    ctx.strokeRect(pic.x, pic.y, pic.w, pic.h);
    ctx.setLineDash([]);
    text(conn && conn.name === "demo" ? "demo: no camera" : "no camera picture yet", pic.cx, pic.cy - 10, 18, DIM, "center");
    text("(sound directions still show)", pic.cx, pic.cy + 14, 14, DIM, "center");
  }
  ctx.save();
  ctx.beginPath();
  ctx.rect(pic.x, pic.y, pic.w, pic.h);
  ctx.clip();
  cal.pic = pic;
  const half = S.hfov / 2, f = pic.w / 2 / Math.tan((half * Math.PI) / 180);
  const xOf = (rel) => pic.cx - f * Math.tan((Math.max(-89, Math.min(89, rel)) * Math.PI) / 180);
  // ruler
  ctx.fillStyle = "rgba(0, 0, 0, 0.55)";
  ctx.fillRect(pic.x, pic.b - 24, pic.w, 24);
  for (let a = -Math.floor(half / 10) * 10; a <= half; a += 10) {
    const x = xOf(a);
    ctx.strokeStyle = a === 0 ? INK : DIM;
    ctx.lineWidth = a === 0 ? 2 : 1;
    ctx.beginPath();
    ctx.moveTo(x, pic.b - 24);
    ctx.lineTo(x, pic.b - (a % 20 === 0 ? 14 : 18));
    ctx.stroke();
    if (a % 20 === 0) text(`${Math.abs(a)}°`, x, pic.b - 8, 11, DIM, "center");
  }
  const now = model.now, recent = model.dets.filter((d) => now - d.t < HOLD_S);
  let behindShown = false;
  for (const d of recent) {
    const fade = 1 - (now - d.t) / HOLD_S, a = orient(d.angle), rel = wrap(a - S.offset), el = d.elev ?? null;
    const color = kindColor(d), latest = d === recent[recent.length - 1];
    const at = Math.abs(rel) < 90 ? toScreen(pic, a, el) : null;
    if (at && at[0] >= pic.x && at[0] <= pic.r) {
      const x = at[0], xs = (da) => toScreen(pic, a + da, el)?.[0] ?? x;
      const spread = Math.abs(xs(d.sigma) - xs(-d.sigma)) / 2;
      const hw = Math.max(18, spread) * 2, g = ctx.createLinearGradient(x - hw, 0, x + hw, 0);
      g.addColorStop(0, alpha(color, 0));
      g.addColorStop(0.5, alpha(color, latest ? 0.55 * fade : 0.25 * fade));
      g.addColorStop(1, alpha(color, 0));
      ctx.fillStyle = g;
      ctx.fillRect(x - hw, pic.y, 2 * hw, pic.h);
      ctx.strokeStyle = alpha("#fff3d6", fade);
      ctx.lineWidth = latest ? 2.5 : 1.5;
      ctx.beginPath();
      ctx.moveTo(x, pic.y);
      ctx.lineTo(x, pic.b);
      ctx.stroke();
      if (d.elev !== null && d.elev !== undefined) {  // the raised mics give its height: ring at that spot
        const y = Math.max(pic.y + 16, Math.min(pic.b - 40, at[1]));
        ctx.beginPath();
        ctx.arc(x, y, latest ? 16 : 10, 0, 2 * Math.PI);
        ctx.strokeStyle = alpha(color, (latest ? 1 : 0.6) * fade);
        ctx.lineWidth = latest ? 4 : 2;
        ctx.stroke();
      }
      if (latest) {
        const near = Math.abs(rel) + d.sigma > half, up = upText(d);
        labelBox(x + 14, pic.y + 0.16 * pic.h, [
          `${d.kind ? KINDS[d.kind].name : `${d.strength > 30 ? "LOUD · " : ""}Sound`} · ${near ? "near the edge" : "in view"}`,
          `${sideText(rel)}${up ? ` · ${up}` : ""} · ±${d.sigma.toFixed(0)}° · ${d.kind === "alarm" ? `${(d.freq / 1000).toFixed(1)} kHz tone`
            : d.kind ? `${(100 * d.score).toFixed(0)}% match` : `${d.strength.toFixed(0)} dB over quiet`}`,
          `${(now - d.t).toFixed(1)} s ago · ${fbText(d)}`], color, pic);
      }
    } else if (latest) {
      const behind = Math.abs(orient(d.angle)) > 90;
      if (Math.abs(rel) >= BEHIND_DEG) {
        drawBehindArrow(pic, rel, d, fade, color);
        behindShown = true;
      } else drawSideArrow(pic, rel, d, fade, color, behind, half);
    }
  }
  if (cal.on) drawCalib(pic);
  ctx.restore();
  drawCaptions(pic, f, behindShown ? 130 : 30);
  text("In view: glow at its spot. Out of view: arrow on that edge; behind you: arrow at the bottom.",
       r.x + 14, r.b - 14, 12, DIM);
}

function drawSideArrow(pic, rel, d, fade, color, behind, half) {
  const left = rel > 0, frac = Math.min(1, Math.max(0, (Math.abs(rel) - half) / (BEHIND_DEG - half)));
  const y = pic.y + pic.h * (0.18 + 0.6 * frac), x = left ? pic.x + 10 : pic.r - 10, s = left ? 1 : -1;
  ctx.fillStyle = alpha(color, 0.35 + 0.65 * fade);
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x + s * 34, y - 24);
  ctx.lineTo(x + s * 34, y + 24);
  ctx.closePath();
  ctx.fill();
  const up = upText(d);
  const lines = [`${sideText(rel)}${up ? ` · ${up}` : ""} · ${behind ? "behind you" : "in front"}`, `±${d.sigma.toFixed(0)}° · ${fbText(d)}`];
  ctx.font = font(14, true);
  const w = Math.max(...lines.map((l) => ctx.measureText(l).width)) + 22;
  labelBox(left ? x + 44 : x - 44 - w, y - 26, lines, color, pic);
}

function drawBehindArrow(pic, rel, d, fade, color) {
  const x = pic.cx, y = pic.b - 36;
  ctx.fillStyle = alpha(color, 0.35 + 0.65 * fade);
  ctx.beginPath();
  ctx.moveTo(x, y + 8);
  ctx.lineTo(x - 28, y - 22);
  ctx.lineTo(x + 28, y - 22);
  ctx.closePath();
  ctx.fill();
  const lines = [`behind you · ${sideText(orient(d.angle))}`, `±${d.sigma.toFixed(0)}° · ${fbText(d)}`];
  ctx.font = font(14, true);
  const w = Math.max(...lines.map((l) => ctx.measureText(l).width)) + 22;
  labelBox(x - w / 2, y - 90, lines, color, pic);
}

// Calibration on the picture: every spot faintly, the one to put the phone on as a big cross, and each
// captured point as a cross (where the phone was) joined to a dot (where the fit now puts its sound).
function drawCalib(pic) {
  const spot = (fx, fy) => [pic.x + fx * pic.w, pic.y + fy * pic.h];
  const cross = (x, y, s, color, w) => {
    ctx.strokeStyle = color;
    ctx.lineWidth = w;
    ctx.beginPath();
    ctx.moveTo(x - s, y); ctx.lineTo(x + s, y);
    ctx.moveTo(x, y - s); ctx.lineTo(x, y + s);
    ctx.stroke();
  };
  for (const [, fx, fy] of CAL_TARGETS) cross(...spot(fx, fy), 6, "rgba(230, 237, 243, 0.35)", 1);
  const labels = new Map();  // one label per spot, listing its distances
  for (const p of S.cal) {
    const key = `${p.fx.toFixed(3)},${p.fy.toFixed(3)}`;
    if (!labels.has(key)) labels.set(key, { p, d: new Set() });
    labels.get(key).d.add(p.dist);
  }
  for (const { p, d } of labels.values()) {
    const [tx, ty] = spot(p.fx, p.fy);
    const right = p.fx > 0.75;
    text(`${[...d].sort((a, b) => a - b).join(" · ")} m`, right ? tx - 12 : tx + 12, ty + 14, 11, INK, right ? "right" : "left");
  }
  for (const p of S.cal) {
    const [tx, ty] = spot(p.fx, p.fy), q = camProj(p.oa ?? calAngle(p, S.swap), p.e, p.dist);
    const f = calFix(p.oa ?? 0, p.e), col = p.err === undefined ? DIM : p.err < 2 ? "#3fb950" : p.err < 5 ? "#e3b341" : "#f85149";
    cross(tx, ty, 9, INK, 2);
    if (q) {
      const dx = pic.cx + ((q[0] + f[0]) * pic.w) / 2, dy = pic.cy - ((q[1] + f[1]) * pic.w) / 2;
      ctx.strokeStyle = col;
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(tx, ty); ctx.lineTo(dx, dy); ctx.stroke();
      ctx.fillStyle = col;
      ctx.beginPath(); ctx.arc(dx, dy, 5, 0, 2 * Math.PI); ctx.fill();
    }
  }
  if (cal.guide && cal.step >= CAL_GUIDE.length) {
    const errs = S.cal.map((p) => p.err ?? 0);
    labelBox(pic.cx - 120, pic.y + 20, ["Calibration done", `${S.cal.length} spots · worst ${Math.max(0, ...errs).toFixed(1)}° off`], "#3fb950", pic);
    return;
  }
  const [fx, fy] = calSpot(), [x, y] = spot(fx, fy);
  ctx.beginPath();
  ctx.arc(x, y, 22, 0, 2 * Math.PI);
  ctx.strokeStyle = "#56d4dd";
  ctx.lineWidth = 3;
  ctx.stroke();
  cross(x, y, 32, "#56d4dd", 3);
  const hits = calRecent().length;
  const name = cal.custom ? "Tapped spot" : CAL_TARGETS[cal.target][0];
  const lines = cal.guide
    ? [`Step ${cal.step + 1} of ${CAL_GUIDE.length} · phone's speaker here`, `${name} · ${S.calDist} m away`, cal.status || "waiting for chirps"]
    : [`Phone's speaker here · ${name}`, `${S.calDist} m away · ${hits} chirp${hits === 1 ? "" : "s"} heard`];
  labelBox(x + 30, y - 50, lines, "#56d4dd", pic);
}

function where(angle) {
  if (angle === null) return [null, "in"];
  if (S.camOff) {  // no picture: relative to straight ahead, "in" = roughly in front of you
    const a = orient(angle);
    return [a, Math.abs(a) <= 20 ? "in" : a > 0 ? "left" : "right"];
  }
  const rel = wrap(orient(angle) - S.offset);
  if (Math.abs(rel) <= S.hfov / 2) return [rel, "in"];
  return [rel, rel > 0 ? "left" : "right"];
}

function drawCaptions(pic, f, lift, opts = {}) {
  if (!cap.on) return;
  const now = performance.now() / 1000, keep = opts.rows || 3;
  const rows = cap.lines.filter((l) => now - l.wall < CAPTION_HOLD_S).slice(-Math.min(2, keep)).map((l) => ({ ...l, interim: false }));
  if (cap.interim) rows.push({ ...cap.interim, interim: true });
  const show = rows.slice(-keep);
  ctx.save();
  ctx.beginPath();
  ctx.rect(pic.x, pic.y, pic.w, pic.h);
  ctx.clip();
  if (cap.interim) {
    const [rel, side] = where(cap.interim.angle);
    if (rel !== null && side === "in" && !S.camOff) {
      const x = toScreen(pic, orient(cap.interim.angle), null)?.[0] ?? pic.cx - f * Math.tan((rel * Math.PI) / 180);
      ctx.font = font(13, true);
      const w = ctx.measureText("speaking").width + 30;
      rrect(x - w / 2, pic.y + 12, w, 26, 13);
      ctx.fillStyle = "rgba(13, 17, 23, 0.85)";
      ctx.fill();
      ctx.strokeStyle = SPEAKER;
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.fillStyle = SPEAKER;
      ctx.beginPath();
      ctx.arc(x - w / 2 + 13, pic.y + 25, 4, 0, 2 * Math.PI);
      ctx.fill();
      text("speaking", x + 7, pic.y + 25, 13, INK, "center", true);
    }
  }
  if (show.length) {
    const px = opts.px || Math.max(16, Math.round(pic.w / 30)), width = pic.w * 0.9, lh = px * 1.25;
    const blocks = show.map((row) => {
      const [, side] = where(row.angle);
      let label = row.text.length < 160 ? row.text : "…" + row.text.slice(-158);
      if (side === "left") label = "◀ " + label;
      else if (side === "right") label = label + " ▶";
      ctx.font = `${row.interim ? "italic " : ""}600 ${px}px -apple-system, system-ui, Roboto, sans-serif`;
      return { lines: wrapLines(label, width), interim: row.interim };
    });
    const total = blocks.reduce((s, b) => s + b.lines.length * lh, 0) + 10 * blocks.length + 10;
    let y = pic.b - lift - total;
    rrect(pic.cx - width / 2 - 14, y, width + 28, total, 10);
    ctx.fillStyle = "rgba(0, 0, 0, 0.7)";
    ctx.fill();
    y += 10;
    for (const b of blocks) {
      ctx.font = `${b.interim ? "italic " : ""}600 ${px}px -apple-system, system-ui, Roboto, sans-serif`;
      ctx.fillStyle = b.interim ? "#c9d1d9" : INK;
      ctx.textAlign = "center";
      ctx.textBaseline = "top";
      for (const l of b.lines) {
        ctx.fillText(l, pic.cx, y);
        y += lh;
      }
      y += 10;
    }
  }
  ctx.restore();
}

function polar(cx, cy, rad, ang) {
  const th = (ang * Math.PI) / 180;
  return [cx - rad * Math.sin(th), cy - rad * Math.cos(th)];
}
const canvasAngle = (ang) => -Math.PI / 2 - (ang * Math.PI) / 180;

function drawGauge(r) {
  panel(r, "Direction (top view, full circle)");
  const top = r.y + 74, cx = r.cx, cy = top + (r.b - top) / 2;
  const rad = Math.max(30, Math.min(r.w * 0.36, (r.b - top) / 2 - 22));
  ctx.beginPath();
  ctx.arc(cx, cy, rad, 0, 2 * Math.PI);
  ctx.fillStyle = "#0f141a";
  ctx.fill();
  ctx.strokeStyle = EDGE;
  ctx.lineWidth = 1.5;
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.arc(cx, cy, rad, canvasAngle(S.offset + S.hfov / 2), canvasAngle(S.offset - S.hfov / 2));
  ctx.closePath();
  ctx.fillStyle = "rgba(88, 166, 255, 0.18)";
  ctx.fill();
  ctx.strokeStyle = DIM;
  for (let a = -150; a <= 180; a += 30) {
    const [x1, y1] = polar(cx, cy, rad, a), [x2, y2] = polar(cx, cy, rad * 0.93, a);
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
  }
  for (const [t, a] of [["FRONT", 0], ["BACK", 180], ["LEFT", 90], ["RIGHT", -90]]) {
    const [x, y] = polar(cx, cy, rad + 18, a);
    text(t, x, y, 12, INK, "center", true);
  }
  const maxd = Math.max(1, ...Object.values(model.geom).map(([, x, y]) => Math.hypot(x, y)));
  const scale = (0.22 * rad) / maxd;
  model.chans.forEach((ch, k) => {
    const [, x, y, z] = model.geom[ch];
    const mx = S.swap ? -x : x, my = S.swapfb ? -y : y, col = model.source(ch) !== null ? COLORS[k % COLORS.length] : "#484f58";
    ctx.beginPath();
    if (z > 0) {  // raised mic, seen from above over the one below it: a ring
      ctx.arc(cx - mx * scale, cy - my * scale, 8, 0, 2 * Math.PI);
      ctx.strokeStyle = col;
      ctx.lineWidth = 2;
      ctx.stroke();
    } else {
      ctx.arc(cx - mx * scale, cy - my * scale, 4.5, 0, 2 * Math.PI);
      ctx.fillStyle = col;
      ctx.fill();
    }
  });
  const now = model.now;
  for (const d of model.dets) {
    const age = now - d.t;
    if (age > 6) continue;
    const [x, y] = polar(cx, cy, rad * 0.86, orient(d.angle));
    ctx.beginPath();
    ctx.arc(x, y, 5, 0, 2 * Math.PI);
    ctx.fillStyle = alpha(kindColor(d), 0.6 * (1 - age / 6));
    ctx.fill();
  }
  const d = model.last;
  if (d && now - d.t < 4) {
    const fade = Math.max(0.25, 1 - (now - d.t) / 4), a = orient(d.angle);
    const sorted = Array.from(d.curve).sort((p, q) => p - q), med = (sorted[179] + sorted[180]) / 2;
    const span = Math.max(1e-9, sorted[359] - med);
    ctx.beginPath();
    for (let i = 0; i < 360; i += 3) {
      const c = Math.max(0, (d.curve[i] - med) / span), [x, y] = polar(cx, cy, rad * (0.3 + 0.55 * c), orient(i - 180));
      if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y);
    }
    ctx.closePath();
    ctx.fillStyle = alpha("#58a6ff", 0.1 * fade);
    ctx.fill();
    ctx.strokeStyle = alpha("#58a6ff", 0.5 * fade);
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, rad - 8, canvasAngle(a + d.sigma), canvasAngle(a - d.sigma));
    ctx.closePath();
    ctx.fillStyle = alpha(d.kind ? kindColor(d) : GLOW, 0.25 * fade);
    ctx.fill();
    const [x, y] = polar(cx, cy, rad - 8, a);
    ctx.strokeStyle = alpha(d.kind ? kindColor(d) : "#ff7b72", fade);
    ctx.lineWidth = 5;
    ctx.lineCap = "round";
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(x, y);
    ctx.stroke();
    ctx.lineCap = "butt";
    const rel = wrap(a - S.offset);
    const whereTxt = Math.abs(rel) <= S.hfov / 2 ? "in view" : Math.abs(a) > 90 ? "behind" : "off screen";
    text(`${sideText(a)} · ${whereTxt}`, r.cx, r.y + 42, 19, INK, "center", true);
    const up = upText(d);
    text(`${up ? `${up} · ` : ""}${fbText(d)} · ${d.n} mics`, r.cx, r.y + 62, 12, DIM, "center", false, true);
  } else {
    const idle = { follow: "listening for the leader…", alarm: "listening for an alarm…", evac: "listening for exit and danger tones…" };
    text(S.find ? idle[S.findMode] : "clap, snap or talk", r.cx, r.y + 46, 15, DIM, "center");
  }
  ctx.beginPath();
  ctx.arc(cx, cy, 3, 0, 2 * Math.PI);
  ctx.fillStyle = INK;
  ctx.fill();
}

function drawLevels(r) {
  panel(r, "Mic levels (dBFS)");
  const chans = model.chans, x0 = r.x + 64, x1 = r.r - 52;
  const rowH = (r.h - 40) / Math.max(1, chans.length);
  const xf = (db) => x0 + Math.max(0, Math.min(1, (db + 80) / 80)) * (x1 - x0);
  chans.forEach((ch, k) => {
    const y = r.y + 34 + k * rowH + rowH / 2, color = COLORS[k % COLORS.length];
    text(`ch${ch}`, r.x + 14, y, 12, color, "left", true);
    const bh = Math.min(14, rowH - 5);
    rrect(x0, y - bh / 2, x1 - x0, bh, 4);
    ctx.fillStyle = "#21262d";
    ctx.fill();
    const src = model.source(ch);
    const name = model.geom[ch][0];
    if (src === null) {
      text(`${name}: no signal`, x0 + 8, y, 11, LOUD, "left", false, true);
      return;
    }
    rrect(x0, y - bh / 2, Math.max(4, xf(model.level[src]) - x0), bh, 4);
    ctx.fillStyle = color;
    ctx.fill();
    if (bh >= 11) text(name, x0 + 6, y, 10, INK, "left", true);
    const xn = xf(model.floor[src]);
    ctx.strokeStyle = DIM;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(xn, y - bh / 2 - 3);
    ctx.lineTo(xn, y + bh / 2 + 3);
    ctx.stroke();
    ctx.setLineDash([]);
    text(`${model.level[src].toFixed(0)}${src === ch ? "" : ` (ch${src})`}`, x1 + 8, y, 11, INK, "left", false, true);
  });
}

function drawTranscript(r) {
  panel(r, "Transcript");
  const failed = asr.status.startsWith("speech model didn't");
  const mics = focus ? `${focus.n} mic${focus.n === 1 ? "" : "s"}` : "waiting for mics";
  const state = !cap.on ? "captions off" : asr.status ? (failed ? "speech model failed" : "loading speech model…")
    : `offline · ${S.focus ? (focus && focus.wRe ? "voice focus" : "voice focus learning") : "plain mix"} · ${mics}`
      + (hq.status ? ` · ${hq.status}` : hq.ready ? ` · ${hq.name}${hq.last ? ` ${hq.last}` : ""}` : "");
  ctx.font = font(11);
  let st = state;
  while (st.length > 4 && ctx.measureText(st).width > r.w - 120) st = st.slice(0, -2);
  if (st !== state) st = st.trimEnd() + "…";
  text(st, r.r - 14, r.y + 18, 11, failed ? LOUD : DIM, "right");
  const inner = R(r.x + 14, r.y + 34, r.w - 28, r.h - 44);
  ctx.save();
  ctx.beginPath();
  ctx.rect(inner.x, inner.y, inner.w, inner.h);
  ctx.clip();
  let y = inner.b;
  if (cap.interim) {
    ctx.font = `italic ${font(14)}`;
    const lines = wrapLines(cap.interim.text, inner.w);
    y -= lines.length * 18;
    lines.forEach((l, j) => {
      ctx.font = `italic ${font(14)}`;
      ctx.fillStyle = "#c9d1d9";
      ctx.textAlign = "left";
      ctx.textBaseline = "middle";
      ctx.fillText(l, inner.x, y + j * 18 + 9);
    });
    y -= 8;
  }
  for (let i = cap.lines.length - 1; i >= 0 && y > inner.y; i--) {
    const ln = cap.lines[i];
    ctx.font = font(14);
    const lines = wrapLines(ln.text, inner.w);
    y -= lines.length * 18;
    lines.forEach((l, j) => text(l, inner.x, y + j * 18 + 9, 14, ln.kind === "sound" ? LOUD : INK, "left", ln.kind === "sound"));
    const [rel, side] = where(ln.angle);
    const t = new Date(performance.timeOrigin + ln.wall * 1000).toLocaleTimeString();
    y -= 16;
    text(`${t}${rel === null ? "" : ` · ${sideText(rel)}${side === "in" || S.camOff ? "" : " (off screen)"}`}`, inner.x, y + 7, 11, DIM);
    y -= 8;
  }
  if (!cap.lines.length && cap.on && asr.status) {
    ctx.font = font(13);
    wrapLines(asr.status, inner.w - 20).forEach((l, j) => text(l, inner.cx, inner.cy + j * 18, 13, failed ? LOUD : DIM, "center"));
  } else if (!cap.lines.length && !cap.interim) {
    text(cap.on ? "say something" : "tap CC for subtitles", inner.cx, inner.cy, 14, DIM, "center");
  }
  ctx.restore();
}

// Find mode's panel (in place of the transcript): which way to turn, and warmer / colder from the loudness
const turnText = (a) => (Math.abs(a) < 10 ? "Straight ahead" : Math.abs(a) > 165 ? "Behind you"
  : `Turn ${a > 0 ? "left" : "right"} ${Math.abs(a).toFixed(0)}°`);
const whereText = (a) => (Math.abs(a) < 15 ? "ahead" : Math.abs(a) > 165 ? "behind you"
  : `${Math.abs(a).toFixed(0)}° ${a > 0 ? "left" : "right"}`);
const trendText = (t) => (t > 2.5 ? ["Getting closer ▲", FOX] : t < -2.5 ? ["Getting farther ▼", LOUD] : ["Same distance", DIM]);
const beaconUrl = (role) => `${location.host}${location.pathname.replace(/[^/]*$/, "")}beacon.html?role=${role}`;

function drawFinder(r) {
  panel(r, `Find · ${FIND_MODES[S.findMode].name}`);
  const inner = R(r.x + 14, r.y + 34, r.w - 28, r.h - 44);
  const rad = Math.max(18, Math.min(inner.h * 0.38, inner.w * 0.18, 64)), cx = inner.x + rad + 4, cy = inner.y + rad + 8;
  const tx = cx + rad + 18, big = Math.min(24, (inner.r - tx) / 8);
  const dial = (arrows, wedges = []) => {
    ctx.strokeStyle = EDGE;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx, cy, rad, 0, 2 * Math.PI);
    ctx.stroke();
    text("ahead", cx, cy - rad - 6, 10, DIM, "center");
    for (const [a, col] of wedges) {  // keep-away zones
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.arc(cx, cy, rad, canvasAngle(a + 25), canvasAngle(a - 25));
      ctx.closePath();
      ctx.fillStyle = alpha(col, 0.35);
      ctx.fill();
    }
    ctx.lineCap = "round";
    for (const [a, col, w] of arrows) {
      ctx.strokeStyle = col;
      ctx.lineWidth = w;
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(...polar(cx, cy, rad * 0.85, a));
      ctx.stroke();
    }
    ctx.lineCap = "butt";
  };
  const idle = (lines, urls = []) => {
    ctx.font = font(13);
    let y = inner.y + 8;
    for (const l of lines) {
      const isUrl = urls.includes(l);
      for (const w of wrapLines(l, inner.w)) {
        text(w, inner.x, y, isUrl ? 13 : 14, isUrl ? FOX : l === lines[0] ? INK : DIM, "left", l === lines[0]);
        y += 19;
      }
    }
  };
  const noBack = (d) => d.fb === null;

  if (S.findMode === "follow") {
    const s = tracks(beacon.dets, "leader", 6).sort((a, b) => a.age - b.age)[0];  // where they are now, not where they were
    if (!s) return idle(["Listening for the leader's tone…", "", "The leader opens this on their phone and taps Start:", beaconUrl("leader"), "(Demo has one built in.)"], [beaconUrl("leader")]);
    const a = orient(s.angle), fresh = s.age < 2, col = fresh ? FOX : DIM, trend = trendText(s.trend);
    dial([[a, col, 6]]);
    text(fresh ? turnText(a) : `Lost them · last ${turnText(a).toLowerCase()}`, tx, cy - 22, big, col, "left", true);
    text(trend[0], tx, cy + 6, 15, fresh ? trend[1] : DIM, "left", true);
    text(`heard ${s.age.toFixed(1)} s ago · ${(100 * s.last.score).toFixed(0)}% match`, tx, cy + 28, 12, DIM);
    if (noBack(s.last)) text("no back mic: could also be behind", tx, cy + 46, 12, DIM);
    return;
  }

  if (S.findMode === "alarm") {
    const all = tracks(alarm.dets, "alarm", 6, [5, 10], 2).sort((a, b) => b.hits - a.hits), s = all[0];  // steadiest first
    if (!s) return idle(["Listening for an alarm…", "", "Smoke and fire alarms, beeping or steady. Press a smoke alarm's test button to try it (or use Demo)."]);
    const a = orient(s.angle), fresh = s.age < 5, col = fresh ? ALARM_COL : DIM, trend = trendText(s.trend);
    dial(all.slice(1).map((o) => [orient(o.angle), alpha(ALARM_COL, 0.4), 3]).concat([[a, col, 6]]));
    text(fresh ? `Alarm ${whereText(a)}` : `Alarm stopped · was ${whereText(a)}`, tx, cy - 22, big, col, "left", true);
    text(`${(s.last.freq / 1000).toFixed(1)} kHz · ${alarm.pattern(s.last.freq)}`, tx, cy + 6, 14, INK);
    text(fresh ? (trend[0] === "Same distance" ? "" : trend[0]) : "", tx, cy + 28, 13, trend[1], "left", true);
    text(all.length > 1 ? `${all.length} alarms heard · loudest shown` : noBack(s.last) ? "no back mic: could also be behind" : "",
      tx, cy + 46, 12, DIM);
    return;
  }

  const exits = tracks(beacon.dets, "exit", 4).filter((e) => e.age < 2);
  const dangers = tracks(beacon.dets, "danger", 4).filter((d) => d.age < 2);
  if (!exits.length && !dangers.length) {
    return idle(["Listening for exit and danger tones…", "", "A phone at each exit opens", beaconUrl("exit"), "and one where it isn't safe opens", beaconUrl("danger"), "(Demo has both built in.)"],
      [beaconUrl("exit"), beaconUrl("danger")]);
  }
  // an exit in the same direction as a danger may be past it: steer to another one if there is one
  const blocked = (e) => dangers.some((d) => Math.abs(wrap(orient(d.angle) - orient(e.angle))) < 35);
  const best = exits.find((e) => !blocked(e)) || exits[0];
  dial(exits.filter((e) => e !== best).map((e) => [orient(e.angle), alpha(FOX, 0.45), 3])
    .concat(best ? [[orient(best.angle), blocked(best) ? DIM : FOX, 6]] : []),
    dangers.map((d) => [orient(d.angle), DANGER]));
  if (best) {
    const a = orient(best.angle), trend = trendText(best.trend);
    text(blocked(best) ? "Exit is past the danger" : `Exit: ${turnText(a).replace("Turn ", "turn ")}`, tx, cy - 22, big, blocked(best) ? DANGER : FOX, "left", true);
    text(trend[0], tx, cy + 4, 14, trend[1], "left", true);
  } else {
    text("No exit heard yet", tx, cy - 22, big, DIM, "left", true);
  }
  text(dangers.length ? `Danger ${whereText(orient(dangers[0].angle))} · keep away` : "No danger heard", tx, cy + 26, 15,
    dangers.length ? DANGER : DIM, "left", true);
  const d0 = (best || dangers[0]).last;
  text(`${exits.length} exit${exits.length === 1 ? "" : "s"}, ${dangers.length} danger${dangers.length === 1 ? "" : "s"}${noBack(d0) ? " · no back mic: front/back unsure" : ""}`,
    tx, cy + 46, 12, DIM);
}

// ------------------------------------------------------------------ camera off: the big radar

const soundName = (d) => (d.kind ? KINDS[d.kind].name : d.voice ? "Voice" : d.strength > 30 ? "Loud sound" : "Sound");
const soundColor = (d) => (d.kind || !d.voice ? kindColor(d) : SPEAKER);
const vestText = (d) => {
  if (d.elev === null || d.elev === undefined) return "";
  const e = d.elev * S.kel;
  return Math.abs(e) < 8 ? "level with the vest" : `${Math.abs(e).toFixed(0)}° ${e > 0 ? "above" : "below"} the vest`;
};
const agoText = (s) => (s < 0.5 ? "just now" : `${s.toFixed(s < 10 ? 1 : 0)} s ago`);

function arrow(x0, y0, x1, y1, w, color) {
  const len = Math.hypot(x1 - x0, y1 - y0), head = Math.min(len * 0.6, w * 2.4);
  ctx.save();
  ctx.translate(x0, y0);
  ctx.rotate(Math.atan2(y1 - y0, x1 - x0));
  ctx.beginPath();
  ctx.moveTo(0, -w / 2);
  ctx.lineTo(len - head, -w / 2);
  ctx.lineTo(len - head, -head * 0.62);
  ctx.lineTo(len, 0);
  ctx.lineTo(len - head, head * 0.62);
  ctx.lineTo(len - head, w / 2);
  ctx.lineTo(0, w / 2);
  ctx.closePath();
  ctx.fillStyle = color;
  ctx.fill();
  ctx.restore();
}

// curved "turn this way" arrow from straight ahead toward angle a, at radius rad
function turnArc(cx, cy, rad, a, color) {
  const end = Math.sign(a) * Math.min(Math.abs(a) - 6, 150), th = canvasAngle(end);
  ctx.strokeStyle = color;
  ctx.lineWidth = 4;
  ctx.lineCap = "round";
  ctx.beginPath();
  ctx.arc(cx, cy, rad, canvasAngle(0), th, a > 0);
  ctx.stroke();
  ctx.lineCap = "butt";
  const dir = th + (a > 0 ? -Math.PI / 2 : Math.PI / 2), [hx, hy] = polar(cx, cy, rad, end);
  const ux = Math.cos(dir), uy = Math.sin(dir), k = 9;
  ctx.beginPath();
  ctx.moveTo(hx + ux * k, hy + uy * k);
  ctx.lineTo(hx - ux * k - uy * k, hy - uy * k + ux * k);
  ctx.lineTo(hx - ux * k + uy * k, hy - uy * k - ux * k);
  ctx.closePath();
  ctx.fillStyle = color;
  ctx.fill();
}

function pill(x, y, t, color, px = 13, bounds = null) {
  ctx.font = font(px, true);
  const w = ctx.measureText(t).width + 18, h = px + 12;
  if (bounds) {
    x = Math.max(bounds.x + w / 2 + 4, Math.min(bounds.r - w / 2 - 4, x));
    y = Math.max(bounds.y + h / 2 + 4, Math.min(bounds.b - h / 2 - 4, y));
  }
  rrect(x - w / 2, y - h / 2, w, h, h / 2);
  ctx.fillStyle = "rgba(13, 17, 23, 0.9)";
  ctx.fill();
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.stroke();
  text(t, x, y + 1, px, INK, "center", true);
}

// the wearer seen from above, facing up (the front), with the mics where they sit on the vest
function drawWearer(cx, cy, s) {
  ctx.beginPath();
  ctx.ellipse(cx, cy, s * 1.15, s * 0.42, 0, 0, 2 * Math.PI);
  ctx.fillStyle = "#30363d";
  ctx.fill();
  ctx.beginPath();
  ctx.arc(cx, cy - s * 0.12, s * 0.36, 0, 2 * Math.PI);
  ctx.fillStyle = "#484f58";
  ctx.fill();
  ctx.beginPath();
  ctx.moveTo(cx - s * 0.12, cy - s * 0.44);
  ctx.lineTo(cx, cy - s * 0.66);
  ctx.lineTo(cx + s * 0.12, cy - s * 0.44);
  ctx.closePath();
  ctx.fill();
  const maxd = Math.max(1, ...Object.values(model.geom).map(([, x]) => Math.abs(x)));
  model.chans.forEach((ch, k) => {
    const [, x, , z] = model.geom[ch];
    const mx = S.swap ? -x : x, col = model.source(ch) !== null ? COLORS[k % COLORS.length] : "#484f58";
    const px = cx - (mx / maxd) * s * 0.95, py = cy + s * 0.2 - (z > 0 ? s * 0.3 : 0);
    ctx.beginPath();
    ctx.arc(px, py, z > 0 ? 5 : 4, 0, 2 * Math.PI);
    if (z > 0) { ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.stroke(); } else { ctx.fillStyle = col; ctx.fill(); }
  });
}

// the last few separate sounds, newest first
function recentSounds(maxAge, n) {
  const out = [];
  for (let i = model.dets.length - 1; i >= 0 && out.length < n; i--) {
    const d = model.dets[i], age = model.now - d.t;
    if (age > maxAge) break;
    const a = orient(d.angle);
    // one entry per source: the same kind of sound from about the same way within a few seconds is the same one
    if (out.some((o) => soundName(o.d) === soundName(d) && Math.abs(wrap(o.a - a)) < 25 && o.d.t - d.t < 6)) continue;
    out.push({ d, a, age });
  }
  return out;
}

function drawRadar(r) {
  panel(r, "Sound around you · camera off");
  const now = model.now, d = model.last, live = d && now - d.t < 4;
  const head = Math.max(70, Math.min(96, r.h * 0.13));
  const capPx = Math.round(Math.max(15, Math.min(22, r.w / 42)));
  const foot = cap.on ? Math.min(r.h * 0.3, capPx * 1.25 * 4 + 40) : 20;  // two subtitles of two lines each
  const listW = r.w >= 560 ? Math.min(240, r.w * 0.3) : 0;  // room on the left for the recent sounds
  const cy = r.y + head + (r.h - head - foot) / 2, cx = r.x + listW + (r.w - listW) / 2;
  const rad = Math.max(40, Math.min((r.w - listW) * 0.36, (r.h - head - foot) / 2 - 30));
  const big = Math.max(18, Math.min(34, r.w / 24));

  // what and where, in words
  if (live) {
    const a = orient(d.angle), age = now - d.t;
    text(`${soundName(d)} · ${sideText(a)}`, cx, r.y + 30 + big / 2, big, soundColor(d), "center", true);
    const turn = Math.abs(a) < 20 ? "in front of you" : turnText(a);
    const bits = [turn, agoText(age), vestText(d), d.fb === null ? "or behind you: no back mic yet" : fbText(d)].filter(Boolean);
    text(bits.join(" · "), cx, r.y + 34 + big + 14, Math.max(12, big * 0.5), DIM, "center");
  } else {
    text(S.find ? "Find mode: see the panel on the right" : "Listening… clap, snap or talk", cx, r.y + 30 + big / 2,
      Math.max(16, big * 0.7), DIM, "center");
  }

  // the dial
  ctx.beginPath();
  ctx.arc(cx, cy, rad, 0, 2 * Math.PI);
  ctx.fillStyle = "#0f141a";
  ctx.fill();
  ctx.strokeStyle = EDGE;
  ctx.lineWidth = 2;
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.arc(cx, cy, rad, canvasAngle(20), canvasAngle(-20));
  ctx.closePath();
  ctx.fillStyle = "rgba(88, 166, 255, 0.09)";
  ctx.fill();
  ctx.setLineDash([4, 6]);
  ctx.strokeStyle = "#21262d";
  ctx.lineWidth = 1;
  for (const f of [0.36, 0.68]) {
    ctx.beginPath();
    ctx.arc(cx, cy, rad * f, 0, 2 * Math.PI);
    ctx.stroke();
  }
  ctx.setLineDash([]);
  for (let a = -165; a <= 180; a += 15) {
    const long = a % 45 === 0, [x1, y1] = polar(cx, cy, rad, a), [x2, y2] = polar(cx, cy, rad * (long ? 0.9 : 0.95), a);
    ctx.strokeStyle = long ? DIM : EDGE;
    ctx.lineWidth = long ? 2 : 1;
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
    if (a % 30 === 0 && a % 90 !== 0) {
      const [tx, ty] = polar(cx, cy, rad * 0.82, a);
      text(`${Math.abs(a)}°`, tx, ty, 11, DIM, "center");
    }
  }
  const lbl = Math.max(13, Math.min(18, rad / 12));
  for (const [t, a] of [["FRONT", 0], ["BACK", 180], ["LEFT", 90], ["RIGHT", -90]]) {
    const [x, y] = polar(cx, cy, rad + lbl + 8, a);
    text(t, x, y, lbl, INK, "center", true);
  }
  text("you face this way", cx, cy - rad * 0.78, 11, alpha("#58a6ff", 0.8), "center");
  drawWearer(cx, cy, rad * 0.17);

  // sounds of the last few seconds
  for (const det of model.dets) {
    const age = now - det.t;
    if (age > 8) continue;
    const [x, y] = polar(cx, cy, rad * 0.9, orient(det.angle));
    ctx.beginPath();
    ctx.arc(x, y, 6, 0, 2 * Math.PI);
    ctx.fillStyle = alpha(soundColor(det), 0.55 * (1 - age / 8));
    ctx.fill();
  }

  // who's talking (the subtitles' direction)
  const talk = cap.on && cap.interim && cap.interim.angle !== null && cap.interim.angle !== undefined ? orient(cap.interim.angle) : null;
  if (talk !== null) {
    const [x, y] = polar(cx, cy, rad * 0.9, talk);
    ctx.beginPath();
    ctx.arc(x, y, 11, 0, 2 * Math.PI);
    ctx.fillStyle = SPEAKER;
    ctx.fill();
    text("talking", x, y - 22, 12, SPEAKER, "center", true);
  }

  // the newest sound: a big arrow, how wide the estimate is, which way to turn
  if (live) {
    const a = orient(d.angle), fade = Math.max(0.35, 1 - (now - d.t) / 4), col = soundColor(d);
    const w = Math.max(10, rad * 0.075), r0 = rad * 0.26;
    if (d.fb === null && Math.abs(a) > 12 && Math.abs(a) < 168) {  // a flat array can't tell front from back
      const m = wrap(180 - a), [gx0, gy0] = polar(cx, cy, r0, m), [gx1, gy1] = polar(cx, cy, rad * 0.8, m);
      arrow(gx0, gy0, gx1, gy1, w * 0.55, alpha(col, 0.18 * fade));
      const [qx, qy] = polar(cx, cy, rad * 0.62, m);
      text("or here?", qx, qy, 12, alpha(INK, 0.55), "center");
    }
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, rad * 0.97, canvasAngle(a + d.sigma), canvasAngle(a - d.sigma));
    ctx.closePath();
    ctx.fillStyle = alpha(col, 0.16 * fade);
    ctx.fill();
    const [x0, y0] = polar(cx, cy, r0, a), [x1, y1] = polar(cx, cy, rad * 0.97, a);
    arrow(x0, y0, x1, y1, w, alpha(col, fade));
    if (Math.abs(a) >= 20) {
      turnArc(cx, cy, rad * 0.46, a, alpha(INK, 0.55 * fade));
      const mid = Math.sign(a) * Math.min(Math.abs(a), 150) / 2, [mx, my] = polar(cx, cy, rad * 0.46 + 18, mid);
      text(Math.abs(a) > 165 ? "turn around" : `turn ${a > 0 ? "left" : "right"}`, mx, my, 12, alpha(INK, 0.75 * fade), "center", true);
    }
    const nearAxis = Math.abs(a) < 22 || Math.abs(a) > 158;  // beside FRONT / BACK, not on top of them
    const [px, py] = polar(cx, cy, rad + lbl + (nearAxis ? 8 : 34), a);
    pill(px + (nearAxis ? (a > 0 ? -1 : 1) * (lbl * 3 + 40) : 0), py, sideText(a), col, 14,
      R(r.x, r.y + head, r.w, r.h - head - foot));
  }

  // list of recent sounds, each with its own little arrow
  const list = recentSounds(15, 5), lx = r.x + 16;
  if (listW) {
    let y = r.y + head + 14;
    text("Recent sounds", lx, y, 12, DIM, "left", true);
    y += 24;
    if (!list.length) text("none in the last 15 s", lx, y, 12, DIM);
    for (const { d: det, a, age } of list) {
      const col = soundColor(det), [ax, ay] = [lx + 12, y];
      ctx.beginPath();
      ctx.arc(ax, ay, 12, 0, 2 * Math.PI);
      ctx.fillStyle = "#0f141a";
      ctx.fill();
      ctx.strokeStyle = EDGE;
      ctx.lineWidth = 1;
      ctx.stroke();
      const [tx, ty] = polar(ax, ay, 10, a), [bx, by] = polar(ax, ay, -6, a);
      arrow(bx, by, tx, ty, 3, alpha(col, Math.max(0.35, 1 - age / 15)));
      text(`${soundName(det)} · ${sideText(a)}`, lx + 32, y - 7, 13, INK, "left", true);
      text(agoText(age), lx + 32, y + 9, 11, DIM);
      y += 40;
      if (y > r.b - foot - 10) break;
    }
  }

  drawCaptions(R(r.x + 12, r.y + 34, r.w - 24, r.h - 44), 0, 4, { rows: 2, px: capPx });
  ctx.beginPath();
  ctx.arc(cx, cy, 3, 0, 2 * Math.PI);
  ctx.fillStyle = INK;
  ctx.fill();
}

let lastDraw = 0;
function render(now) {
  requestAnimationFrame(render);
  if (now - lastDraw < 30) return;  // 30 fps is plenty here and leaves the tablet's CPU for audio and USB
  lastDraw = now;
  const t0 = performance.now();
  draw();
  busy.ms += performance.now() - t0;
}

function draw() {
  ctx.fillStyle = BG;
  ctx.fillRect(0, 0, W, H);
  const g = 10;
  if (S.camOff) {  // the radar takes the camera's place
    if (W >= H * 1.15) {
      const lw = W * 0.64, x1 = lw + g / 2, cw = W - x1 - g, th = (H - 3 * g) * 0.62;
      drawRadar(R(g, g, lw - 1.5 * g, H - 2 * g));
      (S.find ? drawFinder : drawTranscript)(R(x1, g, cw, th));
      drawLevels(R(x1, 2 * g + th, cw, H - 3 * g - th));
    } else {
      const rh = Math.min(W * 1.05, H * 0.62), y = rh + 2 * g, bh = H - y - g, lw = (W - 3 * g) * 0.4;
      drawRadar(R(g, g, W - 2 * g, rh));
      drawLevels(R(g, y, lw, bh));
      (S.find ? drawFinder : drawTranscript)(R(2 * g + lw, y, W - 3 * g - lw, bh));
    }
    return;
  }
  if (W >= H * 1.15) {
    const lw = W * 0.64, x1 = lw + g / 2, cw = W - x1 - g;
    drawCamera(R(g, g, lw - 1.5 * g, H - 2 * g));
    const gh = H * 0.44, th = H * 0.27;
    drawGauge(R(x1, g, cw, gh));
    (S.find ? drawFinder : drawTranscript)(R(x1, g + gh + g, cw, th));
    drawLevels(R(x1, 3 * g + gh + th, cw, H - 4 * g - gh - th));
  } else {
    const ch = Math.min(W * 0.82, H * 0.5);
    drawCamera(R(g, g, W - 2 * g, ch));
    const y = ch + 2 * g, rh = H - y - g, gw = (W - 3 * g) * 0.56, gh = rh * 0.64;
    drawGauge(R(g, y, gw, gh));
    drawLevels(R(g, y + gh + g, gw, rh - gh - g));
    (S.find ? drawFinder : drawTranscript)(R(2 * g + gw, y, W - 3 * g - gw, rh));
  }
}

// ------------------------------------------------------------------ status line

function rate(times, now) {
  while (times.length && now - times[0] > 2000) times.shift();
  return times.length > 1 ? ((times.length - 1) * 1000) / Math.max(1, now - times[0]) : 0;
}

let busyAt = performance.now();
setInterval(() => {
  const now = performance.now(), pk = rate(pktTimes, now), fps = rate(frameTimes, now), rx = rate(rxTimes, now);
  const load = (100 * busy.ms) / (now - busyAt), kbs = usbBytes / (now - busyAt);
  const perRead = usbReads ? usbBytes / usbReads : 0;
  busy.ms = 0;
  usbBytes = 0;
  usbReads = 0;
  busyAt = now;
  let s = `${APP_VERSION} · ${conn ? `${conn.name} · ${connState}` : "not connected"}`;
  if (conn && (connState === "streaming" || conn instanceof DemoConn)) {
    s += ` · ${(pk * 128 / 1000).toFixed(1)} k samples/s`;
    if (S.camOff) {
      s += boardStats.startsWith("camera off") ? " · camera off (board too)"
        : rx > 0.5 && !(conn instanceof DemoConn) ? " · camera off here; the board keeps sending it (v4 firmware turns it off)" : " · camera off";
    } else {
      s += ` · camera ${fps.toFixed(0)} fps`;
      // shown here vs arrived over USB vs what the board says it sent: tells where frames get lost
      const bs = /([\d.]+) camera fps, (\d+) frames skipped, (\d+) mic packets dropped/.exec(boardStats);
      s += ` (got ${rx.toFixed(0)}, board sent ${bs ? `${(+bs[1]).toFixed(0)}, skipped ${bs[2]}, mic drops ${bs[3]}` : "?"})`;
    }
    if (decodeErrors || decodeFails) s += ` · bad JPEG ${decodeErrors}/${decodeFails}`;
    s += ` · USB ${kbs.toFixed(0)} kB/s${perRead ? ` (${(perRead / 1024).toFixed(0)}/${((conn.size || 0) / 1024).toFixed(0)} KB per read)` : ""} · load ${load.toFixed(0)}%`;
    if (streamMask !== 0xff) {
      let k = 0;
      for (let c = 0; c < CH; c++) if (streamMask & (1 << c)) k++;
      s += ` · ${k} mics over USB`;
    } else if (maskAsks > 2) {
      s += ` · asked the board for fewer mics ${maskAsks}×, still getting all 8`;
    }
    if (gaps) s += ` · ${gaps} gaps`;
    if (pk === 0) s += " · no mic data yet";
  }
  if (boardFw) s += ` · ${boardFw.split(" · ")[0]}`;
  if (wrongFirmware) s += " · board is sending 4 mics: update it to cam_mic_vest8 (More → Update firmware)";
  if (boardMsg && !boardStats) s += ` · ${boardMsg}`;
  $("conn").textContent = s;
  $("conn").title = [boardFw, boardStats || boardMsg].filter(Boolean).join("\n");
}, 500);

// The tablet's USB link can't carry all eight mics plus the camera, so ask the board for just the mics
// the layout uses (and any data-line partner standing in for a dead one). Repeated, because the board
// goes back to all eight when the requests stop.
setInterval(() => {
  if (conn && !(conn instanceof DemoConn) && connState === "streaming" && !boardFw) conn.send(new Uint8Array([63]));  // '?': which firmware
  // Chrome's Web Serial on macOS loses 0xFF bytes (camera frames break, mic packets go missing), so ask the
  // board for a stream without any. Boards that don't know 'x' ignore it.
  if (conn instanceof SerialConn && connState === "streaming") conn.send(new Uint8Array([120]));  // 'x'
  // camera off: the board shuts it down while 'c' keeps coming (v4 firmware; older boards ignore it)
  if (conn && !(conn instanceof DemoConn) && connState === "streaming") {
    if (S.camOff) conn.send(new Uint8Array([99]));  // 'c'
    else if (boardStats.startsWith("camera off")) conn.send(new Uint8Array([67]));  // 'C'
  }
  if (!(conn instanceof UsbConn) || connState !== "streaming" || model.total < FS) return;
  let mask = 0;
  for (const ch of model.chans) mask |= 1 << ch;
  for (const [, src] of model.active()) mask |= 1 << src;
  conn.send(new Uint8Array([107, mask]));  // 'k'
  maskAsks = streamMask === 0xff ? maskAsks + 1 : 0;
}, 2000);

setInterval(() => {
  const t0 = performance.now();
  model.tick(S.thr, t0 < cap.voiceUntil);
  if (S.find) findBuzz(S.findMode === "alarm" ? alarm.process() : beacon.process(FIND_MODES[S.findMode].codes));
  else if (cal.on) {
    beacon.process(["calib"]);
    calAuto();
  }
  busy.ms += performance.now() - t0;
}, 33);

// Buzzes, so you can move without watching the screen: a tap when the leader or an exit is straight ahead,
// a double buzz when you face a danger, a long one when an alarm starts.
let buzzAt = -99, lastAlarm = -99;
function findBuzz(found) {
  if (!found.length || !navigator.vibrate) return;
  const now = model.now, ahead = (d, deg) => Math.abs(orient(d.angle)) < deg;
  if (S.findMode === "alarm") {
    if (now - lastAlarm > 15) navigator.vibrate([400, 150, 400]);
    lastAlarm = now;
  } else if (found.some((d) => d.kind === "danger" && ahead(d, 20))) {
    if (now - buzzAt > 1.5) { navigator.vibrate([150, 80, 150]); buzzAt = now; }
  } else if (found.some((d) => (d.kind === "leader" || d.kind === "exit") && ahead(d, 12))) {
    navigator.vibrate(60);
  }
}

// ------------------------------------------------------------------ controls

const hint = (t) => { $("hint").textContent = t; };

function uiSync() {
  $("cc").classList.toggle("on", cap.on);
  $("find").classList.toggle("on", S.find);
  $("findmode").classList.toggle("hidden", !S.find);
  $("findmode").value = S.findMode;
  $("focus").classList.toggle("on", S.focus);
  $("cam").classList.toggle("on", !S.camOff);
  $("cam").textContent = S.camOff ? "Camera off" : "Camera";
  $("swaplr").classList.toggle("on", S.swap);
  $("swapfb").classList.toggle("on", S.swapfb);
  $("mirror").classList.toggle("on", S.mirror);
  $("rotate").textContent = `Rotate ${S.rotate}°`;
  $("fov").value = Math.round(S.hfov);
  $("thr").value = S.thr;
  $("thrv").textContent = `${S.thr} dB`;
  $("layout").value = S.layout;
}

for (const [k, p] of Object.entries(PRESETS)) {
  const o = document.createElement("option");
  o.value = k;
  o.textContent = p.name;
  $("layout").appendChild(o);
}

function lastAngle() {
  const d = model.last;
  if (!d || model.now - d.t > 5) {
    hint("No recent sound: clap first, then press the button within 5 s.");
    return null;
  }
  return orient(d.angle);
}

// the quick buttons replace a calibration: its points no longer match
function dropCal(why) {
  if (!S.cal.length) return "";
  S.cal = [];
  Object.assign(S, { roll: 0, kel: 1, camX: 0, camZ: 0 });
  calShow();
  return ` (${why}: calibration points cleared)`;
}
$("centre").onclick = () => {
  const a = lastAngle();
  if (a === null) return;
  const dropped = dropCal("Set centre");
  S.offset = a;
  const el = model.last.elev;
  if (el !== null && el !== undefined) S.tilt = el;
  save();
  hint(`Centre set: picture centre = ${a >= 0 ? "+" : ""}${a.toFixed(0)}° on the array` +
       (el !== null && el !== undefined ? `, ${el.toFixed(0)}° up.` : ".") + dropped);
};
$("edge").onclick = () => {
  const a = lastAngle();
  if (a === null) return;
  const half = wrap(S.offset - a);
  if (half < 5 || half > 70) {
    hint(`That clap was ${half.toFixed(0)}° from centre; clap at the RIGHT edge, or try Swap L/R.`);
    return;
  }
  const dropped = dropCal("Right edge");
  S.hfov = 2 * half;
  save();
  uiSync();
  hint(`Field of view set to ${S.hfov.toFixed(0)}°.${dropped}`);
};

// ---- calibration controls
for (const [i, [name]] of CAL_TARGETS.entries()) {
  const o = document.createElement("option");
  o.value = i;
  o.textContent = name;
  $("caltarget").appendChild(o);
}
const median = (v) => {
  const s = v.slice().sort((a, b) => a - b), m = s.length >> 1;
  return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2;
};
function calShow(res = null) {
  $("calbar").classList.toggle("open", cal.on);
  $("calib").classList.toggle("on", cal.on);
  $("calguide").classList.toggle("on", cal.guide);
  $("calfine").classList.toggle("on", S.calFine);
  $("caltarget").value = cal.custom ? "" : cal.target;
  $("caldist").value = String(S.calDist);
  $("usualdist").value = S.dist;
  const n = S.cal.length;
  if (!n) { $("calinfo").textContent = "no points yet"; return; }
  if (!res) {
    const errs = S.cal.filter((p) => p.err !== undefined).map((p) => p.err);
    res = { rms: errs.length ? Math.sqrt(errs.reduce((s, e) => s + e * e, 0) / errs.length) : 0 };
  }
  const dists = new Set(S.cal.map((p) => p.dist)).size;
  const cm = (v, pos, neg) => `${Math.abs(v / 10).toFixed(0)} cm ${v >= 0 ? pos : neg}`;
  $("calinfo").textContent = `${n} point${n === 1 ? "" : "s"} at ${dists} distance${dists === 1 ? "" : "s"} · off by ${res.rms.toFixed(1)}° (rms)` +
    ` · view ${S.hfov.toFixed(0)}° · aim ${sideText(S.offset)}, ${S.tilt >= 0 ? "+" : ""}${S.tilt.toFixed(0)}° up · roll ${S.roll.toFixed(1)}°` +
    (S.kel !== 1 ? ` · height ×${S.kel.toFixed(2)}` : "") +
    (dists > 1 ? ` · camera ${cm(S.camX, "left", "right")}, ${cm(S.camZ, "up", "down")} of the mics` : "");
}
function setCam(off) {
  S.camOff = off;
  if (off) {
    if (cal.on) setCal(false);
    image?.close?.();
    image = null;
    pendingJpeg = null;
  }
  if (conn && connState === "streaming") conn.send(new Uint8Array([off ? 99 : 67])).catch(() => {});  // 'c' / 'C'
  hint(off ? "Camera off: the tablet stops decoding the picture (the board shuts its camera down with v4 firmware)"
    : "Camera on");
  save();
  uiSync();
}
function setCal(on) {
  cal.on = on;
  if (on) {
    if (S.camOff) setCam(false);  // calibrating needs the picture
    if (S.find) setFind(false);
    beacon.reset();
    model.dets = [];
    model.last = null;
    cal.since = model.now;
    cal.lastDir = null;
    if (cal.guide) guideGo(0, `On a phone open ${beaconUrl("calib")}, turn the volume up and tap Start.` +
                              (S.cal.length ? " (Earlier points are kept; Clear all to start fresh.)" : ""));
    else hint(`On a phone open ${beaconUrl("calib")} and tap Start. Put its speaker on the cyan cross, wait for chirps, ` +
              "then Capture. Phone not exactly on the cross? Tap where it is in the picture first.");
  } else {
    model.dets = [];
    hint(S.cal.length ? "Calibration saved." : "Calibrate: put a phone playing the calibration chirp on spots in the picture");
  }
  calShow();
}
// The guide's next spot (or the end), told on screen and in the hint line.
function guideGo(step, before = "") {
  cal.step = step;
  cal.custom = null;
  cal.offSince = null;
  cal.status = "";
  const lead = before ? before + " " : "";
  if (step >= CAL_GUIDE.length) {
    const errs = S.cal.filter((p) => p.err !== undefined).map((p) => p.err);
    const rms = Math.sqrt(errs.reduce((s, e) => s + e * e, 0) / Math.max(1, errs.length));
    hint(`${lead}Calibration done: sound marks land within ${rms.toFixed(1)}° of the picture on average ` +
         `(worst ${Math.max(0, ...errs).toFixed(1)}°). Tap Done, or Guide to go round again.`);
    calShow();
    return;
  }
  const [t, d] = CAL_GUIDE[step], back = step > 0 && d !== S.calDist;
  cal.target = t;
  S.calDist = d;
  save();
  hint(`${lead}Step ${step + 1} of ${CAL_GUIDE.length}: ${back ? `step back to ${d} m, then ` : ""}put the phone's ` +
       `speaker on the cyan cross (${CAL_TARGETS[t][0].toLowerCase()}), ${d} m from the vest, facing it. ` +
       "Hold it still; it captures by itself.");
  calShow();
}
// Where the phone should go to bring its sound onto the cross: dx, dy = where the sound lands minus the cross.
function moveWords(dx, dy) {
  const w = [];
  if (Math.abs(dx) > 0.1) w.push(dx > 0 ? "left" : "right");
  if (Math.abs(dy) > 0.1) w.push(dy > 0 ? "down" : "up");
  return w.join(" and ") || "a little";
}
// The guide, every tick: capture once the last few chirps agree, and say what's in the way until then.
function calAuto() {
  if (!cal.guide || cal.step >= CAL_GUIDE.length) return;
  if (model.now < cal.since) { cal.status = "moving to this spot…"; return; }
  const hits = calRecent().slice(-CAL_AUTO_N);
  if (hits.length < CAL_AUTO_N) {
    cal.status = hits.length ? `listening… ${hits.length} of ${CAL_AUTO_N} chirps` : "waiting for chirps: tap Start on the phone";
    cal.offSince = null;
    return;
  }
  const ref = hits[hits.length - 1].angle, rel = hits.map((d) => wrap(d.angle - ref)), mid = median(rel);
  const els = hits.map((d) => d.elev).filter((v) => v !== null && v !== undefined), em = els.length ? median(els) : null;
  const spread = median(rel.map((v) => Math.abs(v - mid))), espread = em === null ? 0 : median(els.map((v) => Math.abs(v - em)));
  if (spread > CAL_STEADY || espread > CAL_STEADY) {
    cal.status = "hold the phone still";
    cal.offSince = null;
    return;
  }
  const ang = wrap(ref + mid);
  if (cal.lastDir && Math.hypot(wrap(ang - cal.lastDir[0]), em === null || cal.lastDir[1] === null ? 0 : em - cal.lastDir[1]) < 3) {
    cal.status = "still at the last spot: move the phone onto the cross";
    return;
  }
  if (S.cal.length >= 3) {  // the fit so far says where this sound lands: off the cross means the phone is
    const oa = calAngle({ ang }, S.swap), q = camProj(oa, em, S.calDist), [, , X, Y] = calSpot();
    if (q) {
      const f = calFix(oa, em), dx = q[0] + f[0] - X, dy = q[1] + f[1] - Y;
      if (Math.hypot(dx, dy) > CAL_OFF) {
        cal.offSince ??= model.now;
        if (model.now - cal.offSince < CAL_FORCE_S) {
          cal.status = `sound is off the cross: move the phone ${moveWords(dx, dy)}`;
          return;
        }
      }
    }
  }
  cal.offSince = null;
  calCapture();
}
function calCapture() {
  const hits = calRecent();
  if (hits.length < 4) {
    hint(`Only ${hits.length} chirp${hits.length === 1 ? "" : "s"} heard in the last ${CAL_HITS_S} s: is the phone on ` +
         "the Calibrate sound with the volume up? Closer, or lower the Trigger in More.");
    return;
  }
  const ref = hits[hits.length - 1].angle, rel = hits.map((d) => wrap(d.angle - ref));
  let mid = median(rel);
  const els = hits.map((d) => d.elev).filter((v) => v !== null && v !== undefined);
  const spread = median(rel.map((v) => Math.abs(v - mid)));
  // finer than the 1° grid: the chirps near the median, their curves summed, parabola through the peak
  const sum = new Float64Array(360);
  for (const [i, d] of hits.entries()) if (Math.abs(rel[i] - mid) <= Math.max(2, 2 * spread)) for (let k = 0; k < 360; k++) sum[k] += d.curve[k];
  const k0 = (((Math.round(ref + mid) + 180) % 360) + 360) % 360, y0 = sum[(k0 + 359) % 360], y1 = sum[k0], y2 = sum[(k0 + 1) % 360];
  const den = y0 - 2 * y1 + y2;
  if (den < 0 && y1 >= y0 && y1 >= y2) mid = wrap(k0 - 180 + (0.5 * (y0 - y2)) / den - ref);
  const name = cal.custom ? "the tapped spot" : CAL_TARGETS[cal.target][0];
  const [fx, fy] = cal.custom || CAL_TARGETS[cal.target].slice(1), pic = cal.pic;
  const aspect = pic ? pic.h / pic.w : 0.75;
  S.cal.push({ ang: wrap(ref + mid), e: els.length ? median(els) : null, dist: S.calDist, fx, fy,
               X: (fx - 0.5) * 2, Y: (0.5 - fy) * 2 * aspect, n: hits.length, spread });
  cal.since = model.now;
  const res = calFit(), p = S.cal[S.cal.length - 1];
  let msg = `Captured ${name} at ${S.calDist} m: ${hits.length} chirps, ±${spread.toFixed(1)}° scatter; now off by ${p.err.toFixed(1)}° there.`;
  if (spread > 4) msg += " That scatter is high: quieter room, phone louder or closer, then Undo and capture again.";
  if (res.flipped) msg += " Left and right were the wrong way round: Swap L/R turned on.";
  if (cal.guide) {
    cal.lastDir = [p.ang, p.e];
    cal.since = model.now + CAL_MOVE_S;
    guideGo(cal.step + 1, `✓ ${msg}`);
    uiSync();
    return;
  }
  // walk the five main spots, then suggest another distance (that's what pins down where the camera sits)
  if (!cal.custom) {
    cal.target = (cal.target + 1) % 5;
    msg += cal.target === 0 ? " Five spots done: move the phone to another distance and go round again." : ` Next: ${CAL_TARGETS[cal.target][0]}.`;
  }
  cal.custom = null;
  hint(msg);
  calShow(res);
  uiSync();
}
$("calib").onclick = () => setCal(!cal.on);
$("cam").onclick = () => setCam(!S.camOff);
$("caldone").onclick = () => setCal(false);
$("calgrab").onclick = calCapture;
$("calguide").onclick = () => {
  cal.guide = !cal.guide;
  cal.since = model.now;
  cal.lastDir = null;
  if (cal.guide) guideGo(0, "Guide started from the first spot.");
  else {
    hint("Guide off: pick a Spot, put the phone there and tap Capture.");
    calShow();
  }
};
$("calskip").onclick = () => {
  cal.since = model.now;
  if (cal.guide) {
    if (cal.step < CAL_GUIDE.length) guideGo(cal.step + 1, "Skipped.");
    return;
  }
  cal.target = (cal.target + 1) % CAL_TARGETS.length;
  cal.custom = null;
  calShow();
};
$("caltarget").onchange = (e) => {
  cal.target = +e.target.value;
  cal.custom = null;
  cal.since = model.now;
  if (cal.guide) {
    cal.guide = false;
    hint(`Guide off: put the phone on ${CAL_TARGETS[cal.target][0].toLowerCase()} and tap Capture.`);
  }
  calShow();
};
$("caldist").onchange = (e) => { S.calDist = +e.target.value; cal.since = model.now; save(); };
$("usualdist").onchange = (e) => { S.dist = Math.max(0.3, Math.min(10, +e.target.value || 1.5)); save(); calShow(); };
$("calfine").onclick = () => { S.calFine = !S.calFine; save(); calShow(); };
$("calundo").onclick = () => {
  if (!S.cal.length) return;
  S.cal.pop();
  const res = calFit();
  save();
  calShow(res);
  hint(S.cal.length ? "Last point removed." : "No points left.");
};
$("calclear").onclick = () => {
  if (S.cal.length && !confirm(`Delete all ${S.cal.length} calibration points?`)) return;
  dropCal("Clear all");
  save();
  hint("Calibration points cleared; the view and aim stay as they were.");
};
// tap the picture: the phone is there (when it isn't exactly on the cross)
cv.addEventListener("pointerdown", (e) => {
  if (!cal.on || !cal.pic) return;
  const r = cv.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top, pic = cal.pic;
  if (x < pic.x || x > pic.r || y < pic.y || y > pic.b) return;
  cal.custom = [(x - pic.x) / pic.w, (y - pic.y) / pic.h];
  cal.since = model.now;
  calShow();
  hint("Spot moved to where you tapped. Capture when the chirps are coming from there.");
});
const refit = () => { if (S.cal.length) calShow(calFit()); };
$("swaplr").onclick = () => { S.swap = !S.swap; refit(); save(); uiSync(); };
$("swapfb").onclick = () => { S.swapfb = !S.swapfb; refit(); save(); uiSync(); };
$("mirror").onclick = () => { S.mirror = !S.mirror; hint("Mirrored." + dropCal("Mirror")); save(); uiSync(); };
$("rotate").onclick = () => { S.rotate = (S.rotate + 90) % 360; hint(`Rotated ${S.rotate}°.` + dropCal("Rotate")); save(); uiSync(); };
$("fov").onchange = (e) => { S.hfov = Math.max(20, Math.min(140, +e.target.value || 60)); save(); uiSync(); };
$("thr").oninput = (e) => { S.thr = +e.target.value; save(); uiSync(); };
$("layout").onchange = (e) => { S.layout = e.target.value; model.setGeom(PRESETS[S.layout].geom); save(); };
$("res").onchange = (e) => conn?.send(new TextEncoder().encode(e.target.value));
$("morebtn").onclick = () => { $("more").classList.toggle("open"); $("morebtn").classList.toggle("on"); setTimeout(resize, 0); };
$("rec").onclick = () => {
  const on = !$("rec").classList.contains("on");
  // 'v' = mics + the camera's picture, 'a' = mics only (the camera is off, so there's no picture to keep)
  conn?.send(new TextEncoder().encode(on ? (S.camOff ? "a" : "v") : "s"));
  hint(on ? (S.camOff ? "Recording the mics to the SD card (camera off: no video)." : "Recording mics + video to the SD card.")
          : "Stopped: the take is on the SD card.");
  $("rec").classList.toggle("on", on);
  $("rec").textContent = on ? "■ Stop SD" : "● Rec SD";
};
// Captions off for Find mode: drop both speech models so their memory and CPU go back to the tablet
// (they come back from this device's cache when CC is turned on again)
function asrUnload() {
  try { asr.model?.terminate?.(); } catch (e) { /* already gone */ }
  asr.model = null;
  asr.rec = null;
  asr.n = 0;
  hqUnload();
}

$("stt").onchange = (e) => {
  S.stt = e.target.value;
  save();
  hqUnload();
  if (asr.rec) hqLoad();
  const k = S.stt in STT ? S.stt : null;
  hint(k ? `Speech: ${STT[k]}.` + (k === "better" || k === "best" ? " It downloads the first time, then stays on this device." : "")
    : "Speech: Auto picks for this device.");
};
sttAuto.then(([k, why]) => {
  const box = $("stt");
  box.innerHTML = "";
  for (const [v, label] of [["auto", `Auto: ${STT[k].split(":")[0]} (${why})`], ...Object.entries(STT)]) box.add(new Option(label, v));
  box.value = S.stt in STT ? S.stt : "auto";
});

function setCC(on) {
  cap.on = on;
  S.cc = on;
  cap.interim = null;
  if (on) asrLoad();
}

function setFind(on) {
  S.find = on;
  model.dets = [];
  model.last = null;
  beacon.reset();
  alarm.reset();
  if (on && cap.on) { setCC(false); asrUnload(); }
  const hints = {
    follow: "Follow: the leader plays the Leader tone (beacon page); follow the green arrow, it buzzes when they're straight ahead",
    alarm: "Alarm: shows which way a beeping or steady alarm is; long buzz when one starts",
    evac: "Evacuate: phones play Exit or Danger tones (beacon page); go where the green arrow points, away from red",
  };
  if (on && cal.on) setCal(false);
  hint(on ? hints[S.findMode] : "Calibrate: put a phone playing the calibration chirp on spots in the picture");
}

$("cc").onclick = () => {
  if (!cap.on && S.find) setFind(false);
  setCC(!cap.on);
  if (!cap.on) asrUnload();
  save();
  uiSync();
};
$("find").onclick = () => { setFind(!S.find); save(); uiSync(); };
$("findmode").onchange = (e) => { S.findMode = e.target.value; setFind(true); save(); uiSync(); };
$("focus").onclick = () => { S.focus = !S.focus; save(); uiSync(); };

async function connect(c) {
  const why = $("why");
  why.className = "";
  why.textContent = "";
  try {
    await c.open();
  } catch (e) {
    if (e.name !== "NotFoundError") {
      why.className = "err";
      why.textContent = `Couldn't open the board: ${e.message}`;
    }
    return;
  }
  conn = c;
  wrongFirmware = false;
  boardFw = "";
  fwLayoutDone = false;
  lastSeq = null;
  gaps = 0;
  parser.reset();
  $("connect").classList.add("hidden");
  focus = null;
  await c.run((data) => {
    const t0 = performance.now();
    usbBytes += data.length;
    parser.feed(data);
    busy.ms += performance.now() - t0;
  }, (s) => { connState = s; });
  if (conn === c) {
    conn = null;
    $("connect").classList.remove("hidden");
  }
}

const android = /Android/i.test(navigator.userAgent);
function usbConn() {
  if (android && "usb" in navigator) return new UsbConn();
  if ("serial" in navigator) return new SerialConn();
  if ("usb" in navigator) return new UsbConn();
  return null;
}

$("go").onclick = () => {
  const c = usbConn();
  if (!c) {
    $("why").className = "err";
    $("why").textContent = window.isSecureContext
      ? "This browser can't open USB devices. Use Chrome (Android tablet/phone, or desktop)."
      : "USB needs a secure page: open this from its https:// address (or localhost).";
    return;
  }
  connect(c);
};
$("demo").onclick = () => connect(new DemoConn());
$("disconnect").onclick = async () => {
  const c = conn;
  conn = null;
  await c?.close();
  image = null;
  $("connect").classList.remove("hidden");
};

const params = new URLSearchParams(location.search);
if (!("serial" in navigator) && !("usb" in navigator)) {
  $("why").textContent = window.isSecureContext
    ? "This browser has no USB access: use Chrome. Demo works anywhere."
    : "Open this page from its https:// address for USB. Demo works anywhere.";
}
if (S.find) setFind(true);
uiSync();
calShow();
resize();
requestAnimationFrame(render);
if (cap.on) asrLoad();
if ("serviceWorker" in navigator && window.isSecureContext) navigator.serviceWorker.register("sw.js").catch(() => {});
if (params.get("ws")) connect(new WsConn(params.get("ws")));
else if (params.has("demo")) connect(new DemoConn());
