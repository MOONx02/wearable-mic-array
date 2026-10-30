// Mic array 360 in the browser: reads the ESP32-S3 (cam_mic_record8) USB stream, finds the direction of
// sounds with SRP-PHAT over every mic pair (same maths as mic_camera_view360.py), draws the camera
// picture with a glow / edge arrows, a 360° radar, mic levels, and live subtitles from the browser's
// speech recognition placed by the direction of the talker.
"use strict";

const FS = 16000, CH = 8, C = 343, SEG = 1024, NFFT = 2048, INTERP = 8;
const BAND = [300, 6000], HOLD_S = 2.5, HISTORY_S = 10, BEHIND_DEG = 135, FB_SURE = 0.25;
const DEAD_PTP = 8, STUCK_FRACTION = 0.5, CAPTION_HOLD_S = 9;
const VIDS = [0x303a, 0x2886];
const COLORS = ["#58a6ff", "#f0883e", "#3fb950", "#d2a8ff", "#ff7b72", "#e3b341", "#79c0ff", "#56d4dd"];
const BG = "#0d1117", PANEL = "#161b22", EDGE = "#30363d", INK = "#e6edf3", DIM = "#8b949e";
const GLOW = "#ffa657", LOUD = "#ff7b72", SPEAKER = "#58a6ff";

// channel: [name, x mm toward the wearer's left, y mm toward the front]
const FRONT = { 0: ["front outer L", 68.6, 22], 1: ["front outer R", -68.6, 22],
                2: ["front inner L", 22.9, 22], 3: ["front inner R", -22.9, 22] };
const PRESETS = {
  rear5: { name: "5 mics: 4 front + rear centre", geom: { ...FRONT, 4: ["rear centre", 0, -22] } },
  six: { name: "6 mics: 4 front + 2 back", geom: { ...FRONT, 5: ["back R", -68.6, -22], 7: ["back L", 68.6, -22] } },
  front4: { name: "4 front mics (no front/back)", geom: { ...FRONT } },
};

const S = Object.assign({ layout: "rear5", hfov: 60, offset: 0, swap: false, swapfb: false, rotate: 0,
                          mirror: false, thr: 12, cc: true }, JSON.parse(localStorage.getItem("mic360") || "{}"));
if (!PRESETS[S.layout]) S.layout = "rear5";
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
    this.wp = 0;
    this.total = 0;
    this.dc = new Float64Array(CH);
    this.level = new Float64Array(CH).fill(-120);
    this.floor = new Float64Array(CH).fill(-70);
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
      const d = this.dc[c], buf = this.x[c];
      let w = this.wp;
      for (let i = 0; i < frames; i++) {
        buf[w] = s[i * CH + c] - d;
        if (++w === this.N) w = 0;
      }
    }
    this.wp = (this.wp + frames) % this.N;
    this.total += frames;
  }
  tail(c, n, out) {
    const buf = this.x[c];
    let r = (this.wp - n + this.N) % this.N;
    for (let i = 0; i < n; i++) {
      out[i] = buf[r];
      if (++r === this.N) r = 0;
    }
    return out;
  }

  tick(thr) {
    if (this.total < FS / 10) return;
    for (let c = 0; c < CH; c++) {
      const blk = this.tail(c, 800, this.blk);
      let m = 0;
      for (let i = 0; i < 800; i++) m += blk[i];
      m /= 800;
      let v = 0;
      for (let i = 0; i < 800; i++) v += (blk[i] - m) * (blk[i] - m);
      const lv = 10 * Math.log10(v / 800 / (32768 * 32768) + 1e-12);
      this.level[c] = lv;
      if (this.total < FS / 2) {
        this.floor[c] = this.total > FS / 10 ? Math.min(this.floor[c], lv) : lv;
        continue;
      }
      this.floor[c] = lv < this.floor[c] ? 0.7 * this.floor[c] + 0.3 * lv : this.floor[c] + 0.02;
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
    this.direction(thr);
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

  steering(chans) {
    const key = chans.join(",");
    let st = this.steer.get(key);
    if (st) return st;
    const pos = chans.map((c) => [this.geom[c][1] / 1000, this.geom[c][2] / 1000]);
    const xs = pos.map((p) => p[0]), ys = pos.map((p) => p[1]);
    const ptpx = Math.max(...xs) - Math.min(...xs), ptpy = Math.max(...ys) - Math.min(...ys);
    const m = (Math.ceil((Math.hypot(ptpx, ptpy) / C) * FS) + 2) * INTERP;
    const t = pos.map(([x, y]) => {
      const a = new Float64Array(360);
      for (let k = 0; k < 360; k++) {
        const th = ((k - 180) * Math.PI) / 180;
        a[k] = -(x * Math.sin(th) + y * Math.cos(th)) / C;
      }
      return a;
    });
    const pairs = [];
    for (let i = 0; i < chans.length; i++) {
      for (let j = i + 1; j < chans.length; j++) {
        const idx = new Int32Array(360);
        for (let k = 0; k < 360; k++) {
          idx[k] = Math.min(2 * m, Math.max(0, Math.round((t[j][k] - t[i][k]) * FS * INTERP) + m));
        }
        pairs.push([i, j, idx]);
      }
    }
    // GCC-PHAT is only needed at the lags the array can produce, so evaluate those directly instead of
    // an 8x zero-padded inverse FFT per pair.
    const k0 = Math.ceil((BAND[0] * NFFT) / FS), k1 = Math.floor((BAND[1] * NFFT) / FS);
    const nb = k1 - k0 + 1, L = 2 * m + 1, N = NFFT * INTERP;
    const cosT = new Float32Array(L * nb), sinT = new Float32Array(L * nb);
    for (let l = 0; l < L; l++) {
      for (let b = 0; b < nb; b++) {
        const ph = (2 * Math.PI * (k0 + b) * (l - m)) / N;
        cosT[l * nb + b] = Math.cos(ph);
        sinT[l * nb + b] = Math.sin(ph);
      }
    }
    st = { pairs, m, L, k0, nb, cosT, sinT, hasBack: ptpy > 0.01 };
    this.steer.set(key, st);
    return st;
  }

  srp(srcs, st) {
    const re = [], im = [];
    for (const s of srcs) {
      const r = new Float64Array(NFFT), i = new Float64Array(NFFT);
      const seg = this.tail(s, SEG, this.seg);
      for (let n = 0; n < SEG; n++) r[n] = seg[n];
      fft(r, i);
      re.push(r);
      im.push(i);
    }
    const curve = new Float64Array(360), cc = new Float64Array(st.L);
    const gr = new Float64Array(st.nb), gi = new Float64Array(st.nb);
    for (const [i, j, idx] of st.pairs) {
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
      for (let k = 0; k < 360; k++) curve[k] += cc[idx[k]];
    }
    return curve;
  }

  direction(thr) {
    const act = this.active();
    if (this.total < FS / 2 || act.length < 2) return;
    const srcs = act.map((a) => a[1]);
    let strength = -Infinity;
    for (const s of srcs) strength = Math.max(strength, this.level[s] - this.floor[s]);
    if (strength < thr) return;
    if (this.last && this.now - this.last.t < 0.08) return;
    const st = this.steering(act.map((a) => a[0]));
    const curve = this.srp(srcs, st);
    let k = 0;
    for (let i = 1; i < 360; i++) if (curve[i] > curve[k]) k = i;
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
    const det = { t: this.now, angle, sigma, fb, strength, curve, n: act.length };
    this.dets.push(det);
    this.last = det;
    while (this.dets.length && this.now - this.dets[0].t > HISTORY_S) this.dets.shift();
  }
}

// ------------------------------------------------------------------ packet parser (same as the USB protocol)

class Parser {
  constructor(on) {
    this.on = on;
    this.buf = new Uint8Array(0);
    this.td = new TextDecoder();
  }
  feed(chunk) {
    let b = chunk;
    if (this.buf.length) {
      b = new Uint8Array(this.buf.length + chunk.length);
      b.set(this.buf);
      b.set(chunk, this.buf.length);
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
        const s = new Int16Array(frames * CH);
        for (let i = 0, o = p + 10; i < s.length; i++, o += 2) s[i] = dv.getInt16(o, true);
        this.on.mic(seq, frames, s);
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
        this.on.status(this.td.decode(b.subarray(p + 6, p + 6 + len)));
        p += 6 + len;
      } else {
        p += 2;
      }
    }
    this.buf = p < n ? b.slice(p) : new Uint8Array(0);
    if (this.buf.length > 1 << 20) this.buf = new Uint8Array(0);
  }
}

// ------------------------------------------------------------------ connections

class SerialConn {  // Web Serial (desktop Chrome / Edge, newer Android Chrome)
  constructor() { this.name = "USB serial"; }
  async open() {
    this.port = await navigator.serial.requestPort({ filters: VIDS.map((v) => ({ usbVendorId: v })) });
    await this.port.open({ baudRate: 115200, bufferSize: 1 << 16 });
  }
  async reopen() {
    for (;;) {
      await sleep(700);
      if (this.closed) return false;
      try {
        const ports = await navigator.serial.getPorts();
        const p = ports.find((x) => VIDS.includes(x.getInfo().usbVendorId)) || this.port;
        try { await p.close(); } catch (e) { /* not open */ }
        await p.open({ baudRate: 115200, bufferSize: 1 << 16 });
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
  async send(bytes) {
    if (!this.port?.writable) return;
    const w = this.port.writable.getWriter();
    try { await w.write(bytes); } finally { w.releaseLock(); }
  }
  async close() {
    this.closed = true;
    try { await this.reader?.cancel(); } catch (e) { /* ignore */ }
    try { await this.port?.close(); } catch (e) { /* ignore */ }
  }
}

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
      try {
        for (let i = 0; i < 4; i++) q.push(this.dev.transferIn(ep, 16384));
        for (;;) {
          const r = await q.shift();
          if (this.closed) return;
          q.push(this.dev.transferIn(ep, 16384));
          if (r.status === "ok" && r.data && r.data.byteLength) {
            onData(new Uint8Array(r.data.buffer, r.data.byteOffset, r.data.byteLength));
          } else if (r.status === "stall") {
            await this.dev.clearHalt("in", ep);
          }
        }
      } catch (e) {
        if (this.closed) return;
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

class DemoConn {  // no board: claps from all round the array, through the current layout
  constructor() { this.name = "demo"; this.angles = [0, 40, -25, 120, -150, 180, 75, -100, 15, 160]; }
  async open() {}
  clap(ang) {
    const n = NFFT, re = new Float64Array(n), im = new Float64Array(n);
    for (let i = 200; i < n - 200; i++) re[i] = gauss() * Math.exp(-(i - 200) / 250) * Math.min(1, (i - 200) / 40);
    fft(re, im);
    const u = [Math.sin((ang * Math.PI) / 180), Math.cos((ang * Math.PI) / 180)];
    const out = {};
    let peak = 1e-9;
    for (const [ch, [, x, y]] of Object.entries(model.geom)) {
      const delay = -((x / 1000) * u[0] + (y / 1000) * u[1]) / C;
      const r = new Float64Array(n), i = new Float64Array(n);
      for (let k = 0; k < n; k++) {
        const f = (k <= n / 2 ? k : k - n) * (FS / n), ph = -2 * Math.PI * f * delay;
        const c = Math.cos(ph), s = Math.sin(ph);
        r[k] = re[k] * c - im[k] * s;
        i[k] = -(re[k] * s + im[k] * c);  // conjugate, so the forward FFT acts as an inverse
      }
      fft(r, i);
      r.fill(0, 0, 190);  // fractional-delay ringing ahead of the onset; a real clap has none
      for (let k = 0; k < n; k++) peak = Math.max(peak, Math.abs(r[k]));
      out[ch] = r;
    }
    for (const ch in out) out[ch] = out[ch].map((v) => (v / peak) * 12000);
    return out;
  }
  run(onData, onState) {
    onState("demo (no board)");
    let seq = 0, k = 0, pending = null, pos = 0, nextClap = performance.now() + 1000;
    return new Promise((resolve) => {
      this.timer = setInterval(() => {
        if (this.closed) { clearInterval(this.timer); return resolve(); }
        const frames = 128, pkt = new Uint8Array(10 + frames * CH * 2), dv = new DataView(pkt.buffer);
        pkt.set([0xa5, 0x5a, 77, 56]);
        dv.setUint32(4, seq++, true);
        dv.setUint16(8, frames, true);
        const now = performance.now();
        if (now >= nextClap) {
          pending = this.clap(this.angles[k++ % this.angles.length]);
          pos = 0;
          nextClap = now + 1400;
        }
        for (let f = 0; f < frames; f++) {
          for (let c = 0; c < CH; c++) {
            let v = c in model.geom ? gauss() * 25 : -30935;
            if (pending && c in pending && pos + f < NFFT) v += pending[c][pos + f];
            dv.setInt16(10 + (f * CH + c) * 2, Math.max(-32768, Math.min(32767, Math.round(v))), true);
          }
        }
        pos += frames;
        if (pos >= NFFT) pending = null;
        onData(pkt);
      }, 8);
    });
  }
  async send() {}
  async close() { this.closed = true; }
}

function gauss() {
  return Math.sqrt(-2 * Math.log(Math.random() + 1e-12)) * Math.cos(2 * Math.PI * Math.random());
}

// ------------------------------------------------------------------ app state

const model = new Model();
let conn = null, connState = "", wrongFirmware = false, boardStats = "", boardMsg = "";
let image = null, decoding = false, lastSeq = null, gaps = 0;
const pktTimes = [], frameTimes = [];

const parser = new Parser({
  mic(seq, frames, s) {
    if (lastSeq !== null && seq !== ((lastSeq + 1) >>> 0)) gaps++;
    lastSeq = seq;
    model.push(s, frames);
    pktTimes.push(performance.now());
  },
  wrong() { wrongFirmware = true; },
  jpeg(bytes) {
    if (decoding) return;
    decoding = true;
    createImageBitmap(new Blob([bytes], { type: "image/jpeg" }))
      .then((bm) => { image?.close?.(); image = bm; frameTimes.push(performance.now()); })
      .catch(() => {})
      .finally(() => { decoding = false; });
  },
  status(msg) {
    if (msg.startsWith("stats: ")) boardStats = msg.slice(7);
    else boardMsg = msg;
  },
});

const orient = (a) => {
  const b = S.swap ? -a : a;
  return S.swapfb ? wrap(180 - b) : wrap(b);
};
const sideText = (rel) => Math.abs(rel) < 2 ? "straight ahead" : Math.abs(rel) > 178 ? "directly behind"
  : `${Math.abs(rel).toFixed(0)}° ${rel > 0 ? "left" : "right"}`;
const fbText = (d) => d.fb === null ? "front/back unknown (no back mic)" : d.fb >= FB_SURE ? "front/back sure" : "front/back unsure";

// ------------------------------------------------------------------ captions (browser speech recognition)

const Recognition = window.SpeechRecognition || window.webkitSpeechRecognition;
const cap = { on: S.cc, lines: [], interim: null, rec: null, failed: "", lastFinal: "" };

function speechAngle() {
  const d = model.last;
  return d && model.now - d.t < 2.5 ? d.angle : null;
}

function addLine(text, angle) {
  const now = performance.now() / 1000, last = cap.lines[cap.lines.length - 1];
  if (last && now - last.wall < 2.5 && last.text.length + text.length < 160) {
    last.text += " " + text;
    last.wall = now;
    if (angle !== null) last.angle = angle;
  } else {
    cap.lines.push({ wall: now, text, angle });
    if (cap.lines.length > 60) cap.lines.shift();
  }
}

function capStart() {
  if (!Recognition) {
    cap.failed = "no speech recognition in this browser: use Chrome, or turn on Live Caption in Android settings";
    return;
  }
  if (cap.rec) return;
  const r = new Recognition();
  r.continuous = true;
  r.interimResults = true;
  r.lang = navigator.language || "en-US";
  r.onresult = (e) => {
    let fin = "", intr = "";
    for (let i = e.resultIndex; i < e.results.length; i++) {
      const t = e.results[i][0].transcript;
      if (e.results[i].isFinal) fin += t; else intr += t;
    }
    const a = speechAngle();
    fin = fin.trim();
    // Android Chrome repeats earlier results in continuous mode; keep only what's new
    if (fin && cap.lastFinal && fin.startsWith(cap.lastFinal)) fin = fin.slice(cap.lastFinal.length).trim();
    if (fin) { addLine(fin, a); cap.lastFinal = (cap.lastFinal + " " + fin).trim().slice(-400); }
    intr = intr.trim();
    cap.interim = intr ? { text: intr, angle: a } : null;
  };
  r.onerror = (e) => {
    if (e.error === "not-allowed" || e.error === "service-not-allowed") {
      cap.failed = "microphone blocked for this page: allow it in Chrome's site settings, or use Live Caption";
      cap.on = false;
      uiSync();
    }
  };
  const started = performance.now();
  r.onend = () => {
    cap.rec = null;
    cap.interim = null;
    cap.lastFinal = "";
    cap.quickEnds = performance.now() - started < 1500 ? (cap.quickEnds || 0) + 1 : 0;
    if (cap.quickEnds >= 6) {
      cap.failed = "speech recognition keeps stopping here: use Chrome, or Android Live Caption";
      cap.quickEnds = 0;
      return;
    }
    if (cap.on && !cap.failed) setTimeout(capStart, 250);
  };
  try {
    r.start();
    cap.rec = r;
    cap.failed = "";
  } catch (e) {
    cap.failed = "speech recognition didn't start: " + e.message;
  }
}

function capStop() {
  try { cap.rec?.stop(); } catch (e) { /* ignore */ }
  cap.rec = null;
  cap.interim = null;
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
    const fade = 1 - (now - d.t) / HOLD_S, rel = wrap(orient(d.angle) - S.offset);
    const color = d.strength > 30 ? LOUD : GLOW, latest = d === recent[recent.length - 1];
    if (Math.abs(rel) <= half) {
      const x = xOf(rel), spread = Math.abs(xOf(rel + d.sigma) - xOf(rel - d.sigma)) / 2;
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
      if (latest) {
        const near = Math.abs(rel) + d.sigma > half;
        labelBox(x + 14, pic.y + 0.16 * pic.h, [
          `${d.strength > 30 ? "LOUD · " : ""}Sound · ${near ? "near the edge" : "in view"}`,
          `${sideText(rel)} · ±${d.sigma.toFixed(0)}° · ${d.strength.toFixed(0)} dB over quiet`,
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
  const lines = [`${sideText(rel)} · ${behind ? "behind you" : "in front"}`, `±${d.sigma.toFixed(0)}° · ${fbText(d)}`];
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

function where(angle) {
  if (angle === null) return [null, "in"];
  const rel = wrap(orient(angle) - S.offset);
  if (Math.abs(rel) <= S.hfov / 2) return [rel, "in"];
  return [rel, rel > 0 ? "left" : "right"];
}

function drawCaptions(pic, f, lift) {
  if (!cap.on) return;
  const now = performance.now() / 1000;
  const rows = cap.lines.filter((l) => now - l.wall < CAPTION_HOLD_S).slice(-2).map((l) => ({ ...l, interim: false }));
  if (cap.interim) rows.push({ ...cap.interim, interim: true });
  const show = rows.slice(-3);
  ctx.save();
  ctx.beginPath();
  ctx.rect(pic.x, pic.y, pic.w, pic.h);
  ctx.clip();
  if (cap.interim) {
    const [rel, side] = where(cap.interim.angle);
    if (rel !== null && side === "in") {
      const x = pic.cx - f * Math.tan((rel * Math.PI) / 180);
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
    const px = Math.max(16, Math.round(pic.w / 30)), width = pic.w * 0.9, lh = px * 1.25;
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
    const [, x, y] = model.geom[ch];
    const mx = S.swap ? -x : x, my = S.swapfb ? -y : y;
    ctx.beginPath();
    ctx.arc(cx - mx * scale, cy - my * scale, 4.5, 0, 2 * Math.PI);
    ctx.fillStyle = model.source(ch) !== null ? COLORS[k % COLORS.length] : "#484f58";
    ctx.fill();
  });
  const now = model.now;
  for (const d of model.dets) {
    const age = now - d.t;
    if (age > 6) continue;
    const [x, y] = polar(cx, cy, rad * 0.86, orient(d.angle));
    ctx.beginPath();
    ctx.arc(x, y, 5, 0, 2 * Math.PI);
    ctx.fillStyle = alpha(d.strength > 30 ? LOUD : GLOW, 0.6 * (1 - age / 6));
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
    ctx.fillStyle = alpha(GLOW, 0.25 * fade);
    ctx.fill();
    const [x, y] = polar(cx, cy, rad - 8, a);
    ctx.strokeStyle = alpha("#ff7b72", fade);
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
    text(`${fbText(d)} · ${d.n} mics`, r.cx, r.y + 62, 12, DIM, "center", false, true);
  } else {
    text("clap, snap or talk", r.cx, r.y + 46, 15, DIM, "center");
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
    if (src === null) {
      text("no signal", x0 + 8, y, 11, LOUD, "left", false, true);
      return;
    }
    rrect(x0, y - bh / 2, Math.max(4, xf(model.level[src]) - x0), bh, 4);
    ctx.fillStyle = color;
    ctx.fill();
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
  const state = cap.failed ? cap.failed : !Recognition ? "no speech recognition" : cap.on ? (cap.rec ? "listening" : "starting…") : "captions off";
  ctx.font = font(11);
  let st = state;
  while (st.length > 4 && ctx.measureText(st).width > r.w - 120) st = st.slice(0, -2);
  if (st !== state) st = st.trimEnd() + "…";
  text(st, r.r - 14, r.y + 18, 11, cap.failed ? LOUD : DIM, "right");
  const inner = R(r.x + 14, r.y + 34, r.w - 28, r.h - 44);
  ctx.save();
  ctx.beginPath();
  ctx.rect(inner.x, inner.y, inner.w, inner.h);
  ctx.clip();
  let y = inner.b;
  for (let i = cap.lines.length - 1; i >= 0 && y > inner.y; i--) {
    const ln = cap.lines[i];
    ctx.font = font(14);
    const lines = wrapLines(ln.text, inner.w);
    y -= lines.length * 18;
    lines.forEach((l, j) => text(l, inner.x, y + j * 18 + 9, 14, INK));
    const [rel, side] = where(ln.angle);
    const t = new Date(performance.timeOrigin + ln.wall * 1000).toLocaleTimeString();
    y -= 16;
    text(`${t}${rel === null ? "" : ` · ${sideText(rel)}${side === "in" ? "" : " (off screen)"}`}`, inner.x, y + 7, 11, DIM);
    y -= 8;
  }
  if (!cap.lines.length && cap.failed) {
    ctx.font = font(13);
    wrapLines(cap.failed, inner.w - 20).forEach((l, j) => text(l, inner.cx, inner.cy + j * 18, 13, LOUD, "center"));
  } else if (!cap.lines.length) {
    text(cap.on ? "say something" : "tap CC for subtitles", inner.cx, inner.cy, 14, DIM, "center");
  }
  ctx.restore();
}

function render() {
  ctx.fillStyle = BG;
  ctx.fillRect(0, 0, W, H);
  const g = 10;
  if (W >= H * 1.15) {
    const lw = W * 0.64, x1 = lw + g / 2, cw = W - x1 - g;
    drawCamera(R(g, g, lw - 1.5 * g, H - 2 * g));
    const gh = H * 0.44, th = H * 0.27;
    drawGauge(R(x1, g, cw, gh));
    drawTranscript(R(x1, g + gh + g, cw, th));
    drawLevels(R(x1, 3 * g + gh + th, cw, H - 4 * g - gh - th));
  } else {
    const ch = Math.min(W * 0.82, H * 0.5);
    drawCamera(R(g, g, W - 2 * g, ch));
    const y = ch + 2 * g, rh = H - y - g, gw = (W - 3 * g) * 0.56, gh = rh * 0.64;
    drawGauge(R(g, y, gw, gh));
    drawLevels(R(g, y + gh + g, gw, rh - gh - g));
    drawTranscript(R(2 * g + gw, y, W - 3 * g - gw, rh));
  }
  requestAnimationFrame(render);
}

// ------------------------------------------------------------------ status line

function rate(times, now) {
  while (times.length && now - times[0] > 2000) times.shift();
  return times.length > 1 ? ((times.length - 1) * 1000) / Math.max(1, now - times[0]) : 0;
}

setInterval(() => {
  const now = performance.now(), pk = rate(pktTimes, now), fps = rate(frameTimes, now);
  let s = conn ? `${conn.name} · ${connState}` : "not connected";
  if (conn && connState === "streaming") {
    s += ` · ${(pk * 128 / 1000).toFixed(1)} k samples/s · camera ${fps.toFixed(0)} fps`;
    if (gaps) s += ` · ${gaps} gaps`;
    if (pk === 0) s += " · no mic data yet";
  }
  if (wrongFirmware) s += " · board is sending 4 mics: flash cam_mic_record8";
  if (boardMsg && !boardStats) s += ` · ${boardMsg}`;
  $("conn").textContent = s;
  $("conn").title = boardStats || boardMsg;
}, 500);

setInterval(() => model.tick(S.thr), 33);

// ------------------------------------------------------------------ controls

const hint = (t) => { $("hint").textContent = t; };

function uiSync() {
  $("cc").classList.toggle("on", cap.on);
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

$("centre").onclick = () => {
  const a = lastAngle();
  if (a === null) return;
  S.offset = a;
  save();
  hint(`Centre set: picture centre = ${a >= 0 ? "+" : ""}${a.toFixed(0)}° on the array.`);
};
$("edge").onclick = () => {
  const a = lastAngle();
  if (a === null) return;
  const half = wrap(S.offset - a);
  if (half < 5 || half > 70) {
    hint(`That clap was ${half.toFixed(0)}° from centre; clap at the RIGHT edge, or try Swap L/R.`);
    return;
  }
  S.hfov = 2 * half;
  save();
  uiSync();
  hint(`Field of view set to ${S.hfov.toFixed(0)}°.`);
};
$("swaplr").onclick = () => { S.swap = !S.swap; save(); uiSync(); };
$("swapfb").onclick = () => { S.swapfb = !S.swapfb; save(); uiSync(); };
$("mirror").onclick = () => { S.mirror = !S.mirror; save(); uiSync(); };
$("rotate").onclick = () => { S.rotate = (S.rotate + 90) % 360; save(); uiSync(); };
$("fov").onchange = (e) => { S.hfov = Math.max(20, Math.min(140, +e.target.value || 60)); save(); uiSync(); };
$("thr").oninput = (e) => { S.thr = +e.target.value; save(); uiSync(); };
$("layout").onchange = (e) => { S.layout = e.target.value; model.setGeom(PRESETS[S.layout].geom); save(); };
$("res").onchange = (e) => conn?.send(new TextEncoder().encode(e.target.value));
$("morebtn").onclick = () => { $("more").classList.toggle("open"); $("morebtn").classList.toggle("on"); setTimeout(resize, 0); };
$("rec").onclick = () => {
  const on = !$("rec").classList.contains("on");
  conn?.send(new TextEncoder().encode(on ? "a" : "s"));
  $("rec").classList.toggle("on", on);
  $("rec").textContent = on ? "■ Stop SD" : "● Rec SD";
};
$("cc").onclick = () => {
  cap.on = !cap.on;
  S.cc = cap.on;
  save();
  cap.failed = "";
  if (cap.on) capStart(); else capStop();
  uiSync();
};

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
  lastSeq = null;
  gaps = 0;
  $("connect").classList.add("hidden");
  if (cap.on) capStart();
  await c.run((data) => parser.feed(data), (s) => { connState = s; });
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
uiSync();
resize();
requestAnimationFrame(render);
if (params.get("ws")) connect(new WsConn(params.get("ws")));
else if (params.has("demo")) connect(new DemoConn());
