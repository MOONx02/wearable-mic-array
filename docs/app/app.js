// Mic array 360 in the browser: reads the ESP32-S3 (cam_mic_record8) USB stream, finds the direction of
// sounds with SRP-PHAT over every mic pair (same maths as mic_camera_view360.py), draws the camera
// picture with a glow / edge arrows, a 360° radar, mic levels, and live subtitles: the array's voice focus
// (MVDR + Wiener, as in mic_caption_view.py) feeds an offline recognizer (Vosk), placed by the talker's direction.
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
                          mirror: false, thr: 12, cc: true, focus: true }, JSON.parse(localStorage.getItem("mic360") || "{}"));
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

  tick(thr, voice = false) {
    if (this.total < FS / 10) return;
    this.ticks = (this.ticks || 0) + 1;
    for (let c = 0; c < CH; c++) {
      const blk = this.tail(c, 800, this.blk);
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
    this.direction(thr, voice);
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

  steering(chans, top = BAND[1]) {
    const key = chans.join(",") + "@" + top;
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
    const k0 = Math.ceil((BAND[0] * NFFT) / FS), k1 = Math.floor((top * NFFT) / FS);
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

  // voice: someone is talking (voice focus says so), so search only the voice band, where the talker
  // beats a fan; a clap keeps the full band
  direction(thr, voice = false) {
    const act = this.active();
    if (this.total < FS / 2 || act.length < 2) return;
    const srcs = act.map((a) => a[1]);
    let strength = -Infinity;
    for (const s of srcs) strength = Math.max(strength, this.level[s] - this.floor[s]);
    if (strength < thr) return;
    if (this.last && this.now - this.last.t < 0.08) return;
    const st = this.steering(act.map((a) => a[0]), voice ? 3400 : BAND[1]);
    // while someone talks, show the whole phrase's direction (as the subtitles do), not one 64 ms snapshot
    const phrase = voice ? speechCurve() : null;
    const curve = phrase && phrase.n >= 5 ? phrase.sum : this.srp(srcs, st);
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
        const s = new Int16Array(b.buffer.slice(b.byteOffset + p + 10, b.byteOffset + end));  // little-endian host
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
        // several large reads queued, so the board's buffer keeps draining even when the page is busy
        for (let i = 0; i < 12; i++) q.push(this.dev.transferIn(ep, 16384));
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
    if (ev[0] === "say" && this.clips[ev[1]]) return { sig: spatial(this.clips[ev[1]], ev[2], false, 2500, "rms"), gap: 0.9 * FS };
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
    return new Promise((resolve) => {
      this.timer = setInterval(() => {
        if (this.closed) { clearInterval(this.timer); return resolve(); }
        if (cam && !camBusy && performance.now() >= nextCam) {  // 10 fps JPEG, like the board's camera
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
            const fi = (made + f) & 65535;
            for (let c = 0; c < CH; c++) {
              let v = -30935;
              if (c in model.geom) {
                v = gauss() * 25 + (fan[c] ? fan[c][fi] : 0);
                if (ev && ev.sig[c]) v += ev.sig[c][pos];
              }
              dv.setInt16(10 + (f * CH + c) * 2, Math.max(-32768, Math.min(32767, Math.round(v))), true);
            }
            if (ev && ++pos >= ev.sig.len) { wait = ev.gap; ev = null; }
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
}

function gauss() {
  return Math.sqrt(-2 * Math.log(Math.random() + 1e-12)) * Math.cos(2 * Math.PI * Math.random());
}

const ffts = new Map();
const fftOf = (n) => ffts.get(n) || (ffts.set(n, makeFFT(n)), ffts.get(n));

// Mono signal arriving from angle ang at every mic in the current layout (fractional delays in the
// frequency domain), scaled so its peak or active-speech RMS is `level`.
function spatial(sig, ang, circular, level, by) {
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
  const th = (ang * Math.PI) / 180, ux = Math.sin(th), uy = Math.cos(th), out = { len: circular ? n : sig.length + pad };
  for (const [ch, [, x, y]] of Object.entries(model.geom)) {
    const delay = -((x / 1000) * ux + (y / 1000) * uy) / C, r = new Float64Array(n), i = new Float64Array(n);
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
let conn = null, connState = "", wrongFirmware = false, boardStats = "", boardMsg = "";
let image = null, decoding = false, lastSeq = null, gaps = 0;
const pktTimes = [], frameTimes = [], rxTimes = [];
const busy = { ms: 0 };  // main-thread time spent on our own work, for the load figure in the status line

const parser = new Parser({
  mic(seq, frames, s) {
    if (lastSeq !== null && seq !== ((lastSeq + 1) >>> 0)) gaps++;
    lastSeq = seq;
    model.push(s, frames);
    if (cap.on) runFocus(s, frames);
    pktTimes.push(performance.now());
  },
  wrong() { wrongFirmware = true; },
  jpeg(bytes) {
    rxTimes.push(performance.now());
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

// ------------------------------------------------------------------ voice focus (StreamingFocus from mic_caption_view.py)

// MVDR beam toward the talker + Wiener hiss filter on 8 ms frames. It learns the background from quiet
// frames and the talker from speech frames as it goes, so it follows whoever is talking.
const FN = 512, FH = 128, FB = FN / 2 + 1, FK0 = 10, FK1 = 108, FLO = 3;  // band 300–3400 Hz; bins under 70 Hz dropped
const fft512 = makeFFT(FN);
const FWIN = Float64Array.from({ length: FN }, (_, i) => Math.sqrt(0.5 - 0.5 * Math.cos((2 * Math.PI * i) / FN)));

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
  constructor(srcs) {
    const n = (this.n = srcs.length), nn = n * n;
    this.srcs = srcs.slice();
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
  frame(cb) {
    const n = this.n, nn = n * n;
    for (let q = 0; q < n; q++) { this.buf[q].copyWithin(0, FH); this.buf[q].set(this.pend[q], FN - FH); }
    this.fill = Math.min(FN, this.fill + FH);
    if (this.fill < FN) return;
    let eb = 0, ew = 0;
    for (let q = 0; q < n; q++) {
      const re = this.Xre[q], im = this.Xim[q], b = this.buf[q];
      for (let i = 0; i < FN; i++) { re[i] = b[i] * FWIN[i]; im[i] = 0; }
      fft512(re, im);
      for (let k = FLO; k < FB; k++) {
        const p = re[k] * re[k] + im[k] * im[k];
        ew += p;
        if (k >= FK0 && k <= FK1) eb += p;
      }
    }
    const e = 10 * Math.log10(eb / (n * (FK1 - FK0 + 1)) + 1e-12), w = 10 * Math.log10(ew / (n * (FB - FLO)) + 1e-12);
    if (this.floor === null) this.floor = e;
    else if (e < this.floor) this.floor = 0.9 * this.floor + 0.1 * e;
    else this.floor += 0.002;
    const speech = e > this.floor + 8, quiet = e < this.floor + 3;
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
      if (quiet || speech) {
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
        const g = Math.max(prio / (1 + prio), 0.12);
        this.gPrev[k] = g;
        this.postPrev[k] = post;
        Yre[k] = yr * g;
        Yim[k] = yi * g;
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

function trackSpeechDir(weight) {
  const act = model.active();
  if (act.length < 2) return;
  const st = model.steering(act.map((a) => a[0]), 3400);
  const c = model.srp(act.map((a) => a[1]), st), sorted = Array.from(c).sort((a, b) => a - b);
  const med = (sorted[179] + sorted[180]) / 2, span = Math.max(sorted[359] - med, 1e-9);
  const curve = new Float32Array(360);
  for (let k = 0; k < 360; k++) curve[k] = (weight * (c[k] - med)) / span;
  const t = asr.fed / FS;
  utt.log.push({ t, curve, back: st.hasBack });
  while (utt.log.length && t - utt.log[0].t > 60) utt.log.shift();
}

function speechCurve(t0 = utt.lastEnd, t1 = Infinity) {
  const sum = new Float64Array(360);
  let n = 0, back = true;
  for (const e of utt.log) {
    if (e.t < t0 || e.t > t1) continue;
    for (let k = 0; k < 360; k++) sum[k] += e.curve[k];
    back = e.back;
    n++;
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

// Better words: Vosk shows text instantly, then Moonshine base (about as accurate as Whisper base.en)
// re-reads each finished sentence from the same focused audio and replaces it.
const HQ_URL = "asr_worker.js";
const hq = { worker: null, ready: false, status: "", queue: [], busy: null, id: 0, ring: new Float32Array(FS * 40), last: "" };

function hqLoad() {
  if (hq.worker || typeof Worker === "undefined") return;
  try {
    hq.worker = new Worker(HQ_URL, { type: "module" });
  } catch (e) {
    hq.status = "better words unavailable";
    return;
  }
  hq.status = "loading better words…";
  hq.worker.onmessage = ({ data: m }) => {
    if (hq.failed) return;
    if (m.type === "progress") hq.status = `loading better words ${m.mb.toFixed(0)}/${m.total.toFixed(0)} MB (first time only)`;
    else if (m.type === "stage") hq.status = m.text;
    else if (m.type === "ready") { hq.ready = true; hq.status = ""; hq.threads = m.threads; hqNext(); }
    else if (m.type === "error") {
      console.warn("moonshine:", m.message);
      if (!hq.ready) { hq.failed = true; hq.status = "better words didn't load"; }
    }
    else if (m.type === "done") {
      const job = hq.busy;
      hq.busy = null;
      if (job && job.id === m.id && plausible(m.text, m.sec)) setPart(job.part, m.text);
      hq.last = `${(m.ms / 1000).toFixed(1)} s for ${m.sec.toFixed(1)} s`;
      hqNext();
    }
  };
  hq.worker.onerror = (e) => { hq.status = "better words didn't load"; console.warn("moonshine worker:", e.message); };
  hq.worker.postMessage({ type: "load" });
}

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
  const srcs = model.active().map((a) => a[1]);
  if (!srcs.length) return;
  if (!focus || focus.srcs.join() !== srcs.join()) focus = new Focus(srcs);
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
  const failed = asr.status.startsWith("speech model didn't");
  const mics = focus ? `${focus.n} mic${focus.n === 1 ? "" : "s"}` : "waiting for mics";
  const state = !cap.on ? "captions off" : asr.status ? (failed ? "speech model failed" : "loading speech model…")
    : `offline · ${S.focus ? (focus && focus.wRe ? "voice focus" : "voice focus learning") : "plain mix"} · ${mics}`
      + (hq.status ? ` · ${hq.status}` : hq.ready ? ` · Moonshine${hq.last ? ` ${hq.last}` : ""}` : "");
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
    text(`${t}${rel === null ? "" : ` · ${sideText(rel)}${side === "in" ? "" : " (off screen)"}`}`, inner.x, y + 7, 11, DIM);
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
}

// ------------------------------------------------------------------ status line

function rate(times, now) {
  while (times.length && now - times[0] > 2000) times.shift();
  return times.length > 1 ? ((times.length - 1) * 1000) / Math.max(1, now - times[0]) : 0;
}

let busyAt = performance.now();
setInterval(() => {
  const now = performance.now(), pk = rate(pktTimes, now), fps = rate(frameTimes, now), rx = rate(rxTimes, now);
  const load = (100 * busy.ms) / (now - busyAt);
  busy.ms = 0;
  busyAt = now;
  let s = conn ? `${conn.name} · ${connState}` : "not connected";
  if (conn && (connState === "streaming" || conn instanceof DemoConn)) {
    s += ` · ${(pk * 128 / 1000).toFixed(1)} k samples/s · camera ${fps.toFixed(0)} fps`;
    // shown here vs arrived over USB vs what the board says it sent: tells where frames get lost
    const bs = /([\d.]+) camera fps, (\d+) frames skipped/.exec(boardStats);
    if (rx > fps + 1.5 || bs) s += ` (got ${rx.toFixed(0)}${bs ? `, board sent ${(+bs[1]).toFixed(0)}, skipped ${bs[2]}` : ""})`;
    s += ` · load ${load.toFixed(0)}%`;
    if (gaps) s += ` · ${gaps} gaps`;
    if (pk === 0) s += " · no mic data yet";
  }
  if (wrongFirmware) s += " · board is sending 4 mics: flash cam_mic_record8";
  if (boardMsg && !boardStats) s += ` · ${boardMsg}`;
  $("conn").textContent = s;
  $("conn").title = boardStats || boardMsg;
}, 500);

setInterval(() => {
  const t0 = performance.now();
  model.tick(S.thr, t0 < cap.voiceUntil);
  busy.ms += performance.now() - t0;
}, 33);

// ------------------------------------------------------------------ controls

const hint = (t) => { $("hint").textContent = t; };

function uiSync() {
  $("cc").classList.toggle("on", cap.on);
  $("focus").classList.toggle("on", S.focus);
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
  cap.interim = null;
  if (cap.on) asrLoad();
  uiSync();
};
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
  lastSeq = null;
  gaps = 0;
  $("connect").classList.add("hidden");
  focus = null;
  await c.run((data) => {
    const t0 = performance.now();
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
uiSync();
resize();
requestAnimationFrame(render);
if (cap.on) asrLoad();
if ("serviceWorker" in navigator && window.isSecureContext) navigator.serviceWorker.register("sw.js").catch(() => {});
if (params.get("ws")) connect(new WsConn(params.get("ws")));
else if (params.has("demo")) connect(new DemoConn());
