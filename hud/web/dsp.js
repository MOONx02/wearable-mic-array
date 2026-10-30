// Browser port of the beamforming in hud_core.py (STFT, delay-and-sum, diffuse-noise MVDR).
// Angles: 0 = straight ahead, + = wearer's left. Positions in metres: x forward, y left, z up.

const DSP = (() => {
  const C = 343.0;

  function makeFFT(n) {
    const levels = Math.log2(n);
    const rev = new Uint32Array(n);
    for (let i = 0; i < n; i++) {
      let r = 0;
      for (let b = 0; b < levels; b++) r |= ((i >> b) & 1) << (levels - 1 - b);
      rev[i] = r;
    }
    const cos = new Float64Array(n / 2), sin = new Float64Array(n / 2);
    for (let i = 0; i < n / 2; i++) {
      cos[i] = Math.cos(2 * Math.PI * i / n);
      sin[i] = Math.sin(2 * Math.PI * i / n);
    }
    // In-place complex FFT; inverse = true gives the unscaled inverse transform.
    return function fft(re, im, inverse) {
      for (let i = 0; i < n; i++) {
        const j = rev[i];
        if (j > i) {
          let t = re[i]; re[i] = re[j]; re[j] = t;
          t = im[i]; im[i] = im[j]; im[j] = t;
        }
      }
      const sgn = inverse ? 1 : -1;
      for (let size = 2; size <= n; size <<= 1) {
        const half = size >> 1, step = n / size;
        for (let start = 0; start < n; start += size) {
          for (let k = 0; k < half; k++) {
            const wr = cos[k * step], wi = sgn * sin[k * step];
            const a = start + k, b = a + half;
            const xr = re[b] * wr - im[b] * wi;
            const xi = re[b] * wi + im[b] * wr;
            re[b] = re[a] - xr; im[b] = im[a] - xi;
            re[a] += xr; im[a] += xi;
          }
        }
      }
    };
  }

  function hann(n) {
    const w = new Float64Array(n);
    for (let i = 0; i < n; i++) w[i] = 0.5 - 0.5 * Math.cos(2 * Math.PI * i / n);
    return w;
  }

  // Frame t is centred on sample t * hop (signal zero-padded by nfft / 2 at both ends).
  // Returns {re, im} as Float32Array[mic][frame * nbins + bin].
  function stft(channels, nfft, hop) {
    const len = channels[0].length;
    const frames = Math.floor(len / hop) + 1;
    const nb = nfft / 2 + 1, half = nfft / 2;
    const w = hann(nfft), fft = makeFFT(nfft);
    const re = new Float64Array(nfft), im = new Float64Array(nfft);
    const out = { re: [], im: [], frames, nbins: nb };
    for (const x of channels) {
      const R = new Float32Array(frames * nb), I = new Float32Array(frames * nb);
      for (let t = 0; t < frames; t++) {
        const s0 = t * hop - half;
        for (let n = 0; n < nfft; n++) {
          const s = s0 + n;
          re[n] = s >= 0 && s < len ? x[s] * w[n] : 0;
          im[n] = 0;
        }
        fft(re, im, false);
        for (let k = 0; k < nb; k++) {
          R[t * nb + k] = re[k];
          I[t * nb + k] = im[k];
        }
      }
      out.re.push(R);
      out.im.push(I);
    }
    return out;
  }

  function istft(Yre, Yim, frames, nfft, hop, len) {
    const nb = nfft / 2 + 1, half = nfft / 2;
    const w = hann(nfft), fft = makeFFT(nfft);
    const padded = new Float64Array(len + nfft + hop), norm = new Float64Array(padded.length);
    const re = new Float64Array(nfft), im = new Float64Array(nfft);
    for (let t = 0; t < frames; t++) {
      for (let k = 0; k < nb; k++) {
        re[k] = Yre[t * nb + k];
        im[k] = Yim[t * nb + k];
      }
      for (let k = nb; k < nfft; k++) {
        re[k] = re[nfft - k];
        im[k] = -im[nfft - k];
      }
      fft(re, im, true);
      const s0 = t * hop;
      for (let n = 0; n < nfft; n++) {
        padded[s0 + n] += (re[n] / nfft) * w[n];
        norm[s0 + n] += w[n] * w[n];
      }
    }
    const y = new Float32Array(len);
    for (let i = 0; i < len; i++) {
      const nrm = norm[i + half];
      y[i] = nrm > 1e-8 ? padded[i + half] / nrm : 0;
    }
    return y;
  }

  function sinc(x) {
    if (Math.abs(x) < 1e-12) return 1;
    const px = Math.PI * x;
    return Math.sin(px) / px;
  }

  function invert(a) {
    const n = a.length;
    const m = a.map((row, i) => [...row, ...Array.from({ length: n }, (_, j) => (i === j ? 1 : 0))]);
    for (let c = 0; c < n; c++) {
      let p = c;
      for (let r = c + 1; r < n; r++) if (Math.abs(m[r][c]) > Math.abs(m[p][c])) p = r;
      [m[c], m[p]] = [m[p], m[c]];
      const d = m[c][c];
      for (let j = 0; j < 2 * n; j++) m[c][j] /= d;
      for (let r = 0; r < n; r++) {
        if (r === c) continue;
        const f = m[r][c];
        if (f !== 0) for (let j = 0; j < 2 * n; j++) m[r][j] -= f * m[c][j];
      }
    }
    return m.map((row) => row.slice(n));
  }

  // Per-frequency inverse of the diffuse-noise coherence matrix (with diagonal loading).
  function diffuseInverses(pos, fs, nfft, loading) {
    const M = pos.length, nb = nfft / 2 + 1;
    const dist = pos.map((p) => pos.map((q) => Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2])));
    const inv = [];
    for (let k = 0; k < nb; k++) {
      const f = (k * fs) / nfft;
      const g = dist.map((row, i) => row.map((d, j) => sinc((2 * f * d) / C) + (i === j ? loading : 0)));
      inv.push(invert(g));
    }
    return inv;
  }

  // Weights for one steering angle: Float64Array [bin * M * 2 + m * 2 + (0 re | 1 im)].
  function weights(pos, fs, nfft, azDeg, kind, gammaInv) {
    const M = pos.length, nb = nfft / 2 + 1;
    const a = (azDeg * Math.PI) / 180;
    const lead = pos.map((p) => (Math.cos(a) * p[0] + Math.sin(a) * p[1]) / C);
    const W = new Float64Array(nb * M * 2);
    const dr = new Float64Array(M), di = new Float64Array(M);
    for (let k = 0; k < nb; k++) {
      const f = (k * fs) / nfft;
      for (let m = 0; m < M; m++) {
        const ph = 2 * Math.PI * f * lead[m];
        dr[m] = Math.cos(ph);
        di[m] = Math.sin(ph);
      }
      const o = k * M * 2;
      if (kind === "das") {
        for (let m = 0; m < M; m++) {
          W[o + 2 * m] = dr[m] / M;
          W[o + 2 * m + 1] = di[m] / M;
        }
        continue;
      }
      const gi = gammaInv[k];
      let denR = 0, denI = 0;
      const nr = new Float64Array(M), ni = new Float64Array(M);
      for (let m = 0; m < M; m++) {
        for (let n = 0; n < M; n++) {
          nr[m] += gi[m][n] * dr[n];
          ni[m] += gi[m][n] * di[n];
        }
        denR += dr[m] * nr[m] + di[m] * ni[m];
        denI += dr[m] * ni[m] - di[m] * nr[m];
      }
      const dd = denR * denR + denI * denI;
      for (let m = 0; m < M; m++) {
        W[o + 2 * m] = (nr[m] * denR + ni[m] * denI) / dd;
        W[o + 2 * m + 1] = (ni[m] * denR - nr[m] * denI) / dd;
      }
    }
    return W;
  }

  // Steer along trackDeg (one angle per HUD block). Angles snap to the 2.5° grid like hud_core.beamform.
  function beamform(S, pos, fs, nfft, hop, framesPerBlock, trackDeg, kind, gammaInv, len) {
    const M = pos.length, nb = S.nbins, frames = S.frames;
    const cache = new Map();
    const wFor = (deg) => {
      const snapped = Math.round(deg / 2.5) * 2.5;
      if (!cache.has(snapped)) cache.set(snapped, weights(pos, fs, nfft, snapped, kind, gammaInv));
      return cache.get(snapped);
    };
    const Yre = new Float32Array(frames * nb), Yim = new Float32Array(frames * nb);
    for (let t = 0; t < frames; t++) {
      const blk = Math.min(Math.floor(t / framesPerBlock), trackDeg.length - 1);
      const W = wFor(trackDeg[blk]);
      for (let k = 0; k < nb; k++) {
        let yr = 0, yi = 0;
        const o = k * M * 2, idx = t * nb + k;
        for (let m = 0; m < M; m++) {
          const wr = W[o + 2 * m], wi = W[o + 2 * m + 1];
          const xr = S.re[m][idx], xi = S.im[m][idx];
          yr += wr * xr + wi * xi;
          yi += wr * xi - wi * xr;
        }
        Yre[idx] = yr;
        Yim[idx] = yi;
      }
    }
    return istft(Yre, Yim, frames, nfft, hop, len);
  }

  function blockDb(y, fs, nBlocks, blockS) {
    const n = Math.round(blockS * fs), out = new Float64Array(nBlocks);
    for (let b = 0; b < nBlocks; b++) {
      let s = 0;
      for (let i = b * n; i < (b + 1) * n; i++) s += i < y.length ? y[i] * y[i] : 0;
      out[b] = 10 * Math.log10(s / n + 1e-12);
    }
    return out;
  }

  return { stft, istft, beamform, blockDb, diffuseInverses };
})();

if (typeof module !== "undefined") module.exports = DSP;
