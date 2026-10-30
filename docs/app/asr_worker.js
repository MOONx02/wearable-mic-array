// Moonshine base (MIT, Useful Sensors) through transformers.js, all served from this site so it works
// offline. Runs off the main thread so drawing and the USB reads never wait on it.
import { pipeline, env } from "./lib/transformers.js";

env.allowRemoteModels = false;
env.allowLocalModels = true;
env.localModelPath = new URL("models/", self.location.href).pathname;  // a full URL would count as remote
env.useBrowserCache = true;  // keeps the model on the device after the first download
env.backends.onnx.wasm.wasmPaths = new URL("lib/ort/", self.location.href).href;
// several threads only work when the page is cross-origin isolated (sw.js adds the headers)
const forced = new URL(self.location.href).searchParams.get("threads");
env.backends.onnx.wasm.numThreads = forced ? +forced
  : self.crossOriginIsolated ? Math.min(4, navigator.hardwareConcurrency || 2) : 1;

let asr = null;

async function load() {
  const seen = {};
  asr = await pipeline("automatic-speech-recognition", "moonshine-base", {
    device: "wasm",
    dtype: { encoder_model: "q8", decoder_model_merged: "q8" },
    progress_callback: (p) => {
      if (p.status !== "progress" || !p.total) return;
      seen[p.file] = [p.loaded, p.total];
      let a = 0, b = 0;
      for (const [l, t] of Object.values(seen)) { a += l; b += t; }
      postMessage({ type: "progress", mb: a / 1e6, total: b / 1e6 });
    },
  });
  postMessage({ type: "stage", text: "warming up better words…" });
  // first run builds the kernels; do it now instead of on the first sentence
  await asr(new Float32Array(16000), { max_new_tokens: 2 });
  postMessage({ type: "ready", threads: env.backends.onnx.wasm.numThreads });
}

onmessage = async ({ data }) => {
  if (data.type === "load") {
    try { await load(); } catch (e) { postMessage({ type: "error", message: String(e?.message || e) }); }
    return;
  }
  if (data.type === "run" && asr) {
    const t = performance.now(), sec = data.audio.length / 16000;
    let text = "";
    try {
      // the pipeline's default allows floor(seconds) * 6 tokens, which is zero for short phrases
      text = (await asr(data.audio, { max_new_tokens: Math.ceil(sec * 6.5) + 6 })).text.trim();
    } catch (e) {
      postMessage({ type: "error", message: String(e?.message || e) });
    }
    postMessage({ type: "done", id: data.id, text, ms: performance.now() - t, sec });
  }
};
