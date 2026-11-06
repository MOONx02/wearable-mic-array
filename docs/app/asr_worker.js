// Better words for each finished sentence, off the main thread so drawing and the USB reads never wait on it.
// Moonshine base (MIT, Useful Sensors) is served from this site so it works offline; the Whisper models
// (MIT, OpenAI) are too big for the site and come from Hugging Face the first time, then stay on the device.
import { pipeline, env } from "./lib/transformers.js";

env.localModelPath = new URL("models/", self.location.href).pathname;  // a full URL would count as remote
env.useBrowserCache = true;  // keeps the model on the device after the first download
env.backends.onnx.wasm.wasmPaths = new URL("lib/ort/", self.location.href).href;
// several threads only work when the page is cross-origin isolated (sw.js adds the headers)
const forced = new URL(self.location.href).searchParams.get("threads");
env.backends.onnx.wasm.numThreads = forced ? +forced
  : self.crossOriginIsolated ? Math.min(4, navigator.hardwareConcurrency || 2) : 1;

// gpu(f16): weights for WebGPU (q8 runs slowly there); wasm: weights for the CPU, or null when the CPU is too slow
const MODELS = {
  standard: { id: "moonshine-base", name: "Moonshine", local: true,
              wasm: { encoder_model: "q8", decoder_model_merged: "q8" } },
  better: { id: "onnx-community/whisper-small.en", name: "Whisper small",
            gpu: (f16) => ({ encoder_model: f16 ? "fp16" : "fp32", decoder_model_merged: "q4" }),
            wasm: { encoder_model: "q8", decoder_model_merged: "q8" } },
  best: { id: "onnx-community/whisper-large-v3-turbo", name: "Whisper turbo", english: true,
          gpu: (f16) => ({ encoder_model: f16 ? "fp16" : "q4", decoder_model_merged: f16 ? "q4f16" : "q4" }),
          wasm: null },
};

let asr = null, opts = {};

async function gpuAdapter() {
  try {
    const a = await navigator.gpu?.requestAdapter({ powerPreference: "high-performance" });
    return a && !a.isFallbackAdapter ? a : null;
  } catch (e) {
    return null;
  }
}

async function load(key) {
  const m = MODELS[key];
  env.allowLocalModels = !!m.local;
  env.allowRemoteModels = !m.local;
  const gpu = m.gpu ? await gpuAdapter() : null;
  if (!gpu && !m.wasm) throw new Error(`${m.name} needs a graphics chip the browser can use (WebGPU)`);
  const device = gpu ? "webgpu" : "wasm";
  const seen = {};
  asr = await pipeline("automatic-speech-recognition", m.id, {
    device,
    dtype: gpu ? m.gpu(gpu.features.has("shader-f16")) : m.wasm,
    progress_callback: (p) => {
      if (p.status !== "progress" || !p.total) return;
      seen[p.file] = [p.loaded, p.total];
      let a = 0, b = 0;
      for (const [l, t] of Object.values(seen)) { a += l; b += t; }
      postMessage({ type: "progress", mb: a / 1e6, total: b / 1e6 });
    },
  });
  opts = m.english ? { language: "english", task: "transcribe" } : {};
  postMessage({ type: "stage", text: "warming up better words…" });
  // first run builds the kernels; do it now instead of on the first sentence
  await asr(new Float32Array(16000), { ...opts, max_new_tokens: 2 });
  postMessage({ type: "ready", name: m.name, device, threads: env.backends.onnx.wasm.numThreads });
}

onmessage = async ({ data }) => {
  if (data.type === "load") {
    try { await load(data.model || "standard"); } catch (e) { postMessage({ type: "error", message: String(e?.message || e) }); }
    return;
  }
  if (data.type === "run" && asr) {
    const t = performance.now(), sec = data.audio.length / 16000;
    let text = "";
    try {
      // the pipeline's default allows floor(seconds) * 6 tokens, which is zero for short phrases
      text = (await asr(data.audio, { ...opts, max_new_tokens: Math.ceil(sec * 6.5) + 6 })).text.trim();
    } catch (e) {
      postMessage({ type: "error", message: String(e?.message || e) });
    }
    postMessage({ type: "done", id: data.id, text, ms: performance.now() - t, sec });
  }
};
