// Keeps the app usable with no internet. The speech models are not cached here: Vosk keeps its own copy
// in IndexedDB and Moonshine in the "transformers-cache" store after the first download.
const CACHE = "mic360-v13";
const CORE = ["./", "index.html", "beacon.html", "app.js", "asr_worker.js", "lib/vosk.js", "lib/transformers.js",
              "manifest.json", "icon-192.png", "icon-512.png",
              "demo/tts_a1.wav", "demo/tts_a2.wav", "demo/tts_b1.wav"];

self.addEventListener("install", (e) => {
  e.waitUntil(caches.open(CACHE).then((c) => c.addAll(CORE)).then(() => self.skipWaiting()));
});

self.addEventListener("activate", (e) => {
  e.waitUntil(caches.keys()
    .then((keys) => Promise.all(keys.filter((k) => k.startsWith("mic360-") && k !== CACHE).map((k) => caches.delete(k))))
    .then(() => self.clients.claim()));
});

// Cross-origin isolation lets Moonshine use several CPU cores; GitHub Pages can't send these headers itself.
function isolate(r) {
  if (!r || r.status === 0 || r.type === "opaque") return r;
  const h = new Headers(r.headers);
  h.set("Cross-Origin-Opener-Policy", "same-origin");
  h.set("Cross-Origin-Embedder-Policy", "require-corp");
  return new Response(r.body, { status: r.status, statusText: r.statusText, headers: h });
}

self.addEventListener("fetch", (e) => {
  const url = new URL(e.request.url);
  if (e.request.method !== "GET" || url.origin !== location.origin || url.pathname.includes("/models/")) return;
  // network first so updates arrive, the saved copy when offline (or when Wi-Fi has no internet)
  // no-cache: GitHub Pages lets the browser keep files for 10 minutes, which would hide an update
  const fromNet = fetch(new Request(e.request, { cache: "no-cache" })).then((r) => {
    if (r.ok) {
      const copy = r.clone();
      caches.open(CACHE).then((c) => c.put(e.request, copy));
    }
    return r;
  });
  const timeout = new Promise((_, reject) => setTimeout(() => reject(new Error("slow")), 4000));
  e.respondWith(Promise.race([fromNet, timeout])
    .catch(() => caches.match(e.request, { ignoreSearch: true }).then((r) => r || fromNet))
    .then(isolate));
});
