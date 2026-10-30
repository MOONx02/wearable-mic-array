// Keeps the app usable with no internet. The speech model is not cached here: the recognizer keeps its
// own copy (in IndexedDB) after the first download.
const CACHE = "mic360-v3";
const CORE = ["./", "index.html", "app.js", "lib/vosk.js", "manifest.json", "icon-192.png", "icon-512.png",
              "demo/tts_a1.wav", "demo/tts_a2.wav", "demo/tts_b1.wav"];

self.addEventListener("install", (e) => {
  e.waitUntil(caches.open(CACHE).then((c) => c.addAll(CORE)).then(() => self.skipWaiting()));
});

self.addEventListener("activate", (e) => {
  e.waitUntil(caches.keys()
    .then((keys) => Promise.all(keys.filter((k) => k !== CACHE).map((k) => caches.delete(k))))
    .then(() => self.clients.claim()));
});

self.addEventListener("fetch", (e) => {
  const url = new URL(e.request.url);
  if (e.request.method !== "GET" || url.origin !== location.origin || url.pathname.includes("/models/")) return;
  // network first so updates arrive, the saved copy when offline (or when Wi-Fi has no internet)
  const fromNet = fetch(e.request).then((r) => {
    if (r.ok) {
      const copy = r.clone();
      caches.open(CACHE).then((c) => c.put(e.request, copy));
    }
    return r;
  });
  const timeout = new Promise((_, reject) => setTimeout(() => reject(new Error("slow")), 4000));
  e.respondWith(Promise.race([fromNet, timeout])
    .catch(() => caches.match(e.request, { ignoreSearch: true }).then((r) => r || fromNet)));
});
