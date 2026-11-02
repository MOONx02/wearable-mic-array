#!/bin/sh
# Builds cam_mic_vest6 and puts what the browser updater (web_app/update.html) flashes in
# web_app/firmware/: the four flash parts at the addresses arduino-cli would upload them to, and
# manifest.json naming them. Run from anywhere: sh esp32s3/tools/build_vest6.sh
set -e
here=$(cd "$(dirname "$0")/.." && pwd)
sketch="$here/cam_mic_vest6"
out="$here/web_app/firmware"
build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT

arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi,CDCOnBoot=cdc --output-dir "$build" "$sketch"

core=$(arduino-cli config get directories.data)/packages/esp32/hardware/esp32
core="$core/$(ls "$core" | sort -V | tail -1)"
version=$(sed -n 's/.*FIRMWARE_VERSION = \([0-9]*\);.*/\1/p' "$sketch/cam_mic_vest6.ino")

mkdir -p "$out"
rm -f "$out"/*.bin
cp "$build/cam_mic_vest6.ino.bootloader.bin" "$out/bootloader.bin"
cp "$build/cam_mic_vest6.ino.partitions.bin" "$out/partitions.bin"
cp "$core/tools/partitions/boot_app0.bin" "$out/boot_app0.bin"
cp "$build/cam_mic_vest6.ino.bin" "$out/cam_mic_vest6.bin"

part() {  # file address
  printf '    {"file": "%s", "address": %d, "size": %d, "md5": "%s"}' "$1" "$2" \
    "$(wc -c < "$out/$1" | tr -d ' ')" "$(md5 -q "$out/$1" 2>/dev/null || md5sum "$out/$1" | cut -d' ' -f1)"
}
{
  printf '{\n  "name": "cam_mic_vest6",\n  "version": %s,\n' "$version"
  printf '  "built": "%s",\n' "$(date -u +%Y-%m-%dT%H:%MZ)"
  printf '  "about": "6 mics: D1 inner pair, D3 outer pair, D4 top pair (SEL 3V3 = right), clock D0, button D2",\n'
  printf '  "parts": [\n'
  part bootloader.bin 0; printf ',\n'
  part partitions.bin 32768; printf ',\n'
  part boot_app0.bin 57344; printf ',\n'
  part cam_mic_vest6.bin 65536; printf '\n'
  printf '  ]\n}\n'
} > "$out/manifest.json"
cat "$out/manifest.json"
