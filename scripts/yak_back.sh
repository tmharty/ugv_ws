#!/usr/bin/env bash
#
# Audio soak test. Loops forever: announces "Recording in 5, 4, 3, 2, 1"
# over the speaker, records for 10 seconds, then plays the recording back
# once per entry in SPEEDS (default: normal, then chipmunk). Ctrl-C to stop.
#
# Device defaults are the UGV Beast's verified hardware (2026-08-14):
#   capture  = USB camera mic      (the audio board's own mic is dead)
#   playback = built-in audio board speakers
# Override for other machines, e.g. on the x86 dev box:
#   MIC_DEV=default SPK_DEV=default ./scripts/yak_back.sh
#
# SPEEDS controls the playbacks: one playback per listed factor. Pitch and
# speed shift together (tape-deck style: 1.3 = faster + higher, 0.7 = slower
# + deeper) — no sox/ffmpeg in the container for independent pitch control.
#   SPEEDS="1.0 1.0"      normal twice (the original behavior)
#   SPEEDS="0.7 1.0 1.5"  three playbacks: deep, normal, chipmunk
set -uo pipefail

MIC_DEV="${MIC_DEV:-plughw:CARD=Camera,DEV=0}"
SPK_DEV="${SPK_DEV:-plughw:CARD=Device,DEV=0}"
REC_SECS="${REC_SECS:-10}"
REC_RATE=48000
SPEEDS="${SPEEDS:-0.7 1.0 1.3}"
WAV="$(mktemp --suffix=.wav)"
trap 'rm -f "$WAV"' EXIT

step() { printf '\n=== %s ===\n' "$*"; }

say() {  # speak a phrase on the speaker; fall back to printing if espeak-ng is missing
  if command -v espeak-ng >/dev/null; then
    espeak-ng --stdout "$*" | aplay -q -D "$SPK_DEV"
  else
    printf '%s\n' "$*"
  fi
}

iteration=1
while :; do
  step "Iteration $iteration"

  say "Recording in 5, 4, 3, 2, 1"

  step "Recording ${REC_SECS}s from $MIC_DEV — SPEAK NOW (watch the VU meter)"
  arecord -D "$MIC_DEV" -f S16_LE -r "$REC_RATE" -c 1 -V mono -d "$REC_SECS" "$WAV" \
    || { echo "FAIL: arecord on $MIC_DEV"; exit 1; }

  pass=1
  for speed in $SPEEDS; do
    step "Playback $pass (speed x$speed) on $SPK_DEV"
    # Tape-deck pitch/speed shift: replay the samples as headerless raw audio
    # at a scaled rate (tail skips the 44-byte WAV header arecord writes).
    rate=$(awk -v r="$REC_RATE" -v s="$speed" 'BEGIN { printf "%d", r * s }')
    tail -c +45 "$WAV" | aplay -q -D "$SPK_DEV" -t raw -f S16_LE -c 1 -r "$rate" \
      || { echo "FAIL: aplay on $SPK_DEV"; exit 1; }
    pass=$((pass + 1))
  done

  iteration=$((iteration + 1))
done
