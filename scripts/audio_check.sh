#!/usr/bin/env bash
#
# Audio sanity check. Runs on the Jetson (in or
# out of the container) or on the x86 dev machine. Exercises: device listing,
# speaker output, mic capture with a live VU meter, playback of the recording,
# and TTS via espeak-ng.
#
# Device defaults are the UGV Beast's verified hardware (2026-08-14):
#   capture  = USB camera mic      (the audio board's own mic is dead)
#   playback = built-in audio board speakers
# Override for other machines, e.g. on the x86 dev box:
#   MIC_DEV=default SPK_DEV=default ./scripts/audio_check.sh
set -uo pipefail

MIC_DEV="${MIC_DEV:-plughw:CARD=Camera,DEV=0}"
SPK_DEV="${SPK_DEV:-plughw:CARD=Device,DEV=0}"
REC_SECS="${REC_SECS:-5}"
WAV="$(mktemp --suffix=.wav)"
trap 'rm -f "$WAV"' EXIT

fail=0
step() { printf '\n=== %s ===\n' "$*"; }
result() {  # result <ok|FAIL> <message>
  if [ "$1" = ok ]; then printf 'OK: %s\n' "$2"; else printf 'FAIL: %s\n' "$2"; fail=1; fi
}

step "Devices"
echo "capture device : $MIC_DEV"
echo "playback device: $SPK_DEV"
echo
arecord -l || result FAIL "arecord -l"
echo
aplay -l || result FAIL "aplay -l"

step "Speaker test ($SPK_DEV) — expect 'Front Left / Front Right' voice"
speaker-test -D "$SPK_DEV" -c 2 -t wav -l 1 \
  && result ok "speaker-test" || result FAIL "speaker-test on $SPK_DEV"

step "Recording ${REC_SECS}s from $MIC_DEV — SPEAK NOW (watch the VU meter)"
arecord -D "$MIC_DEV" -f S16_LE -r 48000 -c 1 -V mono -d "$REC_SECS" "$WAV" \
  && result ok "recorded $(stat -c%s "$WAV") bytes" || result FAIL "arecord on $MIC_DEV"

step "Playing the recording back on $SPK_DEV — expect your own voice"
aplay -D "$SPK_DEV" "$WAV" \
  && result ok "playback" || result FAIL "aplay on $SPK_DEV"

step "TTS (espeak-ng) — expect a synthetic sentence"
if command -v espeak-ng >/dev/null; then
  espeak-ng --stdout "Audio check complete. All systems nominal. Beep." \
      | aplay -D "$SPK_DEV" \
    && result ok "espeak-ng" || result FAIL "espeak-ng | aplay"
else
  result FAIL "espeak-ng not installed (rebuild the container image)"
fi

step "Summary"
if [ "$fail" = 0 ]; then
  echo "All checks passed. Ears heard, mouth spoke."
else
  echo "One or more checks FAILED — see above."
fi
exit "$fail"
