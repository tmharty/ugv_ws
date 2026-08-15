#!/usr/bin/env bash
#
# Fetch the Phase 1 voice models into <repo>/models/ (host-mounted at
# /home/ws/ugv_ws/models inside the container, which is what voice_params.yaml
# points at). Idempotent: existing non-empty files are skipped, so re-running
# after a partial download only fetches what's missing.
#
# Contents (~210 MB total):
#   openwakeword/          shared feature models + "hey_jarvis" wake word
#   faster-whisper-base.en/  ASR model (int8 inference happens at load time)
#   piper/                 en_US-lessac-medium TTS voice
#
# Runs on the host or in the container; only needs curl. Override the target
# with MODELS_DIR=/elsewhere.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
MODELS_DIR="${MODELS_DIR:-$REPO/models}"

fetch() {  # fetch <dest> <url>
  local dest="$1" url="$2"
  if [ -s "$dest" ]; then
    echo "skip (exists): $dest"
    return 0
  fi
  mkdir -p "$(dirname "$dest")"
  echo "fetch: $url"
  curl -L --fail --progress-bar -o "$dest.part" "$url"
  mv "$dest.part" "$dest"
}

# openWakeWord: pinned release. The two feature models are required by every
# wake word; hey_jarvis is the prebuilt word used until we train a custom name.
OWW=https://github.com/dscripka/openWakeWord/releases/download/v0.5.1
fetch "$MODELS_DIR/openwakeword/melspectrogram.onnx"  "$OWW/melspectrogram.onnx"
fetch "$MODELS_DIR/openwakeword/embedding_model.onnx" "$OWW/embedding_model.onnx"
fetch "$MODELS_DIR/openwakeword/hey_jarvis_v0.1.onnx" "$OWW/hey_jarvis_v0.1.onnx"

# faster-whisper base.en (CTranslate2 format). ear_node prefers this local
# directory when it exists, so the robot never needs Hugging Face at runtime.
WHISPER=https://huggingface.co/Systran/faster-whisper-base.en/resolve/main
for f in model.bin config.json tokenizer.json vocabulary.txt; do
  fetch "$MODELS_DIR/faster-whisper-base.en/$f" "$WHISPER/$f"
done

# Piper voice (pleasantly synthetic). Needs the .onnx and its .onnx.json.
PIPER=https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium
fetch "$MODELS_DIR/piper/en_US-lessac-medium.onnx"      "$PIPER/en_US-lessac-medium.onnx"
fetch "$MODELS_DIR/piper/en_US-lessac-medium.onnx.json" "$PIPER/en_US-lessac-medium.onnx.json"

echo
echo "Models in $MODELS_DIR:"
du -sh "$MODELS_DIR"/*/
