#!/usr/bin/env bash
# Phase 2 setup check: does the host's Ollama daemon + model do native tool
# calling, non-streamed and streamed, with the request shape chat_node sends?
# Run on the Jetson host (or wherever Ollama runs):  scripts/ollama_tools_check.sh
# Expected: `tools` in the capabilities list, and both requests return a
# message.tool_calls entry with function.name "move" and JSON arguments —
# not prose describing the action.
set -u
URL="${OLLAMA_URL:-http://localhost:11434}"
MODEL="${1:-granite4:3b}"

echo "== ollama version";   ollama --version 2>/dev/null || echo "(ollama CLI not on PATH)"
echo "== capabilities for $MODEL (must list 'tools')"
ollama show "$MODEL" 2>/dev/null | sed -n '/[Cc]apabilities/,/^$/p' || echo "(ollama show failed)"

TOOLS='[{"type":"function","function":{"name":"move",
  "description":"Drive the robot forward or backward",
  "parameters":{"type":"object","properties":{
    "direction":{"type":"string","enum":["forward","backward"]},
    "distance_m":{"type":"number"}},
  "required":["direction","distance_m"]}}}]'

for stream in false true; do
  echo "== stream=$stream: 'drive forward half a meter'"
  curl -s -m 120 "$URL/api/chat" -d "{
    \"model\": \"$MODEL\", \"stream\": $stream, \"think\": false,
    \"messages\": [{\"role\":\"user\",\"content\":\"drive forward half a meter\"}],
    \"tools\": $TOOLS }" | grep -o '"tool_calls":\[[^]]*\]' || echo "!! no tool_calls in response"
done
echo "If streaming shows no tool_calls but non-streaming does: upgrade Ollama"
echo "(mid-2025 or later). If neither does: try qwen2.5:3b-instruct-q4_K_M."
