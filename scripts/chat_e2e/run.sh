#!/usr/bin/env bash
# End-to-end check of the chat-with-tools plumbing WITHOUT a real LLM, mic,
# speaker or robot: a fake Ollama (fake_ollama.py) streams scripted tool
# calls, the real chat_node + behavior_ctrl run against simulated /odom,
# and checker.py drives the Phase 2 scenarios (speak-then-act, stop word
# -> zero twist <300 ms, 999 m clamp, unknown tool, compound refusal,
# go_to_point confirmation, offline RuleBrain fallback).
#
# Run inside the Humble container from the workspace root, after a build:
#     bash scripts/chat_e2e/run.sh
# Uses ROS_DOMAIN_ID=77 and port 11435 so it never touches a real daemon.
set +u   # ROS setup.bash is not -u clean
WS="$(cd "$(dirname "$0")/../.." && pwd)"
E="$(cd "$(dirname "$0")" && pwd)"
LOG="${E}/logs"; mkdir -p "$LOG"
source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=77
# behavior_ctrl writes saved points to the tracked map_points.txt; the
# nav scenario saves a fake point, so restore the file afterwards.
MP="$WS/map_points.txt"; [ -f "$MP" ] && cp "$MP" "$LOG/map_points.bak"
python3 "$E/fake_ollama.py" 11435 > "$LOG/ollama.log" 2>&1 & OLL=$!
ros2 run ugv_tools behavior_ctrl > "$LOG/behavior_ctrl.log" 2>&1 & BC=$!
ros2 run ugv_voice chat_node --ros-args -p ollama_url:=http://127.0.0.1:11435 \
  -p speak_before_act_s:=0.5 -p auto_listen:=false -p connect_timeout_s:=1.0 \
  -p request_timeout_s:=10.0 > "$LOG/chat_node.log" 2>&1 & CH=$!
sleep 2
python3 -u "$E/checker.py" 2>&1 | while IFS= read -r line; do
  echo "$line"; [ "$line" = KILL_OLLAMA ] && kill $OLL   # the offline scenario
done
kill $BC $CH $OLL 2>/dev/null; wait 2>/dev/null
[ -f "$LOG/map_points.bak" ] && cp "$LOG/map_points.bak" "$MP"
echo; echo "=== chat_node (tool/estop lines) — full logs in $LOG"
grep -E "tool call|estop|dropped|clamped|offline|goal|barge|failed" "$LOG/chat_node.log" | head -40
