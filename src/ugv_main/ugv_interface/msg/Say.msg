# One utterance for voice_mouth to speak.
uint8 PRIORITY_NORMAL=0   # queued behind current speech
uint8 PRIORITY_SAFETY=1   # interrupts current speech, flushes the queue

string key       # response-library key it was rendered from (logging/tests)
string text      # exact text to speak — scripted library in safe mode,
                 # LLM prose or scripted acks/refusals in chat-with-tools
uint8 priority

# Wav playback entry (record_replay tool). When wav_path is set, text is
# ignored and the file is played through the same queue, so safety-priority
# speech still preempts it.
string wav_path
float32 speed_factor   # tape-deck pitch+speed: 0.7 deep, 1.0 as recorded,
                       # 1.3 chipmunk; 0 means 1.0
bool delete_after      # unlink wav_path once loaded (last playback of a
                       # temp recording — recordings never persist)
