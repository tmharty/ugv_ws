# One recognized utterance from voice_ear.
std_msgs/Header header
string text          # normalized transcript ("" = heard nothing usable)
float32 confidence   # 0..1 ASR self-reported confidence (0.0 if unknown)
string asr_backend   # e.g. "faster-whisper-base.en-int8" (Phase 4 A/B logging)
bool stop_word       # ear's stop-lexicon scanner matched (estop already fired)
