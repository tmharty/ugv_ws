# One utterance for voice_mouth to speak.
uint8 PRIORITY_NORMAL=0   # queued behind current speech
uint8 PRIORITY_SAFETY=1   # interrupts current speech, flushes the queue

string key       # response-library key it was rendered from (logging/tests)
string text      # exact text to speak — scripted library in the kid-facing
                 # pipeline; the opt-in talk-only chat mode sends LLM text
uint8 priority
