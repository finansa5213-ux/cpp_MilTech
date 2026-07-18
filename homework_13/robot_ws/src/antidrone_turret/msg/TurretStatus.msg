uint8 TARGET_NONE=0
uint8 TARGET_LOW_CONFIDENCE=1
uint8 TARGET_LOCKED=2

uint8 ACTION_IDLE=0
uint8 ACTION_TRACK=1

uint8 TRIGGER_SKIP=0
uint8 TRIGGER_REQUESTED=1
uint8 TRIGGER_RELOADING=2

uint8 target_state
uint8 action
uint8 trigger_state
float32 confidence
float32 distance_m
