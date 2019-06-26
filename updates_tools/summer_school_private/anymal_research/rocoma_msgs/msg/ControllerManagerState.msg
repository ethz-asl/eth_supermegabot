# States
int8 STATE_FAILURE = -2     # Failproof controller is active
int8 STATE_EMERGENCY = -1   # Emergency controller is active
int8 STATE_NA = 0           # Not available (safe initialization)
int8 STATE_OK = 1           # Controller is active

time stamp
int8 state
bool estop_cleared
