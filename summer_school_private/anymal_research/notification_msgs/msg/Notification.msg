Header header
uint8 level
uint8 LEVEL_DEBUG = 0
uint8 LEVEL_INFO  = 1
uint8 LEVEL_WARN  = 2
uint8 LEVEL_ERROR = 3
uint8 LEVEL_FATAL = 4
uint32 code
string id
string name             # max 20 chars
string description
string[] output_devices # names of the output devices, broadcast if empty