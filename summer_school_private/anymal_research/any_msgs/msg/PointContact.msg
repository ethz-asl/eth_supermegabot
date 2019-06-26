# Contact definition
uint8 STATE_OPEN=0
uint8 STATE_CLOSED=1
uint8 STATE_SLIPPING=2

Header header
geometry_msgs/Wrench wrench
geometry_msgs/Point position
geometry_msgs/Twist twist
geometry_msgs/Vector3 normal
uint8 state