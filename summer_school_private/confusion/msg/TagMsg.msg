# Tag Id
uint8 id
float64 tag_size

# Corner indeces
uint8 TOP_LEFT = 0
uint8 TOP_RIGHT = 1
uint8 BOTTOM_LEFT = 2
uint8 BOTTOM_RIGHT = 3

# Tag corners in image coordinates
geometry_msgs/Point[4] corners

# Tag pose in camera frame
geometry_msgs/Pose pose