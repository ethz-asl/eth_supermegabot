# Robot state including minimal set of generalized coordinates
# and velocities as well as contact information
Header header
int8 STATE_ERROR_SENSOR = -3
int8 STATE_ERROR_ESTIMATOR = -2
int8 STATE_ERROR_UNKNOWN = -1
int8 STATE_OK = 0
int8 STATE_WARN_SENSOR_POSE = 1 
int8 state
geometry_msgs/PoseStamped pose
geometry_msgs/TwistStamped twist
sensor_msgs/JointState joints
Contact[] contacts
geometry_msgs/TransformStamped[] frame_transforms
