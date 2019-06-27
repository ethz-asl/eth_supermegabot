# Super Mega Bot state including minimal set of generalized coordinates
# and velocities as well as contact information
Header header
int8 STATE_ERROR_SENSOR = -3
int8 STATE_ERROR_ESTIMATOR = -2
int8 STATE_ERROR_UNKNOWN = -1
int8 STATE_OK = 0
int8 STATE_WARN_SENSOR_POSE = 1 
int8 state

# Position of the base with respect to the origin of the odom frame expressed in odom frame and orientation from base to odom
geometry_msgs/PoseStamped pose

# Linear velocity and (local) angular velocity expressed in base frame.
geometry_msgs/TwistStamped twist
#any_msgs/ExtendedJointState joints
float32[] wheelVelocities
float32[] wheelTorques
geometry_msgs/TransformStamped[] frame_transforms
