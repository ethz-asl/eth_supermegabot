string cam_name #camera name which is used as base topic for publishing
geometry_msgs/Pose 	T_CI #pose mapping a point expressed in imu frame into camera frame
string dist_model # distortion model
float64[] dist_coeff # distortion coefficients: pumb_bob (radtan), equi (equidistant)
string projection_model # projection model: pinhole, omni (omnidirectional)
float64[] projection_coeff # camera/lens coefficients, format dependon the model and is the same as kalir
float64[] principal_point # principal point in pixels
float64[] focal_length # focal lengths in pixels
int32 image_width # image width
int32 image_height # image height
