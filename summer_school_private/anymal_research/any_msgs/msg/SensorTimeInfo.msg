# Time information for a single sensor measurement
# e.g. pointcloud, IMU, thermal image...

# Message header including timestamp
std_msgs/Header header

# Monotonic counter for the measurement
uint64 counter

# The amount of time in microsends to make the measurement
# e.g. Image exposure
uint64 duration			