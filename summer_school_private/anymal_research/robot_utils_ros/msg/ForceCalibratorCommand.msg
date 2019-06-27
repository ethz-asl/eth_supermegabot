# Command to start the calibration.
bool cmd_start

# Command to continue sampling. Note that the calibration needs to be started first.
bool cmd_continue

# Command to calibrate after the sampling process.
bool cmd_calibrate

# If true, the outlier detector is active
bool enable_outlier_detector

# Number of samples that should be sampled for the calibration.
uint32 num_samples

# Required minimal number of samples for the calibration.
uint32 num_good_samples