# ISO-9283 compliant pose measurement message

# Test identification
int32 pose_id          # Pose identifier (1-5 for ISO test)
int32 repetition       # Repetition number (1-30)
string pose_label      # Optional label (e.g., "P1", "CENTER")

# Commanded pose (from robot controller)
geometry_msgs/Pose commanded_pose

# Measured pose (from external measurement system)
geometry_msgs/Pose measured_pose

# Measurement metadata
builtin_interfaces/Time timestamp
string measurement_system   # e.g., "laser_tracker", "motion_capture"
string measurement_frame    # Frame ID of measurement
float64 measurement_quality # Quality metric (0-1)