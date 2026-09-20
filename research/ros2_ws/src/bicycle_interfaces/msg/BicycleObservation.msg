# SIMULATION ONLY. Acquisition stamp; local Cartesian map frame. Not raw IMU.
std_msgs/Header header
uint64 sequence
# Command applied over the interval ending at header.stamp: steering rad, accel m/s^2.
float64[2] applied_command
# yaw rad, speed m/s, LEFT lean rad, LEFT lean rate rad/s, LEFT steering rad.
float64[5] fast
float64[5] variance
# 5 Hz synthetic LOCAL METRIC GNSS position; never latitude/longitude.
bool gnss_valid
float64[2] gnss_position
