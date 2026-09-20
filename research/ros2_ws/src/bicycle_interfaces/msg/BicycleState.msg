std_msgs/Header header
uint64 sequence
# x m, y m, yaw rad, speed m/s, LEFT lean rad, LEFT lean rate rad/s, steering rad.
float64[7] state
# Row-major covariance of the above state. Ground truth uses zero covariance.
float64[49] covariance
float64 gnss_age
