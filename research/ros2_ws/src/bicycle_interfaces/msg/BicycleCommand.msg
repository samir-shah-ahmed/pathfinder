# Stamp and sequence of the state on which this command is based.
std_msgs/Header header
uint64 sequence
# steering rad, longitudinal acceleration m/s^2.
float64[2] command
uint64 valid_for_ns
bool enabled
string reason
