## Ros2 message with constraint information for a behavior

# Path constraint(s) on their list of poses and if to be taken in reverse
string path_uuid
bool reverse

# NoGo area constraint(s)
string[] nogo_area_uuid

# Go area constraint(s)
string[] go_area_uuid

# Time constraint
float64 time 0.0