## Ros2 message with policy information for a behavior

# Sentence for speak behavior
string sentence

# marker_id on the dock at which needs to be docked
int64 dock_marker_id -1

# distance when approaching or moving along an object. Note, that this refers to the base-link of the robot in the (negative) x-direction.
# As such, when you want the nose of the Origin robot to just hit the object, then object_distance = 0.5.
float64 object_distance 1.0

# name of the controller (in controller server) to use
string controller_name

# speed limits
float64 min_speed 0.0
float64 max_speed 1.0

# elevation limits
float64 min_height
float64 max_height

# maximum allowed distance from the path (negative values indicate infinity)
float64 max_path_distance -1.0

# A max traveled distance in meters
float64 travel_distance -1

# path offset from the line to follow in meters
float64 offset -1

# side at which the offset is meant
uint8 LEFT_OF_ROBOT=0
uint8 RIGHT_OF_ROBOT=1

uint8 keep_at_side

# Infill policy
complete_coverage_planner_msgs/InfillPolicy infill_policy

# add custom policy field
string custom_policies
