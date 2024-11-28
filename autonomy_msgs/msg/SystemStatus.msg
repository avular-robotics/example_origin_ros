# This message describes the statuses that will be reported by the user interface node.
# In case any of these are not the desired value, please refer to the autopilot user interface.

# This indicates whether the robot is fully booted and ready for user operations.
bool is_operational false

# This status reflects the current behavior status of the robot
string behavior_status "No information yet."

# This status indicates whether the robot has accurate positioning information that would allow it to autonomously navigate.
# The position_accuracy_status is either:
string UNKNOWN = "unknown" # Indicates that no position estimate is received yet
string INACCURATE_POSITION = "inaccurate_position" # indicates that a position is estimated but it is not accurate yet
string INACCURATE_HEADING = "inaccurate_heading" # indicates that an accurate position estimate is available, but no accurate heading
string ACCURATE = "accurate" # indicates that an accurate position and orientation are available
string position_accuracy_status "unknown"
