## Ros2 message with behavior information
# Initialization behaviors X
uint8 INIT_DEFAULT = 1
uint8 INIT_MAPPING = 2
uint8 PRE_JOB = 3
# Navigation behaviors 1X
uint8 MOVE_THROUGH_POSES = 10
uint8 MOVE_ALONG_PATH = 11
uint8 MOVE_IN_DOCK = 12
uint8 MOVE_OUT_DOCK = 13
uint8 COVER_AREA = 16
# Sense behaviors 2X
uint8 READOUT_METER = 20
# Act behaviors 3X
uint8 SPEAK = 30
# Internal behaviors 4X
uint8 WAIT = 40

# What: Type of behavior (see enum-list above)
uint8 type

# UUID of behavior
string uuid

# Where: goal position(s) of the behavior
geometry_msgs/PoseStamped[] goals
string area_uuid

# How: policies and constriants of the behavior
knowledge_base_msgs/Constraint constraint
knowledge_base_msgs/Policy policy
