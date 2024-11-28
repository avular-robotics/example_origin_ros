# This message is used to communicate a control mode for the robot. 
# It represents the desired or current control source being forwarded to the robot.

# Constants
# If other modes are desired, they should be added below.

# Node specific modes (0X)
# No active control
uint8 NONE=00

# Manual control modes (1X)
uint8 MANUAL=10
uint8 TELEOPS=11
uint8 MANUAL_MAPPING=12

# Autonomous control modes (2X)
uint8 AUTOPILOT=20
uint8 DOCKING=21

# User control modes (3X)
uint8 USER=30

# Actual mode
uint8 mode 0