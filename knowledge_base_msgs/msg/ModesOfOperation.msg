## Ros2 message with the modes of operation that can be set via the information manager for the autopilot stack.
## This doesn't represent all the modes of operation of the autopilot.


uint8 DEFAULT_MODE = 0
uint8 MAPPING_MODE = 1
uint8 MULTIMAP_MODE = 2

# Type of mode (see num-list above)
uint8 mode_of_operation
