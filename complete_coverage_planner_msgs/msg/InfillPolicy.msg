# Constant value that indicates that a float value is unset.
float32 UNSET_FLOAT_VALUE=-3.4e+38

# The number of contours to plan from the outline of the coverage area inwards. If set to -1, the number of contours is determined by the configuration.
# NOTE: Currently infill contours are not supported. If this value is set to any other value than -1 it will cause errors and potentially crashes
int32 number_contours -1

# The number of contours to plan from the outline of the coverage area outwards. If set to -1, the number of outer contours is determined by the configuration.
int32 number_outside_contours -1

# Whether to complete the coverage area with a linear fill pattern. If set to -1, the use of linear infill is determined by the configuration.
int8 use_linear_infill -1

# The direction [rad] of the lines in the linear fill pattern. If set to NaN, the direction is determined by the configuration.
float32 line_direction -3.4e+38

# Whether to use the map for planning. If set to -1, the use of the map is determined by the configuration.
int8 use_map -1