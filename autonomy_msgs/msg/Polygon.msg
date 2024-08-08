# Unique generated id
string uuid
# Human readable name
string name

# Coordinate system used for points
autonomy_msgs/CoordinateSystemInfo info
# Points in polygon
# Note, while these are 3d points, they should always be on the same plane.
# It will be up to the algorithm whether this is the xy plane, or something else.
geometry_msgs/Point[] points

# Whether it is a right sided polygon. 
# If it is, if you define them clockwise, it will describe the area inside.
bool right_sided true
