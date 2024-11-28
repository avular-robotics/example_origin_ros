## Ros2 message with polygon information
# Polygon types
string GO_AREA = go_area
string NO_GO_AREA = no_go_area
string COVER_AREA = cover_area
string MAP_LAYOUT = map_layout

# Type of polygon, see the enum list above
string type

# UUID of the polygon
string uuid
# Name of the polygon
string name
# Actual data (x,y,z)
geometry_msgs/Polygon polygon
# UUID of the map, incase the polygon is a MAP_LAYOUT
string map_id
# Coordinate system of the polygon
autonomy_msgs/CoordinateSystemInfo coord_info
