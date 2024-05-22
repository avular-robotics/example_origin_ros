## Ros2 message that holds global costmap information 
## TODO e.nikolla: Discuss about the format of this message (https://avular.atlassian.net/browse/SPANAR-1367)

# Standard metadata header
std_msgs/Header  header
# This is the LocalTangentPlane
autonomy_msgs/CoordinateSystemInfo coord_info
nav_msgs/MapMetaData map_info
uint8[] data
