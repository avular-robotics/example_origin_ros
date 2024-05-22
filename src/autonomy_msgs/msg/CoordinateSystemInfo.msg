uint8 LOCAL_RELATIVE=0
uint8 LOCAL_TANGENT_PLANE=1
uint8 UTM=2
uint8 UNKNOWN=3

# Type of coordinate system used, default value is UNKNOWN
uint8 type 3
# Local relative only
string map_name

# Local tangent plane only
float64 latitude
float64 longitude
float64 altitude

# UTM only
# Zone can be 0 to 60 (-1 is default/unknown)
int8 zone -1
bool north true
float64 x_offset
float64 y_offset
float64 z_offset
