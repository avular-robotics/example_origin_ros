# GNSS status, a supplement to sensor_msgs/NavSatFix.
std_msgs/Header header

# GNSS fix type
int8 STATUS_NO_FIX=-1    # Unable to fix position
int8 STATUS_2D_FIX=0     # 2D fix
int8 STATUS_3D_FIX=1     # 3D fix
int8 STATUS_RTK_FLOAT=2  # RTK with floating solution, the RTK algorithm hasn’t found an acceptable fix solution (yet)
int8 STATUS_RTK_FIX=3    # RTK with fixed solution
int8 status

# Uncertainty of measurement, 1-sigma confidence
float32 horizontal_position_accuracy  # Horizontal position uncertainty (meters) expressed as a bivariate normal distribution, 1-sigma then corresponds to 39.4% confidence
float32 vertical_position_accuracy    # Vertical position uncertainty (meters) expressed as a univariate normal distribution, 1-sigma then corresponds to 68% confidence

# Satellites visible
uint8 satellites_visible # Number of satellites visible
uint8 satellites_used    # Number of satellites used