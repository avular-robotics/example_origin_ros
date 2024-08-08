# Indication of signal strength
# Following the table listed on the following website: https://wiki.teltonika-networks.com/view/Mobile_Signal_Strength_Recommendations

# >= -70dBm
int8 CELLULAR_STRENGTH_EXCELLENT    = 4

# -70dBm to -85dBm
int8 CELLULAR_STRENGTH_GOOD         = 3

# -86dBm to -100dBm
int8 CELLULAR_STRENGTH_FAIR         = 2

# < -100dBm
int8 CELLULAR_STRENGTH_POOR         = 1

# -110dBm
int8 CELLULAR_STRENGTH_DISCONNECTED = 0

int8 CELLULAR_STRENGTH_UNKNOWN      = -1
int8 cellular_strength

string cellular_operator

int8 WIFI_STRENGTH_EXCELLENT    = 3
int8 WIFI_STRENGTH_FAIR         = 2
int8 WIFI_STRENGTH_POOR         = 1
int8 WIFI_STRENGTH_DISCONNECTED = 0
int8 WIFI_STRENGTH_UNKNOWN      = -1
int8 wifi_strength

string wifi_ssid
string wifi_ip