# Data from Thermal Camera Detection
uint8 camera_direction
int height  # image resolution
int width   # image resolution
float32 x_center
float32 y_center
float32 height_from_center
float32 width_from_center
float32 depth  # estimated depth (e.g: from laser data)

uint8 CAMERA_DIRECTION_UNKNOWN  = 0
uint8 CAMERA_DIRECTION_FORWARD  = 1
uint8 CAMERA_DIRECTION_DOWNWARD = 2
