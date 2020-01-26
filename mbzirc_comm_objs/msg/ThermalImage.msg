# Data from Thermal Camera Detection
uint8 camera_direction
uint8 img_height            # image resolution (pixels)
uint8 img_width             # image resolution (pixels)
float32 u                   # column in pixels
float32 v                   # row in pixels
float32 height              # ROI height in pixels
float32 width               # ROI width in pixels
float32 depth               # Depth in meters (e.g: from laser data)

uint8 CAMERA_DIRECTION_UNKNOWN  = 0
uint8 CAMERA_DIRECTION_FORWARD  = 1
uint8 CAMERA_DIRECTION_DOWNWARD = 2
