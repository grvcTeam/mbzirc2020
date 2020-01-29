std_msgs/Header header
geometry_msgs/PoseWithCovariance pose  # Detected object pose (in header.frame_id coordinates)
geometry_msgs/Point point_of_interest  # Detected point of interest (in header.frame_id coordinates)
geometry_msgs/Point relative_position  # DEPRECATED!
float32 relative_yaw                   # DEPRECATED!
geometry_msgs/Vector3 scale            # Detected object size
uint8 type                             # Detected object type (see TYPE_* constants below)
uint8 color                            # Detected object color (see COLOR_* constats below)
ThermalImage image_espec
bool is_cropped

uint8 TYPE_UNKNOWN = 0
uint8 TYPE_BALL    = 1
uint8 TYPE_DRONE   = 2
uint8 TYPE_BRICK   = 3
uint8 TYPE_FIRE    = 4
uint8 TYPE_PASSAGE = 5
uint8 TYPE_BRICK_TRACK = 6

uint8 COLOR_UNKNOWN = 0
uint8 COLOR_RED     = 1
uint8 COLOR_GREEN   = 2
uint8 COLOR_BLUE    = 3
uint8 COLOR_ORANGE  = 4
uint8 COLOR_WHITE   = 5
