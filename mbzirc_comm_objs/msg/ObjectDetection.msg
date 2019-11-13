std_msgs/Header header
geometry_msgs/PoseWithCovariance pose  # Detected object pose (in header.frame_id coordinates)
geometry_msgs/Point relative_position  # Detected object relative position (w.r.t detector device, e.g: camera)
float32 relative_yaw                   # Detected object relative yaw (w.r.t detector device, e.g: camera)
geometry_msgs/Vector3 scale            # Detected object size
uint8 type                             # Detected object type (see TYPE_* constants below)
uint8 color                            # Detected object color (see COLOR_* constats below)

uint8 TYPE_UNKNOWN = 0
uint8 TYPE_BALLOON = 1
uint8 TYPE_BALL    = 2
uint8 TYPE_DRONE   = 3
uint8 TYPE_BRICK   = 4
uint8 TYPE_FIRE    = 5
uint8 TYPE_PASSAGE = 6

int8 COLOR_UNKNOWN = -1
int8 COLOR_RED     = 0
int8 COLOR_GREEN   = 1
int8 COLOR_BLUE    = 2
int8 COLOR_ORANGE  = 3
int8 NCOLORS = 4
