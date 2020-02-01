std_msgs/Header header
geometry_msgs/PoseWithCovariance pose  # Detected object pose (in header.frame_id coordinates)
                                       # Coordinate systems in the object center except for L wall.
                                       # Passage orientation should be pointing into the window. Y axis aligns with window width.
                                       # Bricks, x axis aligns with shortest dimension.
                                       # L-shaped wall. xy axis in the down-left corner, with the x east and the y north.
                                       # U-channel: orientation between [-pi/2,pi/2]. Y axis aligns with longest channel size 
geometry_msgs/Point point_of_interest  # Detected point of interest (in header.frame_id coordinates)
geometry_msgs/Point relative_position  # DEPRECATED!
float32 relative_yaw                   # DEPRECATED!
geometry_msgs/Vector3 scale            # Detected object size
uint8 type                             # Detected object type (see TYPE_* constants below)
uint8 color                            # Detected object color (see COLOR_* constats below)
ThermalImage image_detection           # Detected object on image plane 
bool is_cropped

uint8 TYPE_UNKNOWN     = 0
uint8 TYPE_BALLOON     = 1
uint8 TYPE_BALL        = 2
uint8 TYPE_BRICK       = 4
uint8 TYPE_BRICK_TRACK = 5
uint8 TYPE_UCHANNEL    = 6
uint8 TYPE_LWALL       = 7
uint8 TYPE_FIRE        = 8
uint8 TYPE_HOLE        = 9
uint8 TYPE_PASSAGE     = 10

int8  BRICK_COLORS = 4
uint8 COLOR_UNKNOWN = 0
uint8 COLOR_RED     = 1
uint8 COLOR_GREEN   = 2
uint8 COLOR_BLUE    = 3
uint8 COLOR_ORANGE  = 4
uint8 COLOR_WHITE   = 5
uint8 COLOR_FIRE    = 6
