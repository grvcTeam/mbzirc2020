std_msgs/Header header
geometry_msgs/PoseWithCovariance pose  # Object pose (in header.frame_id coordinates)
geometry_msgs/Vector3 scale            # Detected object size
uint8 type                             # Object type (see TYPE_* constants below)
uint8 sub_type                         # Object subtype (see SUBTYPE_* constants below)
uint8 color                            # Object color (see COLOR_* constats below)


uint8 TYPE_PILE    = 0
uint8 TYPE_WALL    = 1
uint8 TYPE_FIRE    = 2
uint8 TYPE_PASSAGE = 3

uint8 SUBTYPE_UAV = 0
uint8 SUBTYPE_UGV = 1
uint8 SUBTYPE_OUTFIRE = 2 
uint8 SUBTYPE_FACADEFIRE = 3 
uint8 SUBTYPE_INFIRE = 4

int8 COLOR_UNKNOWN = -1
int8 COLOR_RED     = 0
int8 COLOR_GREEN   = 1
int8 COLOR_BLUE    = 2
int8 COLOR_ORANGE  = 3
int8 NCOLORS = 4
