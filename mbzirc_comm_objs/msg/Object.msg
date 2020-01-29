std_msgs/Header header
geometry_msgs/PoseWithCovariance pose  # Object pose (in header.frame_id coordinates)
geometry_msgs/Vector3 scale            # Detected object size
uint8 type                             # Object type (see TYPE_* constants below)
uint8 sub_type                         # Object subtype (see SUBTYPE_* constants below)
uint8 color                            # Object color (see COLOR_* constats below)
uint16 id                              # Object identifier


uint8 TYPE_PILE    = 0
uint8 TYPE_WALL    = 1
uint8 TYPE_FIRE    = 2
uint8 TYPE_PASSAGE = 3

int8 SUBTYPE_UNKNOWN = -1
int8 SUBTYPE_UAV = 0
int8 SUBTYPE_UGV = 1
int8 SUBTYPE_OUTFIRE = 2 
int8 SUBTYPE_FACADEFIRE = 3 
int8 SUBTYPE_INFIRE = 4
int8 SUBTYPE_GROUND = 5 
int8 SUBTYPE_FIRST = 6 
int8 SUBTYPE_SECOND = 7

