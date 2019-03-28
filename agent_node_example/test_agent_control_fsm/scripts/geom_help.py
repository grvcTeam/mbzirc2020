import PyKDL #Vector, Rotation, Quaternion, Frame
import shapely.geometry #Point, Polygon
import shapely.ops # transform
import geometry_msgs.msg #Pose, Transform, Polygon
import rospy

#conversions
def from_geom_msgs_Transform_to_KDL_Frame(transform):
    pos = PyKDL.Vector(transform.translation.x,transform.translation.y,transform.translation.z)
    rot = PyKDL.Rotation.Quaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
    return PyKDL.Frame(rot,pos)

def from_geom_msgs_Pose_to_KDL_Frame(transform):
    pos = PyKDL.Vector(transform.position.x,transform.position.y,transform.position.z)
    rot = PyKDL.Rotation.Quaternion(transform.orientation.x, transform.orientation.y, transform.orientation.z, transform.orientation.w)
    return PyKDL.Frame(rot,pos)

def from_KDL_Frame_to_geom_msgs_Pose(transform):
    pos = geometry_msgs.msg.Point(transform.p.x(),transform.p.y(),transform.p.z())
    r_quat = transform.M.GetQuaternion()
    rot = geometry_msgs.msg.Quaternion(r_quat[0], r_quat[1], r_quat[2], r_quat[3])
    return geometry_msgs.msg.Pose(pos,rot)

def from_KDL_Frame_to_geom_msgs_Transform(transform):
    pos = geometry_msgs.msg.Vector3(transform.p.x(),transform.p.y(),transform.p.z())
    r_quat = transform.M.GetQuaternion()
    rot = geometry_msgs.msg.Quaternion(r_quat[0], r_quat[1], r_quat[2], r_quat[3])
    return geometry_msgs.msg.Transform(pos,rot)

def from_geom_msgs_Transform_to_Shapely_Point(transform):
    return shapely.geometry.Point(transform.translation.x,transform.translation.y)

def from_geom_msgs_Pose_to_Shapely_Point(transform):
    return shapely.geometry.Point(transform.position.x,transform.position.y)

def from_geom_msgs_Polygon_to_Shapely_Polygon(polygon):
    points = []
    for point in polygon.points:
        if point.z != 0:
            print "WARNING: z component will be ignored in geom_msgs_Polygon_to_Shapely_Polygon transformation"
        points = points + [(point.x,point.y)]

    return shapely.geometry.Polygon(points)

#operations
def transform_Shapely_Polygon_with_KDL_Frame(transform,polygon):
    rot_angle = transform.M.GetRotAngle()
    if rot_angle[1][2] != 1:
        print "WARNING: just rotations in the Z axis are supported for polygon transformations"
        return polygon

    def poly_transform(x,y,z=None):
        p = transform * PyKDL.Vector(x,y,0)
        return p.x(),p.y()

    return shapely.ops.transform(poly_transform,polygon)

def lookup_tf_transform(parent_frame,child_frame,tf_buffer,n_tries=0,freq=2):
    ctr = 0
    rate = rospy.Rate(freq)
    while 1:
        try:
            trans = tf_buffer.lookup_transform(parent_frame, child_frame, rospy.Time())
            break
        except Exception as error:
        #except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            if ctr < n_tries:
                rate.sleep()
                ctr += 1
            else:
                raise error

    return trans
