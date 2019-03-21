import PyKDL #Vector, Rotation, Quaternion, Frame
import shapely.geometry #Point, Polygon
import shapely.ops # transform
import geometry_msgs.msg #Pose, Transform, Polygon

#conversions
def from_geom_msgs_Transform_to_KDL_Frame(transform):
    pos = PyKDL.Vector(transform.translation.x,transform.translation.y,transform.translation.z)
    rot = PyKDL.Rotation.Quaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
    return PyKDL.Frame(rot,pos)

def from_geom_msgs_Pose_to_KDL_Frame(transform):
    pos = PyKDL.Vector(transform.position.x,transform.position.y,transform.position.z)
    rot = PyKDL.Rotation.Quaternion(transform.orientation.x, transform.orientation.y, transform.orientation.z, transform.orientation.w)
    return PyKDL.Frame(rot,pos)

def from_geom_msgs_Transform_to_Shapely_Point(transform):
    return shapely.geometry.Point(transform.translation.x,transform.translation.y)

def from_geom_msgs_Pose_to_Shapely_Point(transform):
    return shapely.geometry.Point(transform.position.x,transform.position.y)

def from_geom_msgs_Polygon_to_Shapely_Polygon(polygon):
    points = []
    for point in polygon.points:
        if point.z != 0:
            print "WARNING: z component will be ignored in geom_msgs_Polygon_to_Shapely_Polygon transformation"
        points = points + [(point.x,point.y,point.z)]

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
