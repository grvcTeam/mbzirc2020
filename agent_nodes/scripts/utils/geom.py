import PyKDL #Vector, Rotation, Quaternion, Frame
import shapely.geometry #Point, Polygon
import shapely.ops # transform
import shapely.affinity
import geometry_msgs.msg #Pose, Transform, Polygon
import rospy

from math import tan, ceil, pi
#import matplotlib.pyplot as plt

#conversions
def from_geom_msgs_Transform_to_geom_msgs_Pose(transform):
    pos = geometry_msgs.msg.Point(transform.translation.x,transform.translation.y,transform.translation.z)
    rot = geometry_msgs.msg.Quaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
    return geometry_msgs.msg.Pose(pos,rot)

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
            rospy.logwarn("z component will be ignored in geom_msgs_Polygon_to_Shapely_Polygon transformation")
        points = points + [(point.x,point.y)]

    return shapely.geometry.Polygon(points)

#operations
def transform_Shapely_Polygon_with_KDL_Frame(transform,polygon):
    rot_angle = transform.M.GetRotAngle()
    if abs(rot_angle[1][2]) != 1:
        rospy.logwarn("just rotations in the Z axis are supported for polygon transformations: {r}".format(r=rot_angle))
        return polygon

    def poly_transform(x,y,z=None):
        p = transform * PyKDL.Vector(x,y,0)
        return p.x(),p.y()

    return shapely.ops.transform(poly_transform,polygon)

def lookup_tf_transform(parent_frame,child_frame,tf_buffer,n_tries=0,freq=2, time = None):
    ctr = 0
    rate = rospy.Rate(freq)
    while 1:
        try:
            t = time if time else rospy.Time()
            trans = tf_buffer.lookup_transform(parent_frame, child_frame, t)
            break
        except Exception as error:
        #except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            if ctr < n_tries:
                rate.sleep()
                ctr += 1
            else:
                raise error

    return trans

#compute a path to cover the bounding box of a polygon
def compute_search_path(aov, height, polygon, pos):

    #compute coverage square
    aov = aov * (pi/180.)
    r = (tan(aov/2) * height)
    lx = ly = 2 * r #TODO: typically aov takes different values in x and y directions
    rospy.loginfo('coverage size: {l}'.format(l=lx))

    #compute polygon bounds and path grid size
    bb = polygon.bounds
    #print bb
    Lx = bb[2] - bb[0]
    Ly = bb[3] - bb[1]

    nx = int(ceil(Lx/lx))
    ny = int(ceil(Ly/ly))

    #choose closest point in bb as first point in the path
    ex = [shapely.geometry.Point(bb[0],bb[1]),shapely.geometry.Point(bb[2],bb[1]),
        shapely.geometry.Point(bb[2],bb[3]),shapely.geometry.Point(bb[0],bb[3])]

    cl = (pos.distance(ex[0]),0)
    for i in range(len(ex)):
        d = pos.distance(ex[i])
        if d < cl[0]:
            cl = (d,i)

    #compute path
    if cl[1] == 0:
        x_plus = lambda x, y : x + y
        y_plus = lambda x, y : x + y
    elif cl[1] == 1:
        x_plus = lambda x, y : x - y
        y_plus = lambda x, y : x + y
    elif cl[1] == 2:
        x_plus = lambda x, y : x - y
        y_plus = lambda x, y : x - y
    elif cl[1] == 3:
        x_plus = lambda x, y : x + y
        y_plus = lambda x, y : x - y

    ox = x_plus(ex[cl[1]].x, r)
    oy = y_plus(ex[cl[1]].y, r)

    path = [(pos.x,pos.y)]
    for j in range(ny):
        r = reversed(range(nx)) if j%2 else range(nx)
        for i in r:
            path += [(x_plus(ox,lx*i),y_plus(oy,ly*j))]

    '''ln = shapely.geometry.LineString(path)
    ln2 = shapely.geometry.LineString(list(polygon.exterior.coords))
    ox -= lx/2
    oy -= ly/2
    ln3 = shapely.geometry.LineString([(ox,oy),(ox+lx,oy),(ox+lx,oy+ly),(ox,oy+ly),(ox,oy)])
    x, y = ln.xy
    plt.plot(x,y)
    x, y = ln2.xy
    plt.plot(x,y,'r')
    x, y = ln3.xy
    plt.plot(x,y,'g')
    plt.show()'''

    return path

# computes trajectory around region. Assuming convex region
def trajectory_around_region(p1,p2,region):
    #inflate region
    r = from_geom_msgs_Polygon_to_Shapely_Polygon(region).buffer(1,3)

    #find closest points to p1 and p2
    dp1 = (p1.distance(Point(r.exterior.coords[0])),0)
    dp2 = (p1.distance(Point(r.exterior.coords[0])),0)
    for i in range(len(r.exterior.coords)):
        d1 = p1.distance(Point(r.exterior.coords[i]))
        if d1 < dp1[0]:
            dp1 = (d1,i)
        d2 = p1.distance(Point(r.exterior.coords[i]))
        if d2 < dp2[0]:
            dp2 = (d2,i)

    #return shortest path
    rev = False
    if dp1[1] > dp2[1]:
        rev = True
        dp = dp1
        dp1 = dp2
        dp2 = dp

    l1 = LineString(r.exterior.coords[dp1[1]:dp2[2]+1])
    if l1.length < r.exterior.length / 2:
        s = list(l1.coords)
    else:
        s1 = list(reversed(r.exterior.coords[1:dp1[1]+1]))
        s2 = list(reversed(r.exterior.coords[dp1[2]:]))
        s = s1 + s2

    return s if not rev else list(reversed(s))


#pos = shapely.geometry.Point(40.,10.)
#pol = shapely.geometry.Polygon([(1,1),(31,1),(31,70),(1,70)])
#compute_search_path(45., 2., pol, pos)
