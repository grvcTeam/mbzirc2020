import copy
import rospy
import tf2_ros
import tf2_geometry_msgs
import mbzirc_comm_objs.msg as msg
import mbzirc_comm_objs.srv as srv

from geometry_msgs.msg import PoseStamped


class RobotProxy(object):
    def __init__(self, robot_id):
        self.id = robot_id
        self.url = 'mbzirc2020_' + self.id + '/'  # TODO: Impose ns: mbzirc2020!?
        # TODO: Unifying robot_model and namespace might be an issue for non homogeneous teams, 
        # but it is somehow forced by the way sensor topics are named in gazebo simulation (mbzirc2020)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pose = PoseStamped()
        rospy.Subscriber(self.url + 'data_feed', msg.RobotDataFeed, self.data_feed_callback)
        # time.sleep(3)  # TODO: allow messages to get in? wait for pose?
        self.home = PoseStamped()

    def set_home(self):
        self.home = copy.deepcopy(self.pose)

    def data_feed_callback(self, data):
        self.pose = data.pose

    # TODO: auto update with changes in self.pose? Not here?
    def build_request_for_go_to_region(self, final_pose, label = 'go_to', radius = 1.0):
        initial_pose = copy.deepcopy(self.pose)
        try:
            if initial_pose.header.frame_id != 'arena':
                initial_pose = self.tf_buffer.transform(initial_pose, 'arena', rospy.Duration(1.0))
            if final_pose.header.frame_id != 'arena':
                final_pose = self.tf_buffer.transform(final_pose, 'arena', rospy.Duration(1.0))
        except:
            rospy.logerr('Failed to transform points to [{}], ignoring!'.format('arena'))

        request = srv.AskForRegionRequest()
        request.agent_id = self.id
        request.label = label
        request.min_corner.header.frame_id = 'arena'
        request.min_corner.point.x = min(initial_pose.pose.position.x - radius, final_pose.pose.position.x - radius)
        request.min_corner.point.y = min(initial_pose.pose.position.y - radius, final_pose.pose.position.y - radius)
        request.min_corner.point.z = min(initial_pose.pose.position.z - radius, final_pose.pose.position.z - radius)
        request.max_corner.header.frame_id = 'arena'
        request.max_corner.point.x = max(initial_pose.pose.position.x + radius, final_pose.pose.position.x + radius)
        request.max_corner.point.y = max(initial_pose.pose.position.y + radius, final_pose.pose.position.y + radius)
        request.max_corner.point.z = max(initial_pose.pose.position.z + radius, final_pose.pose.position.z + radius)

        return request

    def build_request_for_vertical_region(self, center, label = 'vertical', radius = 1.0, z_min = 0, z_max = 25):  # TODO: max_z parameter?
        center_pose = copy.deepcopy(center)
        try:
            if center_pose.header.frame_id != 'arena':
                center_pose = self.tf_buffer.transform(center_pose, 'arena', rospy.Duration(1.0))
        except:
            rospy.logerr('Failed to transform points to [{}], ignoring!'.format('arena'))

        request = srv.AskForRegionRequest()
        request.agent_id = self.id
        request.label = label
        request.min_corner.header.frame_id = 'arena'
        request.min_corner.point.x = center_pose.pose.position.x - radius
        request.min_corner.point.y = center_pose.pose.position.y - radius
        request.min_corner.point.z = z_min
        request.max_corner.header.frame_id = 'arena'
        request.max_corner.point.x = center_pose.pose.position.x + radius
        request.max_corner.point.y = center_pose.pose.position.y + radius
        request.max_corner.point.z = z_max

        return request

    # TODO: Force raw points with no frame_id?
    def get_cost_to_go_to(self, waypoint_in):
        waypoint = copy.deepcopy(waypoint_in)
        # TODO: these try/except inside a function?
        try:
            waypoint = self.tf_buffer.transform(waypoint, self.pose.header.frame_id, rospy.Duration(1.0))  # TODO: check from/to equality
        except:
            rospy.logerr('Failed to transform waypoint from [{}] to [{}]'.format(waypoint.header.frame_id, self.pose.header.frame_id))

        delta_x = waypoint.pose.position.x - self.pose.pose.position.x
        delta_y = waypoint.pose.position.y - self.pose.pose.position.y
        delta_z = waypoint.pose.position.z - self.pose.pose.position.z
        manhattan_distance = abs(delta_x) + abs(delta_y) + abs(delta_z)
        return manhattan_distance
