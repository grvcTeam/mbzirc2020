#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
from std_srvs.srv import SetBool, SetBoolResponse
from mbzirc_gazebo_plugins.msg import SimDroplet
from geometry_msgs.msg import PoseStamped

class FireExtiguisher(object):

    def __init__(self):
        self.enabled = False
        self.droplets_left = 10  # TODO: from param
        self.droplets_velocity = 0.2  # TODO: from params?
        rospy.Service('enable_fire_extiguisher', SetBool, self.enable_callback)
        self.droplet_pub = rospy.Publisher('/spawn_droplet', SimDroplet, queue_size=1)
        self.update_duration = rospy.Duration(1.0)
        rospy.Timer(self.update_duration, self.update_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.ns = rospy.get_namespace()

    def enable_callback(self, req):
        self.enabled = req.data
        return SetBoolResponse(success = True)

    def update_callback(self, event):
        if not self.enabled or self.droplets_left <= 0:
            return
        # else, it is enabled and there are droplets left:
        sim_droplet = SimDroplet()
        extinguisher_origin = PoseStamped()
        extinguisher_origin.header.frame_id = self.ns[1:] + 'extinguisher_link'  # remove first '/' from ns
        extinguisher_origin.header.stamp = rospy.Time.now()  # WATCHOUT!
        try:
            extinguisher_origin = self.tf_buffer.transform(extinguisher_origin, 'map', rospy.Duration(1.0))
            # print(extinguisher_origin)
        except:
            rospy.logerr('Failed to transform waypoint from [{}] to [{}]'.format(extinguisher_origin.header.frame_id, 'map'))

        sim_droplet.position = extinguisher_origin.pose.position
        # sim_droplet.velocity.x = self.droplets_velocity  # TODO: velocity also needs to be transformed
        self.droplets_left -= 1
        self.droplets_velocity *= 0.99  # Some kind of decay law
        self.droplet_pub.publish(sim_droplet)

if __name__ == '__main__':
    rospy.init_node('fire_extigusher', anonymous = True)
    FireExtiguisher()
    rospy.spin()
