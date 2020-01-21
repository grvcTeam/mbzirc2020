#!/usr/bin/env python
import rospy
import rospkg
import subprocess
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from mbzirc_comm_objs.msg import WallList

class FireSaver(object):

    def __init__(self):
        self.ual_pose = PoseStamped()
        self.sf11_range = Range()
        self.wall_list = WallList()
        self.has_pose = False
        self.has_range = False
        self.has_walls = False

        rospy.Subscriber('ual/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('sf11', Range, self.range_callback)
        rospy.Subscriber('walls', WallList, self.walls_callback)

        rospy.Service('save_fire', Trigger, self.save_callback)
        self.yaml_file = None
        self.fire_index = 0

    def set_file(self, yaml_file):
        self.yaml_file = yaml_file
        self.yaml_file.write('fires:\n\n')

    def has_communications(self):
        return self.has_pose and self.has_range and self.has_walls

    def pose_callback(self, data):
        self.ual_pose = data
        self.has_pose = True

    def range_callback(self, data):
        self.sf11_range = data
        self.has_range = True

    def walls_callback(self, data):
        self.wall_list = data
        self.has_walls = True

    def save_callback(self, req):
        if not self.yaml_file:
            error_msg = 'YAML file not specified!'
            rospy.logwarn(error_msg)
            return TriggerResponse(success = False, message = error_msg)

        fire_id = 'default_' + str(self.fire_index)
        self.fire_index = self.fire_index + 1

        print('Saving as: ' + fire_id + '\n')
        print(self.ual_pose)
        print('\n')
        print(self.sf11_range)
        print('\n')
        print(self.wall_list)
        print('\n')

        self.yaml_file.write('  - id: ' + fire_id + '\n')

        self.yaml_file.write('    pose_frame: ' + str(self.ual_pose.header.frame_id) + '\n')
        self.yaml_file.write('    pose: [' + str(self.ual_pose.pose.position.x)    + ', ' + \
                                             str(self.ual_pose.pose.position.y)    + ', ' + \
                                             str(self.ual_pose.pose.position.z)    + ', ' + \
                                             str(self.ual_pose.pose.orientation.x) + ', ' + \
                                             str(self.ual_pose.pose.orientation.y) + ', ' + \
                                             str(self.ual_pose.pose.orientation.z) + ', ' + \
                                             str(self.ual_pose.pose.orientation.w) + ']\n')

        self.yaml_file.write('    sf11_range: ' + str(self.sf11_range.range) + '\n')

        self.yaml_file.write('    walls_frame: ' + str(self.wall_list.header.frame_id) + '\n')
        self.yaml_file.write('    walls:\n')
        for wall in self.wall_list.walls:
            self.yaml_file.write('      - [' + str(wall.start[0]) + ', ' + \
                                               str(wall.start[1]) + ', ' + \
                                               str(wall.end[0])   + ', ' + \
                                               str(wall.end[1])   + ']\n')
        self.yaml_file.write('\n')

        success_msg = 'Saved fire as ' + fire_id
        return TriggerResponse(success = True, message = success_msg)

if __name__ == '__main__':
    rospy.init_node('save_fire', anonymous = True)
    fire_saver = FireSaver()

    # Saved fires file name
    file_name = raw_input("Enter saved fires file name (leave empty to use default): ")
    if file_name is '':
        file_name = 'fire_default.yaml'

    # Autocomplete file extension
    if not file_name.endswith('.yaml'):
        file_name = file_name + '.yaml'

    # Assure that fires folder exists in plan_package
    fires_dir = rospkg.RosPack().get_path('fire_extinguisher') + '/fires/'
    subprocess.call("mkdir -p " + fires_dir, shell=True)

    # Open yaml file and save frame_id
    file_url = fires_dir + file_name
    yaml_file = open(file_url, 'w')
    fire_saver.set_file(yaml_file)
    rospy.loginfo('Fires will be saved to: %s', file_url)

    # Assure we have communications
    while not fire_saver.has_communications():
        rospy.logwarn('save_fire: Unable to stablish communication, retrying...')
        rospy.sleep(1)

    rospy.loginfo('Ready! Call trigger service on [save_fire] to save current fire')
    rospy.spin()

    yaml_file.close()
