import rospy
import smach
import smach_ros
import mbzirc_comm_objs.msg as msg

from move import GoTo
from utils.translate import color_from_int
from geometry_msgs.msg import PoseStamped


# TODO: move to wall utils?
def all_piles_are_found(piles):
    # TODO: Check not only count, but also size of piles
    return len(piles) >= 4


class SearchPiles(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['path'], output_keys = ['piles'])
        self.go_to_task = None
        self.piles = {}
        rospy.Subscriber("estimated_objects", msg.ObjectDetectionList, self.estimation_callback)

    def define_for(self, robot):
        self.go_to_task = GoTo().define_for(robot)
        return self

    # TODO: This is repeated in central unit
    def estimation_callback(self, data):
        for pile in data.objects:
            # TODO: check type and scale?
            color = color_from_int(pile.color)
            pose = PoseStamped()
            pose.header = pile.header
            pose.pose = pile.pose.pose
            self.piles[color] = pose

    def execute(self, userdata):
        userdata.piles = self.piles
        for waypoint in userdata.path:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            if all_piles_are_found(self.piles):
                return 'succeeded'
            child_userdata = smach.UserData()
            child_userdata.waypoint = waypoint
            self.go_to_task.execute(child_userdata)
        return 'succeeded' if all_piles_are_found(self.piles) else 'aborted'

