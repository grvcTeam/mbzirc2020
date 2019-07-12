import roslib
import rospy
import smach
import smach_ros

from utils.geom import *
from utils.agent import *

# message definitions
from std_srvs.srv import SetBool, SetBoolResponse

# task properties. Just required to enable Task execution requests
# ie if it will be used as argument in utils.agent.add_task
ResponseType = SetBoolResponse
DataType = SetBool
transitions={'success':'success','error':'error'}

def gen_userdata(req):
    return smach.UserData()

#main class
class Task(smach.State):

    def __init__(self, name, interface):
        self.name = name
        smach.State.__init__(self,outcomes=['success','error'])

    def execute(self, userdata):
        self.recall_preempt()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                return 'success'
            r.sleep()

        return 'error'
