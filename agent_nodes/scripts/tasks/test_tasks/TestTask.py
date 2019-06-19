import roslib
import rospy
import smach
import smach_ros

from utils.geom import *
from utils.agent import *

# required message definitions
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from std_msgs.msg import String

# task properties
ResponseType = SetBoolResponse
DataType = SetBool
transitions={'success':'success','error':'error'}

# function to create userdata from a task execution request matching the task
# input keys
def gen_userdata(req):

    userdata = smach.UserData()
    userdata.isWrite = req.data
    return userdata

# Task. At initialization it adds required elements to the AgentInterface
# At execution it uses these elements to coordinate components
class Task(smach.State):

    #callbacks
    def set_message(self, msg):
        if self.isWrite:
            rospy.loginfo('setting message to {msg}!'.format(msg=msg.data))
            self.msg = msg.data
        else:
            rospy.loginfo('sorry, I am in read mode')

    def get_message(self, req):
        return TriggerResponse(success=True,message=self.msg)

    def get_message_inactive(self, req):
        return TriggerResponse(success=False,message='There is no active Task attending this service')

    #init
    def __init__(self, name, interface):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['isWrite'])

        #members
        self.msg = 'not set'
        self.isWrite = False
        self.name = name

        #interface elements
        interface.add_publisher('pub_message','my_message',String,1)
        interface.add_subscriber(self,'set_message',String,self.set_message)
        interface.add_server(self,'get_message',Trigger,self.get_message,self.get_message_inactive)

        self.iface = interface

    #main function
    def execute(self, userdata):
        self.isWrite = userdata.isWrite

        r = rospy.Rate(1)
        ctr = 0
        while ctr < 10:
            ctr += 1
            self.iface['pub_message'].publish(String(data=self.msg))
            r.sleep()

        return 'success'
