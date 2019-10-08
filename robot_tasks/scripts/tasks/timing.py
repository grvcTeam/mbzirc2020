import rospy
import smach
import smach_ros


class SleepAndRetry(smach.State):
    def __init__(self, duration = 3.0, max_retries = None):
        smach.State.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'])
        self.duration = duration
        self.max_retries = max_retries
        self.retry_count = 0

    def define_for(self, robot):
        return self

    def execute(self, userdata):
        rospy.sleep(self.duration)
        # TODO: Sleep in shorter period chunks to allow faster preemption?
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if not self.max_retries:
            return 'succeeded'
        elif self.retry_count < self.max_retries:
            self.retry_count += 1
            return 'succeeded'
        else:
            return 'aborted'
