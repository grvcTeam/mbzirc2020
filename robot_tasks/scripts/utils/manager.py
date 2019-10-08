import threading
import rospy


class ThreadWithReturnValue(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs={}, Verbose=None):
        threading.Thread.__init__(self, group, target, name, args, kwargs, Verbose)
        self._return = None

    def run(self):
        if self._Thread__target is not None:
            self._return = self._Thread__target(*self._Thread__args, **self._Thread__kwargs)

    # As ONLY __init__() y run() should be overriden
    def get_return_value(self):
        return self._return


class TaskManager(object):
    def __init__(self, robot_interfaces):
        self.robots = robot_interfaces
        self.idle = {}
        self.locks = {}
        self.tasks = {}
        self.threads = {}

        for robot_id in self.robots:
            self.locks[robot_id] = threading.Lock()
            self.idle[robot_id] = threading.Event()
            self.idle[robot_id].set()

        self.manage_thread = threading.Thread(target = self.manage_tasks)
        self.manage_thread.daemon = True
        self.manage_thread.start()

    def manage_tasks(self):
        rate = rospy.Rate(1)  # [Hz]
        while not rospy.is_shutdown():
            for robot_id, thread in self.threads.items():
                self.locks[robot_id].acquire()
                if not self.idle[robot_id].is_set() and not thread.is_alive():
                    thread.join()
                    outcome = thread.get_return_value()
                    rospy.logwarn('task on robot[{}] finished with output: {}'.format(robot_id, outcome))
                    del self.threads[robot_id]  # TODO: needed?
                    del self.tasks[robot_id]    # TODO: needed?
                    self.idle[robot_id].set()
                self.locks[robot_id].release()
            rate.sleep()

    def start_task(self, robot_id, task, userdata):
        # TODO: Force robot_id field inside task instead?
        with self.locks[robot_id]:
            if not self.idle[robot_id].is_set():
                rospy.logerr('robot {} is not idle!'.format(robot_id))
                return False
            self.idle[robot_id].clear()
            self.tasks[robot_id] = task
            self.tasks[robot_id].define_for(self.robots[robot_id])
            self.threads[robot_id] = ThreadWithReturnValue(target = task.execute, args = (userdata,))
            self.threads[robot_id].daemon = True
            self.threads[robot_id].start()
        return True

    def preempt_task(self, robot_id):
        if robot_id not in self.tasks:
            rospy.logerr('Invalid robot_id [{}]'.format(robot_id))
            return False
        rospy.logwarn('Trying to preempt current task on robot[{}]'.format(robot_id))
        self.tasks[robot_id].request_preempt()
        return True

    def is_idle(self, robot_id):
        return self.idle[robot_id].is_set()

    def are_idle(self, id_list):
        for robot_id in id_list:
            if not self.idle[robot_id].is_set():
                return False
        return True

    def wait_for(self, id_list):
        # TODO: Add timeout?
        while not self.are_idle(id_list):
            rospy.sleep(0.2)  # TODO: tune
