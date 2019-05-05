#!/usr/bin/env python
import roslib
import rospy
import smach
import smach_ros
import tf2_ros

from agent_nodes.msg import ExecTask

#handles an agent interface. Stores elements in the interface with the particularity
#that callbacks depends on the active task in the agent fsm
class AgentInterface():

    def __init__(self, agent_id, agent_fsm):

        self.agent_id = agent_id
        self.fsm = agent_fsm
        self.callables = {}
        self.callbacks = {}
        self.subscribers = {}
        self.servers = {}

        #init
        self.callables['tf_buffer'] = tf2_ros.Buffer()
        self.callables['tf_listener'] = tf2_ros.TransformListener(self.callables['tf_buffer'])
        self.callables['exec_task'] = rospy.Publisher(self.agent_id+'/'+'exec_task', ExecTask, queue_size=1)

    def __getitem__(self,item):

        if not item in self.callables:
            raise ValueError('{agent_id} AgentInterface does not have an element {element}'.format(agent_id=self.agent_id,element=item))
        else:
            return self.callables[item]

    def get_active_states(self, task):
        actives = []
        a_ids = []
        if isinstance(task,smach.StateMachine):
            actives = task.get_active_states()
            sub_actives = []
            sub_ids = []
            for active in actives:
                try:
                    a_ids += [id(task[active])]
                    a = self.get_active_states(task[active])
                    sub_actives += a[0]
                    sub_ids += a[1]
                except Exception as error:
                    print repr(error)
                    print 'Error getting state: {active}'.format(active=active)

            actives += sub_actives
            a_ids += sub_ids
        if hasattr(task, 'get_active_subtask') and task.get_active_subtask():
            a = self.get_active_states(task.get(task.get_active_subtask()))
            actives += [task.get_active_subtask()] + a[0]
            a_ids += [id(task.get(task.get_active_subtask()))] + a[1]

        return actives, a_ids

    def add_publisher(self,name,topic_name, data_class, queue_size):

        if not name in self.callables:
            self.callables[name] = rospy.Publisher(topic_name, data_class, queue_size=queue_size)
        else:
            rospy.logdebug('An element with name {name} already exists in {agent_id} AgentInterface'.format(agent_id=self.agent_id,name=name))

    def add_client(self,name,topic_name, data_class):

        if not name in self.callables:
            #print topic_name
            rospy.wait_for_service(topic_name,20.0) #TODO: waiting time should be a param
            self.callables[name] = rospy.ServiceProxy(topic_name, data_class)
        else:
            rospy.logdebug('An element with name {name} already exists in this AgentInterface'.format(name=name))

    def add_subscriber(self, task, topic_name, data_class, callback):

        if not topic_name in self.callbacks:
            def meta_callback(msg):
                if self.fsm.isInit:
                    actives = self.get_active_states(self.fsm)
                    #print 'ACTIVES: {actives}'.format(actives=actives)
                    for active in actives[1]:
                        if active in self.callbacks[topic_name]:
                            self.callbacks[topic_name][active](msg)

            self.callbacks[topic_name] = {}
            self.callbacks[topic_name]['meta'] = meta_callback
            self.subscribers[topic_name] = rospy.Subscriber(topic_name, data_class, meta_callback)

        self.callbacks[topic_name][id(task)] = callback


    def add_server(self, task, topic_name, data_class, callback, inactive_callback):

        if not topic_name in self.callbacks:
            def meta_callback(req):
                if self.fsm.isInit:
                    actives = self.get_active_states(self.fsm)
                    #print 'ACTIVES: {actives}'.format(actives=actives)
                    for active in actives[1]:
                        if active in self.callbacks[topic_name]:
                            return self.callbacks[topic_name][active](req) #TODO: naively tackling the possibility of paralel Tasks offering the same service

                    return self.callbacks[topic_name]['inactive'](req)

            self.callbacks[topic_name] = {}
            self.callbacks[topic_name]['meta'] = meta_callback
            self.callbacks[topic_name]['inactive'] = inactive_callback
            self.servers[topic_name] = rospy.Service(topic_name, data_class, meta_callback)

        self.callbacks[topic_name][id(task)] = callback

#Runs in a loop until a request to execute a new task arrives
class ExecTaskWatch(smach.State):
    def exec_task_cb(self,msg):
        self.task = msg.task_id
        self.request_preempt()
    def __init__(self, agent_id, task_list):
        self.task = ''
        self.task_list = task_list
        self.sub = rospy.Subscriber(agent_id+'/'+'exec_task', ExecTask, self.exec_task_cb)
        smach.State.__init__(self, outcomes=task_list+['invalid_task'])
    def execute(self, userdata):
        #initialize
        self.recall_preempt()
        #watch for transition
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                return self.task if self.task in self.task_list else 'invalid_task'
            r.sleep()

#called if a child state terminates
def child_term_cb(outcome_map):
    return True

#concurrent outcome map
def out_cb(outcome_map):
    if outcome_map['DEFAULT_TASK'] == 'error':
        return 'error'
    else:
        return outcome_map['EXEC_TASK_WATCH']

#default task container running in paralel the execute task watcher
class DefaultTaskContainer(smach.Concurrence):

    '''Parameters:
        - default_task: the task the agents execute on idle
        - task_list: names of the tasks the agent can execute'''
    def __init__(self, agent_id, default_task, task_list):
        smach.Concurrence.__init__(self,input_keys=list(default_task._input_keys),outcomes=task_list+['error','invalid_task'],
                                   default_outcome='error',child_termination_cb=child_term_cb,
                                   outcome_cb=out_cb)
        with self:
            smach.Concurrence.add('EXEC_TASK_WATCH', ExecTaskWatch(agent_id, task_list))
            smach.Concurrence.add('DEFAULT_TASK', default_task)

#Wraps a Task so the agent can expose it for external execution
#Ensures it has the right outcomes: success, error
class AgentTaskWrapper(smach.StateMachine):

    '''Parameters:
        - task: agent task to be wrapped. Can be a state or a state machine.
        - transitions: dic containing the transitions from the wrapped task outcomes
          to 'success' and 'error' '''
    def __init__(self, name, task, transitions):
        smach.StateMachine.__init__(self,outcomes=['success','error'])

        with self:
            smach.StateMachine.add(name, task, transitions)

#TODO: maybe implement DefaultTask class??

#Agent sm transiting from requested tasks to default task
class AgentStateMachine(smach.StateMachine):

    def __init__(self):
        self.isInit = False

    '''Parameters:
        - default_task: the task the agents execute on idle
        - tasks_dic: names and objects of the tasks the agent can execute'''
    def initialize(self, agent_id, default_task, tasks_dic):
        #TODO: 'error' outcome here simbolizes a critical recovery task, which needs to be an actual
        #state in the statemachine, same as default
        smach.StateMachine.__init__(self,outcomes=['error'],
                                    input_keys=list(default_task._input_keys))

        default = DefaultTaskContainer(agent_id, default_task, tasks_dic.keys())
        default_trans = {'invalid_task':'DEFAULT','error':'error'}
        for task in tasks_dic.keys():
            default_trans[task]=task

        with self:
            smach.StateMachine.add('DEFAULT', default, transitions=default_trans)
            for task in tasks_dic.keys():
                smach.StateMachine.add(task, tasks_dic[task],
                        transitions={'success':'DEFAULT','error':'error'})

        self.isInit = True

    def execute(self, parent_ud=smach.UserData()):
        if self.isInit:
            smach.StateMachine.execute(self,parent_ud)
        else:
            rospy.logerr('Agent State Machine must be initialized before execution')

#Add a task to a tasks dictionary to initialize an AgentFSM
def add_task(name, tasks_dic, interface, task, task_args = []):

    if not hasattr(task, 'Task') or not hasattr(task, 'DataType') or \
        not hasattr(task, 'ResponseType') or not hasattr(task, 'gen_userdata'):
        raise AttributeError('{task} is missing required attribute/s and cannot '\
        'be instantiated. Required attributes are: Task, DataType, ResponseType, '\
        'gen_userdata'.format(task=task))

    instance = task.Task(name, interface, *task_args)
    wrapper = AgentTaskWrapper(name, instance, task.transitions)
    tasks_dic[name] = wrapper

    def cb(req):
        wrapper.userdata = task.gen_userdata(req)
        if 'DEFAULT' in interface.fsm.get_active_states():
            interface['exec_task'].publish(
                    ExecTask(agent_id=interface.agent_id,task_id=name))

            return task.ResponseType(success=True) #TODO: add estimated time to execute Task
        else:
            rospy.logwarn('Agent {agent_id} cannot execute task {name}. Currently executing: {active}'.format(
            agent_id=interface.agent_id,name=name,active=interface.fsm.get_active_states()))
            return task.ResponseType(success=False)

    return rospy.Service(interface.agent_id+'/'+name, task.DataType, cb)

#add a subtask to a task that can be later called
def add_sub_task(name, parent_task, child_task, task_args = []):

    # create task
    if not hasattr(parent_task, 'sub_tasks'):
        parent_task.sub_tasks = {}

    parent_task.sub_tasks[name] = child_task.Task(name,parent_task.iface,*task_args)

    # add calling method
    if not hasattr(parent_task, 'active_task'):
        parent_task.active_task = ""

    if not hasattr(parent_task, 'call_task'):
        def call_task(name, userdata):
            if name in parent_task.sub_tasks:
                parent_task.active_task = name
                parent_task.sub_tasks[name].execute(userdata)
                parent_task.active_task = ""
                return True
            else:
                rospy.logwarn('{parent} task does not have subtask named {child}'.format(parent=parent_task.name, child=name))
                return False

        parent_task.call_task = call_task

    # get active subtask
    if not hasattr(parent_task, 'get_active_subtask'):
        def get_active_subtask():
            return parent_task.active_task

        parent_task.get_active_subtask = get_active_subtask

    if not hasattr(parent_task, '__getitem__'):

        def getitem(item):

            if item in parent_task.sub_tasks:
                return parent_task.sub_tasks[item]
            else:
                return None

        #parent_task.__getitem__ = getitem #TODO: why is this not working?
        parent_task.get = getitem
