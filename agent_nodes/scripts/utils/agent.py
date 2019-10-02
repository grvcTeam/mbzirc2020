#!/usr/bin/env python
import roslib
import rospy
import smach
import smach_ros
import tf2_ros
from pydispatch import dispatcher
import json
import traceback

from mbzirc_comm_objs.srv import AgentIdle, AgentIdleResponse, GetJson, GetJsonResponse, SetAgentProp, SetAgentPropResponse

#handles an agent interface. Stores elements in the interface with the particularity
#that callbacks depends on the active task in the agent fsm
class AgentInterface():

    def __init__(self, agent_id, agent_fsm, agent_props = {}, graph_change_cb = None):

        self.agent_id = agent_id
        self.fsm = agent_fsm
        self.agent_props = agent_props #TODO: now properties are stored as 'name':value pairs in a dict. For easing setting them from a external service datatype is also needed

        # members of the agent interface
        self.callables = {}
        self.callbacks = {}
        self.subscribers = {}
        self.servers = {}

        # elements related with assessing active task
        self.isIdle = True
        agent_fsm.call_transition_cbs = self.check_idle

        #init
        self.callables['tf_buffer'] = tf2_ros.Buffer()
        self.callables['tf_listener'] = tf2_ros.TransformListener(self.callables['tf_buffer'])

        self.callables['graph_list'] = tf2_ros.TransformListener(self.callables['tf_buffer'])
        self.add_client('agent_list','/agent_list', GetJson)

        if graph_change_cb:
            self.add_subscriber(self, 'AGENT', '/changes', String, graph_change_cb)

        #services
        self.idle_server = rospy.Service(self.agent_id+'/'+'is_idle', AgentIdle, self.is_idle_cb)
        self.get_props_server = rospy.Service(self.agent_id+'/'+'agent_props', GetJson, self.get_props_cb)
        self.set_props_server = rospy.Service(self.agent_id+'/'+'set_agent_props', SetAgentProp, self.set_props_cb)

    # returns an interface element
    def __getitem__(self,item):

        if not item in self.callables:
            raise ValueError('{agent_id} AgentInterface does not have an element {element}'.format(agent_id=self.agent_id,element=item))
        else:
            return self.callables[item]

    # service callback
    def is_idle_cb(self,req):
        return AgentIdleResponse(isIdle=self.is_agent_idle())

    def get_props_cb(self,req):
        return GetJsonResponse(jsonStr=json.dumps(self.agent_props))

    def set_props_cb(self,req):
        try:
            new_vals = json.loads(req.jsonStr)
        except Exception as error:
            print repr(error)
            print 'Set agent property failed in parsing json: {j}'.format(j=req.jsonStr)
            return SetAgentPropResponse(success=False)

        set_props = []
        for prop in new_vals:
            if prop in self.agent_props:
                self.agent_props[prop] = new_vals[prop]
                set_props += [prop]

        success = len(set_props) == len(new_vals)

        return SetAgentPropResponse(success=success, set_props=set_props)

    # return active tasks names
    def current_tasks(self):
        return get_active_states(self.fsm)[0]

    #
    def is_agent_idle(self):
        return self.isIdle

    # AgentFSM transition callback. Used to update isIdle
    def check_idle(self):
        self.isIdle = 'DEFAULT' in self.fsm.get_active_states()

    # force a value for isIdle
    def set_idle(self, isIdle):
        self.isIdle = isIdle

    # find active tasks in AgentFSM
    def get_active_states(self, task):
        actives = []
        a_ids = []
        if isinstance(task,smach.StateMachine) or isinstance(task,smach.Concurrence):
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

        if hasattr(task, 'get_default'):
            a = self.get_active_states(task.get(task.get_default()))
            actives += [task.get_default()] + a[0]
            a_ids += [id(task.get(task.get_default()))] + a[1]

        return actives, a_ids

    # add a publisher to the agent interface
    def add_publisher(self,name,topic_name, data_class, queue_size):

        if not name in self.callables:
            self.callables[name] = rospy.Publisher(topic_name, data_class, queue_size=queue_size)
        else:
            rospy.logdebug('An element with name {name} already exists in {agent_id} AgentInterface'.format(agent_id=self.agent_id,name=name))

    # add a client to the agent interface
    def add_client(self,name,topic_name, data_class):

        if not name in self.callables:
            #print topic_name
            rospy.wait_for_service(topic_name,20.0) #TODO: waiting time should be a param
            self.callables[name] = rospy.ServiceProxy(topic_name, data_class)
        else:
            rospy.logdebug('An element with name {name} already exists in this AgentInterface'.format(name=name))

    # add a subscriber to the agent interface
    def add_subscriber(self, task, topic_name, data_class, callback):

        if not topic_name in self.callbacks:
            def meta_callback(msg):
                #call agent level callbacks
                if 'AGENT' in self.callbacks[topic_name]:
                    self.callbacks[topic_name]['AGENT'](msg)

                #call task callbacks
                if self.fsm.isInit:
                    actives = self.get_active_states(self.fsm)
                    #print 'ACTIVES: {actives}'.format(actives=actives)
                    for active in actives[1]:
                        if active in self.callbacks[topic_name]:
                            self.callbacks[topic_name][active](msg)

            self.callbacks[topic_name] = {}
            self.callbacks[topic_name]['meta'] = meta_callback
            self.subscribers[topic_name] = rospy.Subscriber(topic_name, data_class, meta_callback)

        label = 'AGENT' if type(task) == 'str' and task == 'AGENT' else id(task)
        self.callbacks[topic_name][label] = callback

    # add a server to the agent interface
    def add_server(self, task, topic_name, data_class, callback, inactive_callback):

        if not topic_name in self.callbacks:
            def meta_callback(req):
                #call agent level callbacks. If defined, it will override define task callbacks
                if 'AGENT' in self.callbacks[topic_name]:
                    return self.callbacks[topic_name]['AGENT'](req)

                #call task callbacks
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

        label = 'AGENT' if type(task) == 'str' and task == 'AGENT' else id(task)
        self.callbacks[topic_name][label] = callback

#Runs in a loop until a request to execute a new task arrives
#To be executed in paralel with Default task in the DefaultTaskContainer
class ExecTaskWatch(smach.State):
    def exec_task_cb(self,task_id):#msg):
        self.task = task_id #msg.task_id
        self.request_preempt()

    def __init__(self, agent_id, task_list):
        self.task = ''
        self.task_list = task_list
        dispatcher.connect( self.exec_task_cb, signal='exec_task', sender=dispatcher.Any )

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

#called if a child state terminates in the DefaultTaskContainer
def child_term_cb(outcome_map):
    return True

#concurrent outcome map in the DefaultTaskContainer
def out_cb(outcome_map):
    if outcome_map['DEFAULT_TASK'] == 'error':
        return 'error'
    else:
        return outcome_map['EXEC_TASK_WATCH']

#default task container running in paralel the execute task watcher and the default task
class DefaultTaskContainer(smach.Concurrence):

    def set_default(self, task_name):

        self.m_default.set_default(task_name)

    '''Parameters:
        - default_task: the task the agents execute on idle
        - task_list: names of the tasks the agent can execute'''
    def __init__(self, agent_id, d_dic, default_name, task_list):
        smach.Concurrence.__init__(self,input_keys=[],outcomes=task_list+['error','invalid_task'],
                                   default_outcome='error',child_termination_cb=child_term_cb,
                                   outcome_cb=out_cb)

        class MetaDefault(smach.State):

            def __init__(self, d_dic, default_name):
                self.default = default_name
                self.d_tasks = d_dic
                smach.State.__init__(self, outcomes=['success','error'])

            def execute(self, ud):
                outcome = self.d_tasks[self.default].execute(ud)
                return 'success' if outcome != 'error' else 'error'

            def set_default(self, task_name):
                if task_name in self.d_tasks:
                    self.default = task_name
                else:
                    rospy.logwarn('Task {t} does not exist. It cannot be set as default'.format(t=task_name))

            def get_default(self):
                return self.default

            def get(self, t_name):
                return self.d_tasks[self.default]

            def request_preempt(self):
                self.d_tasks[self.default].request_preempt()

        self.m_default = MetaDefault(d_dic, default_name)

        with self:
            smach.Concurrence.add('EXEC_TASK_WATCH', ExecTaskWatch(agent_id, task_list))
            smach.Concurrence.add('DEFAULT_TASK', self.m_default)

        dispatcher.connect( self.set_default, signal='change_default_task', sender=dispatcher.Any )

#Wraps a Task so the agent can expose it for external execution
#Ensures it has the right outcomes: success, error
class AgentTaskWrapper(smach.StateMachine):

    '''Parameters:
        - task: agent task to be wrapped. Can be a state or a state machine.
        - transitions: dic containing the transitions from the wrapped task outcomes
          to 'success' and 'error' '''
    def __init__(self, name, task, transitions):
        smach.StateMachine.__init__(self,outcomes=['success','error'])
        self.name = name

        with self:
            smach.StateMachine.add(name, task, transitions)

    def execute(self, parent_ud = smach.UserData()):
        # Set current state
        self._set_current_state(self.name)

        # Execute the state
        try:
            outcome = self._current_state.execute(self.userdata)
        except smach.InvalidUserCodeError as ex:
            smach.logerr("State '%s' failed to execute." % self._current_label)
            raise ex
        except:
            raise smach.InvalidUserCodeError("Could not execute state '%s' of type '%s': " %
                                             (self._current_label, self._current_state)
                                             + traceback.format_exc())

        output_keys = {}
        for key in self._current_state.get_registered_output_keys():
            output_keys[key] = getattr(self.userdata,key)

        dispatcher.send( signal='task_completed', task_id = self.name, outcome=outcome, output_keys = output_keys, sender=self )
        outcome = self._current_transitions[outcome]

        # Set current state
        self._set_current_state(None)

        return outcome


#Agent fsm transiting from requested tasks to default task
class AgentStateMachine(smach.StateMachine):

    def __init__(self):
        self.isInit = False

    '''Parameters:
        - default_task: the task the agents execute on idle
        - tasks_dic: names and objects of the tasks the agent can execute'''
    def initialize(self, agent_id, d_dic, default_name, tasks_dic):
        #TODO: 'error' outcome here simbolizes a critical recovery task, which needs to be an actual
        #state in the statemachine, same as default
        smach.StateMachine.__init__(self,outcomes=['error'],
                                    input_keys=[])

        default = DefaultTaskContainer(agent_id, d_dic, default_name, tasks_dic.keys())
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

#Add a task to a tasks dictionary to be used to initialize an AgentFSM
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
        if interface.is_agent_idle():
            interface.set_idle(False) #because the fsm takes some time to do the transition
            dispatcher.send( signal='exec_task', task_id=name )

            task_outcome = {}
            def completed_cb(task_id,outcome,output_keys):
                rospy.loginfo('Task {id} completed with outcome {o}'.format(id=task_id,o=outcome))
                if task_id == name:
                    task_outcome['outcome'] = outcome # because cannot assign nonlocal vars in python 2.x
                    task_outcome['output_keys'] = '' # json.dumps(output_keys) TODO: dumps raise error cause some keys are not json seralizable

            dispatcher.connect( completed_cb, signal='task_completed', sender=wrapper )

            r = rospy.Rate(1)
            while not rospy.is_shutdown() and not 'outcome' in task_outcome and not 'output_keys' in task_outcome:
                r.sleep()

            return task.ResponseType(success=True,outcome=task_outcome['outcome'],output_keys=task_outcome['output_keys'])
        else:
            rospy.logwarn('Agent {agent_id} cannot execute task {name}. Currently executing: {active}'.format(
            agent_id=interface.agent_id,name=name,active=interface.fsm.get_active_states()))
            return task.ResponseType(success=False,outcome='not_executed')

    return rospy.Service(interface.agent_id+'/task/'+name, task.DataType, cb)

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