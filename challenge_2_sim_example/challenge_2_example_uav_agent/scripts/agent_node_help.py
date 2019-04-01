#!/usr/bin/env python
import roslib
import rospy
import smach
import smach_ros

from mbzirc_comm_objs.msg import StartTask

#Runs in a loop until a request to execute a new task arrives
class ExecTaskWatch(smach.State):
    def exec_task_cb(self,msg):
        self.task = msg.task_id
        self.request_preempt()
    def __init__(self, task_list):
        self.task = ''
        self.task_list = task_list
        rospy.Subscriber('start_task', StartTask, self.exec_task_cb)
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
    def __init__(self, default_task, task_list):
        smach.Concurrence.__init__(self,input_keys=list(default_task._input_keys),outcomes=task_list+['error','invalid_task'],
                                   default_outcome='error',child_termination_cb=child_term_cb,
                                   outcome_cb=out_cb)
        with self:
            smach.Concurrence.add('EXEC_TASK_WATCH', ExecTaskWatch(task_list))
            smach.Concurrence.add('DEFAULT_TASK', default_task)

#Wraps a Task so the agent can expose it for external execution
#Ensures it has the right outcomes: success, error
class AgentTaskWrapper(smach.StateMachine):

    '''Parameters:
        - task: agent task to be wrapped. Can be a state or a state machine.
        - transitions: dic containing the transitions from the wrapped task outcomes
          to 'success' and 'error' '''
    def __init__(self, task, transitions):
        smach.StateMachine.__init__(self,outcomes=['success','error'])#, input_keys=list(task._input_keys | set(['interface'])))

        with self:
            smach.StateMachine.add('Wrapped_Task', task, transitions) #TODO: call it with wrapper name

#TODO: maybe implement DefaultTask class??

#Agent sm transiting from requested tasks to default task
class AgentStateMachine(smach.StateMachine):

    '''Parameters:
        - default_task: the task the agents execute on idle
        - tasks_dic: names and objects of the tasks the agent can execute'''
    def __init__(self, default_task, tasks_dic):
        #TODO: 'error' outcome here simbolizes a critical recovery task, which needs to be an actual
        #state in the statemachine, same as default
        input_keys_s = default_task._input_keys | set(['interface'])
        '''for task in tasks_dic:
            input_keys_s = input_keys_s | tasks_dic[task]._input_keys'''

        smach.StateMachine.__init__(self,outcomes=['error'],input_keys=list(input_keys_s))
        default = DefaultTaskContainer(default_task, tasks_dic.keys())
        default_trans = {'invalid_task':'DEFAULT','error':'error'}
        for task in tasks_dic.keys():
            default_trans[task]=task

        with self:
            smach.StateMachine.add('DEFAULT', default, transitions=default_trans)
            for task in tasks_dic.keys():
                smach.StateMachine.add(task, tasks_dic[task],
                        transitions={'success':'error','error':'error'})
