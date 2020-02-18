#!/usr/bin/env python

import copy
import time
import rospkg
import yaml
import mbzirc_comm_objs.msg as msg

class BrickTask(object):
    def __init__(self):
        self.color = msg.ObjectDetection.COLOR_UNKNOWN
        self.segment = 0
        self.layer = 0
        self.position = 0.0
        self.state = 'UNINITIALIZED'

    def __repr__(self):
        return "%s(color=%r, segment=%r, layer=%r, position=%r, state=%r)" % (
            self.__class__.__name__, self.color, self.segment, self.layer, self.position, self.state)

brick_task_list = []
brick = BrickTask()
brick.color = msg.ObjectDetection.COLOR_RED
brick.segment = 1
brick.layer = 1
brick.position = 0.1234
brick.state = 'TODO'
brick_task_list.append(copy.deepcopy(brick))

brick.color = msg.ObjectDetection.COLOR_GREEN
brick.segment = 2
brick.layer = 2
brick.position = 0.2234
brick.state = 'DOING'
brick_task_list.append(copy.deepcopy(brick))

brick.color = msg.ObjectDetection.COLOR_BLUE
brick.segment = 3
brick.layer = 1
brick.position = 0.3234
brick.state = 'DONE'
brick_task_list.append(copy.deepcopy(brick))

brick_task_config_filename = 'test_brick_task_config.yaml'
brick_task_config_url = rospkg.RosPack().get_path('robot_tasks') + '/config/' + brick_task_config_filename
with open(brick_task_config_url, 'w') as config:
    yaml.dump({'brick_task_list': brick_task_list}, config)#, default_flow_style=False)

time.sleep(1)

with open(brick_task_config_url, 'r') as config:
    recovered_task_list = yaml.load(config)['brick_task_list']

for task in recovered_task_list:
    print task
