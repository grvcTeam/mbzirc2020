#! /usr/bin/env python
import rospy
import random

# PARAMETER	SPECIFICATION
# Number of UAVs per team	Maximum of 3
# Number of UGVs per team	1
# Arena size	50mx60mx20m
# Brick shapes and material	Rectangular cube, Styrofoam material
# Bricks size (Red, Green, Blue)	Approximately 0.30mx0.20mx0.20m, 0.60mx0.20mx0.20m and1.20x0.20x0.20m
# Bricks size (Orange)	1.80x0.20x0.20 m
# Weight of bricks	O <= 2.0kg , B <= 1.5kg , G <= 1kg , R <= 1kg,
# Brick gripping mechanism	Primarily magnetic, but other gripping mechanisms could be used
# Environment	Outdoor
# Mode of operation	Autonomous; manual allowed but penalized
# RTK/DGPS	Allowed but penalized
# Challenge duration	30 minutes
# Communications	TBD

class UavDataFeed(object):
    def __init__(self, is_idle):
        self.is_idle = is_idle

uav_data_feeds = {}
# def all_idle(available_uavs):
#     for uav_id in available_uavs:
#         if not uav_data_feeds[uav_id].is_idle:
#             return False
#     return True

def main():
    available_uavs = ['1', '2']
    uav_data_feeds['1'] = UavDataFeed(False)
    uav_data_feeds['2'] = UavDataFeed(False)

    finished_uavs = ['3']
    while True:
        if random.random() > 0.5:
            uav_data_feeds['1'].is_idle = True
            print('1 is idle!')

        if random.random() > 0.8:
            uav_data_feeds['2'].is_idle = True
            print('2 is idle!')
    
        for uav_id in available_uavs:
            if uav_data_feeds[uav_id].is_idle and (uav_id not in finished_uavs):
                finished_uavs.append(uav_id)
                print('waiting result of pick_and_place server [{}]'.format(uav_id))
                # self.uav_clients[uav_id]['pick_and_place'].wait_for_result()
                # print(self.uav_clients[uav_id]['pick_and_place'].get_result())
                print('send [{}] more to do!'.format(uav_id))
        rospy.sleep(1.0)
        if set(available_uavs).issubset(finished_uavs):
            print('All done!')
            break

if __name__ == '__main__':
    main()
