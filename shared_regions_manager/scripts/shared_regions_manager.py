#!/usr/bin/env python
import rospy
import Queue
from mbzirc_comm_objs.msg import RegionOwnerList
from mbzirc_comm_objs.srv import RequestSharedRegion, RequestSharedRegionResponse, RequestSharedRegionRequest, AddSharedRegion, AddSharedRegionResponse, RemoveSharedRegion, RemoveSharedRegionResponse
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

class SharedRegionAdmin():

    def add_region_cb(self, req):
        #region = (0: frame,1: polygon,2: [Point],3: [wp occupants], 4: queue, 5: region_owner). #TODO: maybe waiting points could be computed here
        print "add region requested"
        self.regions += [[req.frame_id, req.region, req.waiting_points,
                        ["" for i in range(len(req.waiting_points))],
                        Queue.Queue(maxsize=len(req.waiting_points)), ""]]

        return AddSharedRegionResponse(region_id=len(self.regions)-1)

    def remove_region_cb(self, req):
        success = False
        if req.region_id in range(len(self.regions)):
            self.regions[req.region_id] = None
            success = True

        return RemoveSharedRegionResponse(success=success)

    def handle_request_cb(self, req):
        print "reserve region requested"
        res = RequestSharedRegionResponse()
        if req.region_id in range(len(self.regions)):
            r = self.regions[req.region_id]
            if req.question == RequestSharedRegionRequest.FREE_SHARED_REGION:
                res.answer = RequestSharedRegionResponse.OK
                #release region and move waiting queue
                next_agent = r[4].get() if not r[4].empty() else ""
                for i in range(len(r[3])):
                    if r[3][i] == next_agent:
                        r[3][i] = ""
                r[5] = next_agent
            elif req.question == RequestSharedRegionRequest.RESERVE_SHARED_REGION:
                #region empty
                if not r[5]:
                    res.answer = RequestSharedRegionResponse.OK
                    r[5] = req.agent_id
                #queue not full
                elif not r[4].full():
                    res.answer = RequestSharedRegionResponse.WAIT
                    #assigns waiting point and put in queue
                    r[4].put(req.agent_id)
                    for i in range(len(r[3])):
                        if not r[3][i]:
                            r[3][i] = req.agent_id
                            h = Header(frame_id=r[0],stamp=rospy.Time.now())
                            p = r[2][i]
                            res.waiting_point = PointStamped(header=h,point=p)
                #queue is full
                else:
                    res.answer = RequestSharedRegionResponse.ERROR
        else:
            res.answer = RequestSharedRegionResponse.ERROR

        #publish owners list
        owners = []
        for r in self.regions:
            owners += [r[5]]

        self.pub.publish(RegionOwnerList(region_owners=owners))

        return res

    def __init__(self):

        self.regions = []
        self.pub = rospy.Publisher('shared_region_owners', RegionOwnerList, queue_size=1)
        rospy.Service('add_shared_region', AddSharedRegion, self.add_region_cb)
        rospy.Service('remove_shared_region', RemoveSharedRegion, self.remove_region_cb)
        rospy.Service('request_shared_region', RequestSharedRegion, self.handle_request_cb)

        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('shared_regions_manager', anonymous=True)
        SharedRegionAdmin()
    except rospy.ROSInterruptException:
        pass
