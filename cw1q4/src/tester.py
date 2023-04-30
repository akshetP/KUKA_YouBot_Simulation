#!/usr/bin/env python3

import rospy
import time

from cw1q4_srv.srv import quat2rodriguesRequest
from cw1q4_srv.srv import quat2rodrigues
from cw1q4_srv.srv import quat2zyxRequest
from cw1q4_srv.srv import quat2zyx


def test_client():
    rospy.wait_for_service('quat2rodrigues')

    while not rospy.is_shutdown():
        client=rospy.ServiceProxy('quat2rodrigues',quat2rodrigues)
        req=quat2rodriguesRequest()

        req.q.x=0
        req.q.y=0.7071 
        req.q.z=0.8
        req.q.w=-0.7071     

        res=client(req)
        print(res)
        time.sleep(3)

if __name__ == '__main__':
    try:
        test_client()
    except rospy.ROSInterruptException:
        pass