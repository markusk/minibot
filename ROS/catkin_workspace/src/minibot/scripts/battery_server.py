#!/usr/bin/env python

from beginner_tutorials.srv import *
import rospy

def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('battery_server')
    s = rospy.Service('battery', Battery, handle_add_two_ints)
    print "Ready to provide battery details."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
