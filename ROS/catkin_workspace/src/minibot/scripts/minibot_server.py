#!/usr/bin/env python

# from [packagename.srv import [Service(s)]
from minibot.srv import *
# Minibot,MiniBotResponse
import rospy

def handle_minibot(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return MinibotResponse(req.a + req.b)

def minibot_server():
    rospy.init_node('minibot_server')
    s = rospy.Service('minibot', Minibot, handle_minibot)
    print "Ready to control and observe the minibot."
    rospy.spin()

if __name__ == "__main__":
    minibot_server()
