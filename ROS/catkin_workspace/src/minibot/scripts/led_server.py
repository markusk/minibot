#!/usr/bin/env python

from minibot.srv import *
import rospy


# handle_add_two_ints is called with instances of BatteryRequest and returns instances of BatteryResponse
# The request name comes directly from the .srv filename
def handle_add_two_ints(req):
    # here is all the work done :)
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))

    # The name of the response comes directly from the .srv filename!
    return BatteryResponse(req.a + req.b)


def add_two_ints_server():
    rospy.init_node('battery_server')

    # This declares a new service named battery with the Battery service type.
    s = rospy.Service('battery', Battery, handle_add_two_ints)

    print "Ready to provide battery details."
    rospy.spin()


if __name__ == "__main__":
    add_two_ints_server()
