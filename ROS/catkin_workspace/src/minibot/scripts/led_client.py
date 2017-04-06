#!/usr/bin/env python
# coding=utf-8

# This is a client node

import sys
import rospy
# name of the package(!).srv
from minibot.srv import *

def led_switcher_client(pin):
    # Service 'led' from led_server.py ready?
    rospy.wait_for_service('led')
    try:
        # Create the handle 'led_switcher' with the service type 'Led'.
        # The latter automatically generates the LedRequest and LedResponse objects.
        led_switcher = rospy.ServiceProxy('led', Led)
        # the handle can be called like a normal function
        response = led_switcher(pin, state)
        return response.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "\nError!\n\nUsage: %s [pin] [state]\nExample to turn a LED for GPIO 18 ON: %s 18 1\n"%(sys.argv[0], sys.argv[0])

if __name__ == "__main__":
    # enough arguments?
    if len(sys.argv) == 3:
        pin   = int(sys.argv[1])
        state = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)

    # call the led_switcher function
    # and show the result
    print "GPIO %s switched to %s. Result: %s"%(pin, state, led_switcher_client(pin))
