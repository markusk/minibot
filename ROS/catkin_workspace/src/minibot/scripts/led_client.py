#!/usr/bin/env python

import sys
import rospy
# name of the package(!).srv
from minibot.srv import *

def led_switcher_client(pin):
    rospy.wait_for_service('led')
    try:
        led_switcher = rospy.ServiceProxy('led', Led)
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

    # debug
    print "GPIO %s switched to %s. Result: %s"%(pin, state, led_switcher_client(pin))
