#!/usr/bin/env python

import sys
import rospy
from led.srv import *

def led_switcher_client(pin):
    rospy.wait_for_service('led')
    try:
        led_switcher = rospy.ServiceProxy('led', Led)
        resp1 = led_switcher(pin)
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [pin]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        pin = int(sys.argv[1])
    else:
        print usage()
        sys.exit(1)
    print "Requesting voltage %s+%s"%(x, y)
    print "LED %s switched. Result = %s"%(pin, led_switcher_client(pin))
