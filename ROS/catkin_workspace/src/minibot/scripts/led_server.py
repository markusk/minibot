#!/usr/bin/env python

from led.srv import *
import rospy


# handle_add_two_ints is called with instances of BatteryRequest and returns instances of BatteryResponse
# The request name comes directly from the .srv filename
def handle_led(req):
    # here is all the work done :)
    print "LED %s switched. Result: %s]"%(req.pin, req.pin)

    # The name of the response comes directly from the .srv filename!
    return LedResponse(req.pin)


def led_server():
    rospy.init_node('led_server')

    # This declares a new service named battery with the Battery service type.
    s = rospy.Service('led', Led, handle_led)

    print "Ready to switch LEDs."
    rospy.spin()


if __name__ == "__main__":
    led_server()
