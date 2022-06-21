from __future__ import absolute_import, division, print_function, unicode_literals
from queue import Empty

import rospy
from rover_drivers.libusb_relay import UsbRelay, Relay
from std_srvs.srv import Empty, SetBool, SetBoolResponse
from functools import partial


class UsbRelayNode(object):
    def __init__(self):
        rospy.init_node("usb_relay")
        self.serial_port = rospy.get_param("~serial_port", "/dev/ttyACM0")
        self.baudrate = rospy.get_param("~baudrate", 9600)

        rospy.loginfo("USB Relay Node started; port: %s :: baudrate: %d" % (self.serial_port, self.baudrate))
        self.relay = UsbRelay(port=self.serial_port)
        self.relay1_sub = rospy.Service("/relay/r1_cmd", SetBool, partial(self.relay_srv_cb, arg=0))
        self.relay2_sub = rospy.Service("/relay/r2_cmd", SetBool, partial(self.relay_srv_cb, arg=1))
        self.relay3_sub = rospy.Service("/relay/r3_cmd", SetBool, partial(self.relay_srv_cb, arg=2))
        self.relay4_sub = rospy.Service("/relay/r4_cmd", SetBool, partial(self.relay_srv_cb, arg=3))
        self.relay5_sub = rospy.Service("/relay/r5_cmd", SetBool, partial(self.relay_srv_cb, arg=4))
        self.relay6_sub = rospy.Service("/relay/r6_cmd", SetBool, partial(self.relay_srv_cb, arg=5))
        self.relay7_sub = rospy.Service("/relay/r7_cmd", SetBool, partial(self.relay_srv_cb, arg=6))
        self.relay8_sub = rospy.Service("/relay/r8_cmd", SetBool, partial(self.relay_srv_cb, arg=7))
        self.clear_srv = rospy.Service("/relay/clear", Empty, self.clear_srv_cb)
        self.fullon_srv = rospy.Service("/relay/fullon", Empty, self.fullon_srv_cb)

    def relay_srv_cb(self, msg, arg):
        if msg.data:
            self.relay.on(Relay(arg))
        else:
            self.relay.off(Relay(arg))
        return SetBoolResponse(success=True)

    def clear_srv_cb(self, req):
        self.relay.off_all()
        return SetBoolResponse(success=True)

    def fullon_srv_cb(self, req):
        self.relay.on_all()
        return SetBoolResponse(success=True)

    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            del self.serial_port
