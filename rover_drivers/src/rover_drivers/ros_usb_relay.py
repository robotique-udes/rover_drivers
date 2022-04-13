from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from queue import Empty

import rospy
from rover_drivers.libusb_relay import UsbRelay, Relay
from std_srvs.srv import Empty, SetBool


class UsbRelayNode(object):
    def __init__(self):
        rospy.init_node('usb_relay')
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.baudrate = rospy.get_param('~baudrate', 9600)

        rospy.loginfo("USB Relay Node started; port: %s :: baudrate: %d" %
                      (self.serial_port, self.baudrate))
        self.relay = UsbRelay(port=self.serial_port)
        self.relay1_sub = rospy.Service(
            '/relay/r1_cmd', SetBool, self.relay_srv_cb1)
        self.relay2_sub = rospy.Service(
            '/relay/r2_cmd', SetBool, self.relay_srv_cb2)
        self.relay3_sub = rospy.Service(
            '/relay/r3_cmd', SetBool, self.relay_srv_cb3)
        self.relay4_sub = rospy.Service(
            '/relay/r4_cmd', SetBool, self.relay_srv_cb4)
        self.relay5_sub = rospy.Service(
            '/relay/r5_cmd', SetBool, self.relay_srv_cb5)
        self.relay6_sub = rospy.Service(
            '/relay/r6_cmd', SetBool, self.relay_srv_cb6)
        self.relay7_sub = rospy.Service(
            '/relay/r7_cmd', SetBool, self.relay_srv_cb7)
        self.relay8_sub = rospy.Service(
            '/relay/r8_cmd', SetBool, self.relay_srv_cb8)
        self.clear_srv = rospy.Service(
            "/relay/clear", Empty, self.clear_srv_cb)
        self.fullon_srv = rospy.Service(
            "/relay/fullon", Empty, self.fullon_srv_cb)

    def relay_srv_cb(self, msg, arg):
        if msg.data:
            self.relay.on(Relay(arg))
        else:
            self.relay.off(Relay(arg))

    def clear_srv_cb(self, req):
        self.relay.off_all()
        return ()

    def fullon_srv_cb(self, req):
        self.relay.on_all()
        return ()

    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            del self.serial_port
