#!/usr/bin/env python
"""
Provide a service for powering off the Jetson and Joule.

To test:
    rosservice call /poweroff
"""
import os
import time
import dbus
import rospy
from ras_msgs.srv import Poweroff, PoweroffResponse

class PoweroffService: 
    def __init__(self):
        # Name this node
        rospy.init_node('poweroff')

        """
        if self.can_poweroff():
            # Listen to object locations that are published
            rospy.Service("/poweroff", Poweroff, self.callback)
        else:
            rospy.logerr("You do not have permission to poweroff! Thus /poweroff will not run.")
        """

        rospy.Service("/poweroff", Poweroff, self.callback)

    def can_poweroff(self):
        bus = dbus.SystemBus()
        obj = bus.get_object('org.freedesktop.login1', '/org/freedesktop/login1')
        iface = dbus.Interface(obj, 'org.freedesktop.login1.Manager')
        result = iface.get_dbus_method("CanPowerOff")
        return result() == "yes"

    def cmd_poweroff(self):
        bus = dbus.SystemBus()
        obj = bus.get_object('org.freedesktop.login1', '/org/freedesktop/login1')
        iface = dbus.Interface(obj, 'org.freedesktop.login1.Manager')
        method = iface.get_dbus_method("PowerOff")
        method(True)

    def callback(self, req):
        # too much work to have another dbus service on the Jetson right now...
        # TODO later
        #os.system('ssh jetson "sudo /bin/systemctl poweroff -i"')
        os.system('ssh jetson "sudo /sin/poweroff"')

        time.sleep(2)

        # Sometimes doesn't work, so just run command
        #self.cmd_poweroff()
        os.system('sudo /sbin/poweroff')

        return PoweroffResponse(True)

if __name__ == '__main__':
    try:
        service = PoweroffService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
