#!/usr/bin/env python
import rospy
from jetto_base.battery.status import BatteryMonitor

if __name__ == '__main__':
    rospy.init_node('jetbot_battery_status')
    battery_monitor = BatteryMonitor()
    battery_monitor.run()
