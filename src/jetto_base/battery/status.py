import rospy
from jetto_base.constants import Topics
from jetto_base.msg import BatteryStatus

from . import ina219

FULL_BATTERY_VOLTAGE = 12.6
EMPTY_BATTERY_VOLTAGE = 10.0


class BatteryMonitor():

    DEFAULT_PERIOD = 2

    def __init__(self, pub_topic=Topics.BatteryStatus.value, period=DEFAULT_PERIOD):
        self._pub = rospy.Publisher(pub_topic, BatteryStatus, queue_size=10)
        self._ina219 = ina219.INA219(addr=0x41)
        self._rate = rospy.Rate(1.0 / period)

    def run(self):
        while not rospy.is_shutdown():
            # Read the values
            bus_voltage = self._ina219.getBusVoltage_V()
            shunt_voltage = self._ina219.getShuntVoltage_mV() / 1000
            current = self._ina219.getCurrent_mA() / 1000
            percent = self.voltage_to_percent(bus_voltage, shunt_voltage)

            # Build the message
            status_msg = BatteryStatus()
            status_msg.bus_voltage = bus_voltage
            status_msg.shunt_voltage = shunt_voltage
            status_msg.current = current
            status_msg.percent = percent

            # Send the message
            self._pub.publish(status_msg)

            self._rate.sleep()

    @staticmethod
    def voltage_to_percent(bus_voltage, shunt_voltage):
        voltage = bus_voltage + shunt_voltage
        percent = (voltage - EMPTY_BATTERY_VOLTAGE) / (FULL_BATTERY_VOLTAGE - EMPTY_BATTERY_VOLTAGE)
        return percent
