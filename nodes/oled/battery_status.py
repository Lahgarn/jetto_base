#!/usr/bin/env python
import random

import Adafruit_SSD1306
import rospy

from PIL import Image, ImageDraw, ImageFont

from jetto_base.constants import Topics
from jetto_base.msg import BatteryStatus


class BatteryStatusDisplay:
    frases_gualls = [
        "Estic en forma!",
        "Que dius nano?",
        "Els humans caureu!",
        "Estic en flex!",
        "CuCuuu~"
    ]

    def __init__(self, sub_topic=Topics.BatteryStatus.value):
        self._sub = rospy.Subscriber(sub_topic, BatteryStatus, callback=self._on_battery_status, queue_size=1)
        self._disp = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1)

        self._disp.begin()
        self._disp.clear()
        self._disp.display()

        self._width = self._disp.width
        self._height = self._disp.height

        self._image = Image.new('1', (self._width, self._height))
        self._draw = ImageDraw.Draw(self._image)

        self._font = ImageFont.load_default()

    def _clear_draw(self):
        # Draw a black filled box to clear the image.
        self._draw.rectangle((0, 0, self._width, self._height), outline=0, fill=0)

    def _display_image(self):
        self._disp.image(self._image)
        self._disp.display()

    def _on_battery_status(self, msg):
        self._clear_draw()

        percent = msg.percent * 100

        p = random.randint(0, 100)
        if p <= 5:
            message = random.choice(BatteryStatusDisplay.frases_gualls)
        else:
            message = "  BAT: {: 3.2f} %".format(percent)

        self._draw.text((0, 8)," ================", font=self._font, fill=255)
        self._draw.text((0, 16), message,  font=self._font, fill=255)
        self._draw.text((0, 25)," ================", font=self._font, fill=255)

        self._display_image()


if __name__ == '__main__':
    rospy.init_node('jetbot_oled_battery_status')
    bsd = BatteryStatusDisplay()
    rospy.spin()
