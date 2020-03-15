from Adafruit_MotorHAT import Adafruit_MotorHAT


class Motor():

    def __init__(self, driver, channel):
        self._driver = driver
        self._motor = self._driver.getMotor(channel)
        if(channel == 1):
            self._ina = 1
            self._inb = 0
        else:
            self._ina = 2
            self._inb = 3

    def write(self, value):
        """Sets motor value between [-1, 1]"""
        mapped_value = int(255.0 * value)
        speed = min(max(abs(mapped_value), 0), 255)
        self._motor.setSpeed(speed)
        if mapped_value < 0:
            self._motor.run(Adafruit_MotorHAT.FORWARD)
            self._driver._pwm.setPWM(self._ina, 0, 0)
            self._driver._pwm.setPWM(self._inb, 0, speed*16)
        else:
            self._motor.run(Adafruit_MotorHAT.BACKWARD)
            self._driver._pwm.setPWM(self._ina, 0, speed*16)
            self._driver._pwm.setPWM(self._inb, 0, 0)

    def release(self):
        """Stops motor by releasing control"""
        self._motor.run(Adafruit_MotorHAT.RELEASE)
        self._driver._pwm.setPWM(self._ina, 0, 0)
        self._driver._pwm.setPWM(self._inb, 0, 0)


class MotorDriver():

    I2C_BUS = 1

    LEFT_CHANNEL = 1
    RIGHT_CHANNEL = 2

    def __init__(self, i2c_bus=I2C_BUS):
        self._driver = Adafruit_MotorHAT(i2c_bus=i2c_bus)
        self._lmotor = Motor(self._driver, self.LEFT_CHANNEL)
        self._rmotor = Motor(self._driver, self.RIGHT_CHANNEL)

    def write(self, lvalue, rvalue):
        self._lmotor.write(lvalue)
        self._rmotor.write(rvalue)

    def stop(self):
        self._lmotor.release()
        self._rmotor.release()
