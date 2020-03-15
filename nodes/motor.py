#!/usr/bin/env python
from jetto_base.motors import RawMotorSubscriber


if __name__ == '__main__':
    rmsub = RawMotorSubscriber("motor")
    rmsub.run()
