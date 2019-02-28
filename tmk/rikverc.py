#!/usr/bin/env python3

"""
Program za vodenje robota EV3
[Robo liga FRI 2019: Sadovnjak]
@Copyright: TrijeMaliKlinci
"""

from ev3dev.ev3 import LargeMotor
from time import sleep

motor_A = LargeMotor('outA')
motor_B = LargeMotor('outD')


while True:
    sleep(1)

    motor_A.run_forever(speed_sp=400)
    motor_B.run_forever(speed_sp=400)

    sleep(3)

    motor_A.run_forever(speed_sp=0)
    motor_B.run_forever(speed_sp=0)

    sleep(1)

    motor_A.run_forever(speed_sp=-400)
    motor_B.run_forever(speed_sp=-400)

    sleep(3)

    motor_A.run_forever(speed_sp=0)
    motor_B.run_forever(speed_sp=0)
