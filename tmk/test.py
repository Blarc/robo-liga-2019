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

# motor_A.run_forever(speed_sp=400)
# motor_B.run_forever(speed_sp=400)

# motor_left.stop(stop_action='brake')
# motor_right.stop(stop_action='brake')

speed = 0

while speed < 800:
    motor_A.run_forever(speed_sp=speed)
    motor_B.run_forever(speed_sp=speed)
    speed += 2.5
    sleep(0.001)

motor_A.stop(stop_action='brake')
motor_B.stop(stop_action='brake')


def accelerate_both_motors_to(curr_speed, wanted_speed):
    while curr_speed < wanted_speed:
        motor_A.run_forever(speed_sp=curr_speed)
        motor_B.run_forever(speed_sp=curr_speed)
        curr_speed += 3
        sleep(0.001)


def decelerate_to(curr_speed, wanted_speed):
    while curr_speed > wanted_speed:
        motor_A.run_forever(speed_sp=curr_speed)
        motor_B.run_forever(speed_sp=curr_speed)
        curr_speed -= 3
        sleep(0.001)