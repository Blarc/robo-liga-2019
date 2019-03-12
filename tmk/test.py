#!/usr/bin/env python3

"""
Program za vodenje robota EV3
[Robo liga FRI 2019: Sadovnjak]
@Copyright: TrijeMaliKlinci
"""

from ev3dev.ev3 import LargeMotor, MediumMotor
from time import sleep

# motor_A = LargeMotor('outA')
# motor_B = LargeMotor('outD')
motor_C = MediumMotor('outC')
# motor_C.run_forever(speed_sp=-1000)
# sleep(0.25)

print(motor_C.position)
motor_C.run_forever(speed_sp=1000)
sleep(0.20)
motor_C.stop(stop_action='hold')
print(motor_C.position)


# motor_A.run_forever(speed_sp=400)
# motor_B.run_forever(speed_sp=400)

# motor_left.stop(stop_action='brake')
# motor_right.stop(stop_action='brake')

# time = 0.5
# speed = 900
# motor_A.run_forever(speed_sp=-speed)
# motor_B.run_forever(speed_sp=-speed)
# sleep(time)
# motor_A.run_forever(speed_sp=0)
# motor_B.run_forever(speed_sp=0)
# sleep(time)
# motor_A.run_forever(speed_sp=speed)
# motor_B.run_forever(speed_sp=speed)
# sleep(time)

# motor_A.stop(stop_action='brake')
# motor_B.stop(stop_action='brake')

