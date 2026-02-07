
# Subsystem: deploy / retract, go_up, go_down, needs states fore deploted and where it is at
# Command: GoToL1, GoToL2, GoToL3, UnClimb)

#Things to import:
from time import sleep

#Subsystem methods:
# - deploy: open the intake
# - retract: close the intake
# - go_up: return a certain incremental amount based on the position of the robot --> use that to give to the commands
# - go_down: same as go_up but reverse
#
# #Commands:
# - in initialize: set the variable for current and desired position
# - in execute: make the climber move to the destination
# - in end: print the robot position to command line
# - low_bar: 27 in.
# - mid_bar: 45 in.
# - high_bar: 63 in.
# 18 inches between each bar

import rev
from commands2.subsystem import Subsystem
import math
import wpilib
from rev import ClosedLoopSlot, SparkMax
from constants import TurretConstants
from commands2 import SubsystemBase
import constants

class Climber(SubsystemBase):

    def deploy(self):

    def retract(self):
    
    def go_up(self):

    def go_down(self):