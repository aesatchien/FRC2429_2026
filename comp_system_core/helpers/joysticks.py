"""Controller objects and the Trigger for every button on them.

2027a7 CONTROLLER MODEL - READ BEFORE CHANGING ANY OF THIS
----------------------------------------------------------
Everything here now uses the a7 UNIFIED controller classes (CommandXboxController,
CommandDualSenseController), which sit on wpilib's new HIDDevice.  They replaced the
CommandNiDs* classes we used on a6, and the swap fixed two real bugs at once:

1. THE D-PAD IS NOT A POV HAT on SystemCore.  It arrives as four ordinary buttons.  On a6
   the NiDs classes exposed it as a POV, and commands2's povUp()/povDown()/... were also
   broken independently (they compared a POVDirection enum against an int degree constant,
   which is False for every value, silently).  a7 spells it dpad_up() and there is no POV
   anywhere in the gamepad family.

2. THE NiDs BUTTON AND AXIS INDICES WERE WRONG FOR SYSTEMCORE.  This is the one that cost
   us the auto-tracking bug.  The legacy NI-Driver-Station layout and the SystemCore layout
   agree on the four face buttons and then diverge completely:

       pressed        SystemCore idx   NiDsXboxController read it as
       A / South            0          A          <- agrees
       B / East             1          B          <- agrees
       X / West             2          X          <- agrees
       Y / North            3          Y          <- agrees
       Back                 4          Left Bumper
       Guide                5          Right Bumper
       Start                6          Back
       Left stick           7          Start
       Right stick          8          Left stick
       Left bumper          9          -
       Right bumper        10          NOTHING - off the end of the NiDs map

   So right_bumper - the start-tracking + spin-up button - could never fire, and the robot
   would not turn to the hub when you shot.  Face buttons worked, which is exactly why it
   looked like a targeting bug instead of a controller bug.

   The axes were shifted too (NiDs had LeftTrigger=2, RightTrigger=3, RightX=4; SystemCore
   has RightX=2, RightY=3, LeftTrigger=4, RightTrigger=5), which is what the hardcoded
   getRawAxis(2) / getRawAxis(4) workarounds in drive_by_joystick_subsystem_targeting.py
   were patching around.  Those are gone now - the unified classes name the axes correctly.

Xbox note: a7 uses the modern names, so the old `back` button is view() and `start` is
menu().  The variable names below keep back/start because that is what the bindings say.
"""

import commands2.button
import constants
from commands2.button import CommandDualSenseController, CommandXboxController

driver_controller = CommandXboxController(constants.k_driver_controller_port)
copilot_controller = CommandXboxController(constants.k_co_driver_controller_port)

# NOTE: the Xbox X and Y BUTTONS are methods x() / y() here.  Do not let a bulk rename
# strip these parens - a7 turned the geometry accessors Pose2d.X()/Y() into properties,
# and a careless sweep over '.x()' hits these too and silently binds a bound method
# instead of a Trigger ('function' object has no attribute 'on_true').
axis_trigger_threshold = 0.5

# Driver Buttons
driver_a = driver_controller.a()
driver_b = driver_controller.b()
driver_x = driver_controller.x()
driver_y = driver_controller.y()
driver_lb = driver_controller.left_bumper()
driver_rb = driver_controller.right_bumper()
driver_back = driver_controller.view()    # a7 renamed Back -> View
driver_start = driver_controller.menu()   # a7 renamed Start -> Menu
driver_up = driver_controller.dpad_up()
driver_down = driver_controller.dpad_down()
driver_left = driver_controller.dpad_left()
driver_right = driver_controller.dpad_right()
driver_l_trigger = driver_controller.left_trigger(axis_trigger_threshold)
driver_r_trigger = driver_controller.right_trigger(axis_trigger_threshold)

# Co-Driver Buttons
copilot_a = copilot_controller.a()
copilot_b = copilot_controller.b()
copilot_x = copilot_controller.x()
copilot_y = copilot_controller.y()
copilot_lb = copilot_controller.left_bumper()
copilot_rb = copilot_controller.right_bumper()
copilot_back = copilot_controller.view()
copilot_start = copilot_controller.menu()
copilot_up = copilot_controller.dpad_up()
copilot_down = copilot_controller.dpad_down()
copilot_left = copilot_controller.dpad_left()
copilot_right = copilot_controller.dpad_right()
copilot_l_trigger = copilot_controller.left_trigger(axis_trigger_threshold)
copilot_r_trigger = copilot_controller.right_trigger(axis_trigger_threshold)

"""
Remember - buttons are 1-indexed, not zero
"""
# Button box contains two controllers, we make them joysticks 2 and 3
bbox_1 = commands2.button.CommandJoystick(constants.k_bbox_1_port)  # port 2
bbox_2 = commands2.button.CommandJoystick(constants.k_bbox_2_port)  # port 3

bbox_1_1 = bbox_1.button(1)  # right joystick, true when selected
bbox_1_2 = bbox_1.button(2)  # left joystick,  true when selected
bbox_1_3 = bbox_1.button(3)  # top left red 1
bbox_1_4 = bbox_1.button(4)  # top left red 2
bbox_1_5 = bbox_1.button(5)
bbox_1_6 = bbox_1.button(6)
bbox_1_7 = bbox_1.button(7)
bbox_1_8 = bbox_1.button(8)
bbox_1_9 = bbox_1.button(9)
bbox_1_10 = bbox_1.button(10)
bbox_1_11 = bbox_1.button(11)
bbox_1_12 = bbox_1.button(12)

# bbox_2_1 = bbox_2.button(1)  # L1
# bbox_2_2 = bbox_2.button(2)  # L2
# bbox_2_3 = bbox_2.button(3)  # L3
# bbox_2_4 = bbox_2.button(4)  # L4
# bbox_2_5 = bbox_2.button(5)  # climb down
# bbox_2_6 = bbox_2.button(6)  # climb up ?
# bbox_2_7 = bbox_2.button(7)  #
# bbox_2_8 = bbox_2.button(8)  #

# ---------------------------------------------------------------------------
# PlayStation controller.  This IS a DualSense (PS5), and a7 finally has a class for it -
# on a6 we had to drive it through CommandNiDsPS4Controller, which is a different pad with
# a different button map.
# ---------------------------------------------------------------------------
play_station_controller = CommandDualSenseController(constants.k_ps5_controller_port)

ps_square = play_station_controller.square()
ps_cross = play_station_controller.cross()
ps_circle = play_station_controller.circle()
ps_triangle = play_station_controller.triangle()
ps_l1 = play_station_controller.l1()
ps_r1 = play_station_controller.r1()
ps_l2 = play_station_controller.l2()
ps_r2 = play_station_controller.r2()
ps_share = play_station_controller.create()   # PS5 calls PS4's Share button "Create"
ps_options = play_station_controller.options()
ps_l_stick = play_station_controller.l3()
ps_r_stick = play_station_controller.r3()
ps_ps_logo = play_station_controller.ps()
ps_touchpad = play_station_controller.touchpad()

# The mic button is first-class on the DualSense class now, so the raw button(15) guess
# this used to need is gone.
ps_mic = play_station_controller.microphone()

ps_up = play_station_controller.dpad_up()
ps_down = play_station_controller.dpad_down()
ps_left = play_station_controller.dpad_left()
ps_right = play_station_controller.dpad_right()
