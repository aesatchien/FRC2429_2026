"""Controller objects and the Trigger for every button on them.

ONE DRIVER PAD, EITHER KIND
---------------------------
The driver uses a single pad on k_driver_controller_port, and constants.k_controller_type
says which kind it is - "PS5" or "XBOX".  The PS5 buttons are bound to the SAME names the
Xbox buttons use (cross -> driver_a, circle -> driver_b, ...), matched by physical position,
so robotcontainer.py has exactly one set of driver bindings and they work with either pad.
Switching pads is one constant.  (Trentan's design, Sept 2026 - it replaced a second PS5
pad on its own port with a near-duplicate copy of every driver binding.)

    driver_*       Xbox          PS5            where it is
    driver_a       A             Cross          bottom face button
    driver_b       B             Circle         right face button
    driver_x       X             Square         left face button
    driver_y       Y             Triangle       top face button
    driver_lb/rb   LB / RB       L1 / R1        shoulder bumpers
    driver_l/r_trigger  LT / RT  L2 / R2        analog triggers
    driver_back    View          Create         small button LEFT of centre
    driver_start   Menu          Options        small button RIGHT of centre
    driver_up/...  D-pad         D-pad

back/start keep the old Xbox 360 meaning by POSITION: Back was the left small button and
Start the right one.  a7 uses the modern names, so Back is view() and Start is menu() -
getting that backwards swaps whatever is bound to them.

2027a7 CONTROLLER MODEL - READ BEFORE CHANGING ANY OF THIS
----------------------------------------------------------
Everything here uses the a7 UNIFIED controller classes (CommandXboxController,
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

   The axes were shifted too, which is what the old hardcoded getRawAxis(2) / getRawAxis(4)
   workarounds in drive_by_joystick_subsystem_targeting.py were patching around.  Those are
   gone - the unified classes name the axes correctly.

   So: NEVER use a CommandNiDs* class for a pad plugged into a SystemCore.  That includes
   the copilot - it was left on CommandNiDsXboxController in one version of this file, which
   would have silently remapped every copilot button above index 3.
"""

import commands2.button
import constants
from commands2.button import CommandDualSenseController, CommandXboxController

# NOTE: the Xbox X and Y BUTTONS are methods x() / y() here.  Do not let a bulk rename
# strip these parens - a7 turned the geometry accessors Pose2d.X()/Y() into properties,
# and a careless sweep over '.x()' hits these too and silently binds a bound method
# instead of a Trigger ('function' object has no attribute 'on_true').
axis_trigger_threshold = 0.5

k_controller_types = ("PS5", "XBOX")
if constants.k_controller_type not in k_controller_types:
    # Fail loudly.  The old version fell through to Xbox on anything that was not exactly
    # "PS5", so a typo like "ps5" quietly gave you the wrong button map.
    raise ValueError(f"constants.k_controller_type must be one of {k_controller_types}, "
                     f"got {constants.k_controller_type!r}")

def _raw_axes(pad):
    """Turn OFF the deadband WPILib now builds into every gamepad axis.

    2027 added a default deadband to all gamepads: sticks read 0 below 0.1 and are rescaled
    above it (0.2 raw -> 0.11).  Our drive commands already apply their own deadband
    (dc.k_inner_deadband) and sqrt curve, tuned on 2026 raw values - stacked, the stick was
    dead to ~0.19 and low-speed driving felt mushy.  Zeroing it here gives the commands
    exactly what 2026 gave them.
    """
    hid = pad.get_controller()
    for name in dir(hid):
        if name.startswith('set_') and name.endswith('_deadband'):
            getattr(hid, name)(0.0)


# ---------------------------------------------------------------------------
# Driver - one pad, either kind.  Same variable names either way.
# ---------------------------------------------------------------------------
if constants.k_controller_type == "PS5":
    driver_controller = CommandDualSenseController(constants.k_driver_controller_port)
    driver_a = driver_controller.cross()
    driver_b = driver_controller.circle()
    driver_x = driver_controller.square()
    driver_y = driver_controller.triangle()
    driver_lb = driver_controller.l1()
    driver_rb = driver_controller.r1()
    driver_l_stick = driver_controller.l3()
    driver_r_stick = driver_controller.r3()
    driver_l_trigger = driver_controller.l2()   # DualSense exposes the triggers as buttons
    driver_r_trigger = driver_controller.r2()
    driver_back = driver_controller.create()    # left small button
    driver_start = driver_controller.options()  # right small button
    # PS5-only buttons.  Nothing binds these yet; they exist so nobody has to guess at
    # raw button numbers again - the mic used to be a bare button(15) guess.
    driver_ps_logo = driver_controller.ps()
    driver_touchpad = driver_controller.touchpad()
    driver_mic = driver_controller.microphone()
else:
    driver_controller = CommandXboxController(constants.k_driver_controller_port)
    driver_a = driver_controller.a()
    driver_b = driver_controller.b()
    driver_x = driver_controller.x()
    driver_y = driver_controller.y()
    driver_lb = driver_controller.left_bumper()
    driver_rb = driver_controller.right_bumper()
    driver_l_stick = driver_controller.left_stick()
    driver_r_stick = driver_controller.right_stick()
    driver_l_trigger = driver_controller.left_trigger(axis_trigger_threshold)
    driver_r_trigger = driver_controller.right_trigger(axis_trigger_threshold)
    driver_back = driver_controller.view()      # left small button  (a7 renamed Back -> View)
    driver_start = driver_controller.menu()     # right small button (a7 renamed Start -> Menu)
    driver_ps_logo = driver_touchpad = driver_mic = None   # no such buttons on an Xbox pad

_raw_axes(driver_controller)

# the D-pad is spelled the same on both
driver_up = driver_controller.dpad_up()
driver_down = driver_controller.dpad_down()
driver_left = driver_controller.dpad_left()
driver_right = driver_controller.dpad_right()

# ---------------------------------------------------------------------------
# Co-driver - always an Xbox pad.  Unified class, NOT CommandNiDsXboxController - see the
# index table in the module docstring for what the NiDs class would do to these.
# ---------------------------------------------------------------------------
copilot_controller = CommandXboxController(constants.k_co_driver_controller_port)
_raw_axes(copilot_controller)
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

bbox_1_1 = bbox_1.button(0)  # right joystick, true when selected
bbox_1_2 = bbox_1.button(1)  # left joystick,  true when selected
bbox_1_3 = bbox_1.button(2)  # top left red 1
bbox_1_4 = bbox_1.button(3)  # top left red 2
bbox_1_5 = bbox_1.button(4)
bbox_1_6 = bbox_1.button(5)
bbox_1_7 = bbox_1.button(6)
bbox_1_8 = bbox_1.button(7)
bbox_1_9 = bbox_1.button(8)
bbox_1_10 = bbox_1.button(9)
bbox_1_11 = bbox_1.button(10)
bbox_1_12 = bbox_1.button(11)

# bbox_2_1 = bbox_2.button(1)  # L1
# bbox_2_2 = bbox_2.button(2)  # L2
# bbox_2_3 = bbox_2.button(3)  # L3
# bbox_2_4 = bbox_2.button(4)  # L4
# bbox_2_5 = bbox_2.button(5)  # climb down
# bbox_2_6 = bbox_2.button(6)  # climb up ?
# bbox_2_7 = bbox_2.button(7)  #
# bbox_2_8 = bbox_2.button(8)  #
