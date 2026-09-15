
import math
import commands2
import wpilib
import ntcore

import constants
from subsystems.swerve import Swerve  # allows us to access the definitions
from subsystems.targeting import Targeting
from commands2.button import CommandNiDsPS4Controller, CommandNiDsXboxController, CommandJoystick
from wpimath import Translation2d
from wpimath import Debouncer, SlewRateLimiter
from subsystems.swerve_constants import DriveConstants as dc, RateLimiters as rl
from helpers.log_command import log_command


@log_command(console=True, nt=False, print_init=True, print_end=False)
class DriveByJoystickSubsystemTargeting(commands2.Command):
    def __init__(self, container, swerve: Swerve, targeting: Targeting, controller: CommandNiDsXboxController=None, ps5controller: CommandNiDsPS4Controller=None, button_box: CommandJoystick=None) -> None:
        super().__init__()
        self.set_name('drive_by_joystick_subsystem_targeting')
        
        # -----------------------------------------------------------
        # 1. Subsystems & Dependencies
        # -----------------------------------------------------------
        self.container = container
        self.swerve = swerve
        self.targeting = targeting
        self.add_requirements(self.swerve)

        self.xbox_controller: CommandNiDsXboxController = controller
        self.ps5_controller: CommandNiDsPS4Controller = ps5controller
        self.button_box: CommandJoystick = button_box

        # -----------------------------------------------------------
        # 2. Configuration & State
        # -----------------------------------------------------------
        self.field_oriented = True
        self.last_tracking_on = False

        # -----------------------------------------------------------
        # 3. Input Processing (Debouncers, Limiters)
        # -----------------------------------------------------------
        self.robot_oriented_debouncer = Debouncer(0.1, Debouncer.DebounceType.BOTH)
        
        # Use constants for slew rates to ensure tuning consistency
        self.drive_limiter = SlewRateLimiter(rl.driver_translation_slew_rate)
        self.strafe_limiter = SlewRateLimiter(rl.driver_translation_slew_rate)
        self.turbo_limiter = SlewRateLimiter(rl.turbo_input_slew_rate)
        self.afterburner_limiter = SlewRateLimiter(rl.afterburner_input_slew_rate)

        # Rotation limiters
        self.manual_rot_limiter = SlewRateLimiter(rl.driver_rotation_slew_rate)

        # -----------------------------------------------------------
        # 4. NetworkTables
        # -----------------------------------------------------------
        self._init_networktables()

    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.get_default()
        status_prefix = constants.status_prefix
        # Simulation Debugging Publishers
        self.js_dv1_x_pub = self.inst.get_double_topic(f"{status_prefix}/_joystick_dv1_x").publish()
        self.js_dv1_y_pub = self.inst.get_double_topic(f"{status_prefix}/_joystick_dv1_y").publish()
        self.js_dv_norm_x_pub = self.inst.get_double_topic(f"{status_prefix}/_joystick_dv_norm_x").publish()
        self.js_dv_norm_y_pub = self.inst.get_double_topic(f"{status_prefix}/_joystick_dv_norm_y").publish()
        self.commanded_values_pub = self.inst.get_double_array_topic(f"{status_prefix}/_joystick_commanded_values").publish()

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        pass
    
    def read_xbox(self, hid):
        """Returns (left_y, left_x, right_x, right trigger, robot_oriented) from the Xbox HID"""
        return (
            hid.get_left_y(),
            hid.get_left_x(),
            hid.get_right_x(),
            hid.get_right_trigger(),   # a7 dropped the _axis suffix
            hid.get_left_bumper_button()
        )

    def read_ps5(self, hid):
        """Returns (left_y, left_x, right_x, right trigger, robot_oriented) from the PS5 HID"""
        # The hardcoded get_raw_axis(2) / get_raw_axis(4)+1 that used to be here were
        # patching around the NiDs class having the WRONG axis map for SystemCore.  a7's
        # DualSenseController names the axes correctly, so the workaround is gone - and with
        # it the +1 fudge, because get_r2() is already 0..1 rather than -1..1.
        return (
            hid.get_left_y(),
            hid.get_left_x(),
            hid.get_right_x(),
            hid.get_r2(),
            hid.get_l1_button()
        )

    def execute(self) -> None:
        # -----------------------------------------------------------
        # 1. READ INPUTS
        # -----------------------------------------------------------
        xbox_connected = self.xbox_controller is not None and wpilib.DriverStationBackend.is_joystick_connected(0)
        ps5_connected = self.ps5_controller is not None and wpilib.DriverStationBackend.is_joystick_connected(5)
        
        # a7: get_hid() hands back the CommandGenericHID wrapper (buttons/axes by index only);
        # get_controller() is the wpilib XboxController / DualSenseController with the named
        # getters read_xbox()/read_ps5() use.  With get_hid() this was an AttributeError on the
        # first teleop loop with a pad connected - the simulation test is what caught it.
        if xbox_connected:
            left_y, left_x, right_x, right_trigger, robot_oriented = self.read_xbox(self.xbox_controller.get_controller())
        elif ps5_connected:
            left_y, left_x, right_x, right_trigger, robot_oriented = self.read_ps5(self.ps5_controller.get_controller())
        else:
            left_y, left_x, right_x, right_trigger, robot_oriented = 0.0, 0.0, 0.0, 0.0, False  # cooked??? IDK - Trentan

        if self.button_box is not None:
            # CommandJoystick.get_hid() returns a CommandGenericHID in a7; get_joystick()
            # is the one that hands back the real wpilib.Joystick.
            after_burner = self.button_box.get_joystick().get_raw_button(3)
        else:
            after_burner = False

        inputs = {
            'robot_pose': self.swerve.get_pose(),
            'left_y': left_y,
            'left_x': left_x,
            'right_x': right_x,
            'right_trigger': right_trigger,
            'after_burner' : after_burner,
            'robot_oriented': robot_oriented,
            'tracking_on': self.container.targeting.get_tracking_state(),
            'alliance': wpilib.MatchState.get_alliance()
        }

        # -----------------------------------------------------------
        # 2. CALCULATE
        # -----------------------------------------------------------
        
        # --- Drive Mode & Multipliers ---
        turbo = min(1.0, self.turbo_limiter.calculate(inputs['right_trigger']**2))
        afterburner = self.afterburner_limiter.calculate(inputs['after_burner'])

        if (inputs['after_burner'] == False):
            slowmode_multiplier = dc.kSlowModeCap + ((1 - dc.kSlowModeCap) * turbo)
            angular_slowmode_multiplier = dc.kAngularSlowFloor + ((1 - dc.kAngularSlowFloor) * turbo)
        else:
            slowmode_multiplier = dc.kSlowModeCap + ((1 - dc.kSlowModeCap) * afterburner)
            angular_slowmode_multiplier = dc.kAngularSlowFloor + ((1 - dc.kAngularSlowFloor) * afterburner)

        # --- Field Oriented Logic ---
        if self.robot_oriented_debouncer.calculate(inputs['robot_oriented']):
            self.field_oriented = False
        else:
            self.field_oriented = True

        # --- Translation Processing ---
        desired_fwd, desired_strafe, raw_vector = self._process_translation(inputs, slowmode_multiplier)

        # --- Rotation Processing ---
        raw_rot = -inputs['right_x']
        
        if inputs['tracking_on']:
            # Rising Edge: Reset targeting state  # TODO - just handle this in targeting
            if not self.last_tracking_on:
                self.targeting.reset_state()
            
            # Delegate to subsystem
            desired_rot = self.targeting.get_rotation_output()
            
        elif self.last_tracking_on: # Falling Edge
            # Reset manual limiter to the LAST TARGETING SPEED to ensure smooth handoff
            # This prevents the robot from jerking if stick is 0 but robot is spinning fast
            manual_rot = raw_rot * angular_slowmode_multiplier
            self.manual_rot_limiter.reset(self.targeting.last_rot_output)
            desired_rot = manual_rot
        else:
            desired_rot = self._calculate_manual_rotation(raw_rot, angular_slowmode_multiplier)

        self.last_tracking_on = inputs['tracking_on']

        # -----------------------------------------------------------
        # 3. ACT
        # -----------------------------------------------------------
        self.swerve.drive(
            xSpeed=desired_fwd,
            ySpeed=desired_strafe,
            rot=desired_rot,
            fieldRelative=self.field_oriented,
            rate_limited=False, # We handle all limiting in this command now
            keep_angle=False
        )

        # -----------------------------------------------------------
        # 4. REPORT
        # -----------------------------------------------------------
        if wpilib.RobotBase.is_simulation():
            self.js_dv1_x_pub.set(math.fabs(raw_vector.x))
            self.js_dv1_y_pub.set(math.fabs(raw_vector.y))
            self.js_dv_norm_x_pub.set(math.fabs(desired_fwd))
            self.js_dv_norm_y_pub.set(math.fabs(desired_strafe))
            self.commanded_values_pub.set([desired_fwd, desired_strafe, desired_rot])

    def _process_translation(self, inputs, multiplier):
        # Apply calibration offsets and negation
        joystick_fwd = -(inputs['left_y'] - self.swerve.thrust_calibration_offset)
        joystick_strafe = -(inputs['left_x'] - self.swerve.strafe_calibration_offset)
        
        raw_vector = Translation2d(joystick_fwd, joystick_strafe)
        processing_vector = raw_vector

        # Deadband & Clipping
        if processing_vector.norm() > dc.k_outer_deadband:
            processing_vector *= 1 / processing_vector.norm()
        elif processing_vector.norm() < dc.k_inner_deadband:
            processing_vector = Translation2d(0, 0)

        # Response curve.  Scaling the vector BY sqrt(|v|) makes the magnitude |v|**1.5 - i.e. a
        # power curve that is LESS sensitive near center.  (Do not call this a 'sqrt curve': an
        # actual |v|**0.5 curve is MORE sensitive near center, the opposite of what we want.)
        processing_vector *= math.sqrt(processing_vector.norm())
        
        # Scaling
        processing_vector *= multiplier

        # Rate Limiting
        desired_fwd = self.drive_limiter.calculate(processing_vector.x)
        desired_strafe = self.strafe_limiter.calculate(processing_vector.y)

        # Alliance Adjustment
        if inputs['alliance'] == wpilib.Alliance.RED and self.field_oriented:
            desired_fwd *= -1
            desired_strafe *= -1
            
        return desired_fwd, desired_strafe, raw_vector

    def _calculate_manual_rotation(self, raw_rot, multiplier):
        if abs(raw_rot) < dc.k_inner_deadband:
            raw_rot = 0
        
        desired_rot = raw_rot * multiplier
        return desired_rot # self.manual_rot_limiter.calculate(desired_rot)

    def end(self, interrupted: bool) -> None:
        self.swerve.drive(0, 0, 0, fieldRelative=self.field_oriented, rate_limited=True)