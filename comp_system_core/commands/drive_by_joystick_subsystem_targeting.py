
import math
import commands2
import wpilib
import ntcore

import constants
from subsystems.swerve import Swerve  # allows us to access the definitions
from subsystems.targeting import Targeting
from commands2.button import CommandPS4Controller, CommandXboxController, CommandJoystick
from wpimath.geometry import Translation2d
from wpimath.filter import Debouncer, SlewRateLimiter
from subsystems.swerve_constants import DriveConstants as dc, RateLimiters as rl
from helpers.log_command import log_command


@log_command(console=True, nt=False, print_init=True, print_end=False)
class DriveByJoystickSubsystemTargeting(commands2.Command):
    def __init__(self, container, swerve: Swerve, targeting: Targeting, controller: CommandXboxController=None, ps5controller: CommandPS4Controller=None, button_box: CommandJoystick=None) -> None:
        super().__init__()
        self.setName('drive_by_joystick_subsystem_targeting')
        
        # -----------------------------------------------------------
        # 1. Subsystems & Dependencies
        # -----------------------------------------------------------
        self.container = container
        self.swerve = swerve
        self.targeting = targeting
        self.addRequirements(self.swerve)

        self.xbox_controller: CommandXboxController = controller
        self.ps5_controller: CommandPS4Controller = ps5controller
        self.button_box: CommandJoystick = button_box

        # -----------------------------------------------------------
        # 2. Configuration & State
        # -----------------------------------------------------------
        self.field_oriented = True
        self.last_tracking_on = False

        # -----------------------------------------------------------
        # 3. Input Processing (Debouncers, Limiters)
        # -----------------------------------------------------------
        self.robot_oriented_debouncer = Debouncer(0.1, Debouncer.DebounceType.kBoth)
        
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
        self.inst = ntcore.NetworkTableInstance.getDefault()
        status_prefix = constants.status_prefix
        # Simulation Debugging Publishers
        self.js_dv1_x_pub = self.inst.getDoubleTopic(f"{status_prefix}/_joystick_dv1_x").publish()
        self.js_dv1_y_pub = self.inst.getDoubleTopic(f"{status_prefix}/_joystick_dv1_y").publish()
        self.js_dv_norm_x_pub = self.inst.getDoubleTopic(f"{status_prefix}/_joystick_dv_norm_x").publish()
        self.js_dv_norm_y_pub = self.inst.getDoubleTopic(f"{status_prefix}/_joystick_dv_norm_y").publish()
        self.commanded_values_pub = self.inst.getDoubleArrayTopic(f"{status_prefix}/_joystick_commanded_values").publish()

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        pass
    
    def read_xbox(self, hid):
        """Returns (left_y, left_x, right_x, right trigger, robot_oriented) from the Xbox HID"""
        return (
            hid.getLeftY(),
            hid.getLeftX(),
            hid.getRightX(),
            hid.getRightTriggerAxis(),
            hid.getLeftBumperButton()
        )

    def read_ps5(self, hid):
        """Returns (left_y, left_x, right_x, right trigger, robot_oriented) from the PS5 HID"""
        # This was from gemini, but it is helpful to test the controller mapping in sim.
        # axis_count = hid.getAxisCount()
        # values = [round(hid.getRawAxis(i), 3) for i in range(axis_count)]
        # print(f"DEBUGGING: port={hid.getPort()} axis_count={axis_count} values={values}")
        return (
            hid.getLeftY(),
            hid.getLeftX(),
            hid.getRawAxis(2),  # Right X is axis 2 on PS5 controller
            hid.getRawAxis(4) + 1,  # Right Trigger is axis 4 on PS5 controller, and is -1 by default, so we add 1 to make it 0
            hid.getL1Button()
        )

    def execute(self) -> None:
        # -----------------------------------------------------------
        # 1. READ INPUTS
        # -----------------------------------------------------------
        xbox_connected = self.xbox_controller is not None and wpilib.DriverStation.isJoystickConnected(0)
        ps5_connected = self.ps5_controller is not None and wpilib.DriverStation.isJoystickConnected(5)
        
        if xbox_connected:
            left_y, left_x, right_x, right_trigger, robot_oriented = self.read_xbox(self.xbox_controller.getHID())
        elif ps5_connected:
            left_y, left_x, right_x, right_trigger, robot_oriented = self.read_ps5(self.ps5_controller.getHID())
        else:
            left_y, left_x, right_x, right_trigger, robot_oriented = 0.0, 0.0, 0.0, 0.0, False  # cooked??? IDK - Trentan

        if self.button_box is not None:
            after_burner = self.button_box.getHID().getRawButton(3)
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
            'alliance': wpilib.DriverStation.getAlliance()
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
        if wpilib.RobotBase.isSimulation():
            self.js_dv1_x_pub.set(math.fabs(raw_vector.X()))
            self.js_dv1_y_pub.set(math.fabs(raw_vector.Y()))
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

        # Response Curve (Sqrt)
        processing_vector *= math.sqrt(processing_vector.norm())
        
        # Scaling
        processing_vector *= multiplier

        # Rate Limiting
        desired_fwd = self.drive_limiter.calculate(processing_vector.X())
        desired_strafe = self.strafe_limiter.calculate(processing_vector.Y())

        # Alliance Adjustment
        if inputs['alliance'] == wpilib.DriverStation.Alliance.kRed and self.field_oriented:
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