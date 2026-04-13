import math
import wpilib
from wpilib import Color, Color8Bit
from wpimath.units import inchesToMeters
import constants

class BlockheadMech:
    """
    A class to manage the Mechanism2d visualizations for the robot.
    This replaces the script-based simmech.py and provides an API for
    updating mechanism states.
    """

    def __init__(self):
        """
        Initialize the Mechanism2d objects and publish them to SmartDashboard.
        """
        # Dimensions
        self.width = inchesToMeters(40)  # inches (approx robot length + intake)
        self.height = inchesToMeters(30) # inches (approx max height)
        
        # ----------------- Mechanism2d Views -----------------
        # Side view is best for Intake, Elevator, Shooter, Climber
        self.mech_side = wpilib.Mechanism2d(self.width + inchesToMeters(10), self.height + inchesToMeters(10))

        # ----------------- Visual Constants -----------------
        self.color_chassis = Color8Bit(Color.kGray)
        self.color_wheel = Color8Bit(Color.kWhite)
        self.color_intake = Color8Bit(Color.kOrange)
        self.color_hopper = Color8Bit(Color.kYellow)
        self.color_indexer = Color8Bit(Color.kCyan)
        self.color_shooter = Color8Bit(Color.kRed)
        self.color_ball = Color8Bit(Color.kYellow)
        self.hopper_line_weight = 6
        self.hood_line_weight = 7
        self.line_weight = 10 # 1 inch approx 10 pixels?
        # self.color_climber = Color8Bit(Color.kGreen)
        
        # ----------------- Initialization -----------------
        self._init_structure()
        self._init_intake()
        self._init_hopper()
        self._init_indexer()
        self._init_shooter()
        self._init_ball()
        # self._init_climber()

        # Put to dashboard
        # mech_prefix = constants.mech_prefix
        wpilib.SmartDashboard.putData("Mech Side View", self.mech_side)

    def _get_rel_angle(self, target_abs, parent_abs):
        """
        Calculates the relative angle needed for a ligament given the 
        desired absolute angle and the parent's absolute angle.
        """
        return target_abs - parent_abs

    def _init_structure(self):
        # Drivetrain
        """Stub: Base chassis line."""
        # Robot starting X offset
        self.start_x = inchesToMeters(5)
        
        # Chassis: 2" bar, 27" long. Top at 4.5".
        # Center of bar is at 3.5" (since it's 2" thick, 2.5" to 4.5")
        self.root_chassis = self.mech_side.getRoot("chassis_root", self.start_x, inchesToMeters(3.5))
        self.chassis_ligament = self.root_chassis.appendLigament(
            "chassis", inchesToMeters(27), 0, 20, self.color_chassis
        )

        # Wheels: 4" diameter (2" radius). Center at Y=2.
        # 5" in from edges (0 and 27). So at 5" and 22".
        
        # Rear Wheel
        self.root_rear_wheel = self.mech_side.getRoot("rear_wheel", self.start_x + inchesToMeters(5), inchesToMeters(2))
        self.rear_wheel_ligament = self.root_rear_wheel.appendLigament(
            "rear_spoke", inchesToMeters(2), 0, 6, self.color_wheel
        )
        self.rear_wheel_ligament_2 = self.root_rear_wheel.appendLigament(
            "rear_spoke_2", inchesToMeters(2), 180, 6, self.color_wheel
        )
        
        # Front Wheel
        self.root_front_wheel = self.mech_side.getRoot("front_wheel", self.start_x + inchesToMeters(22), inchesToMeters(2))
        self.front_wheel_ligament = self.root_front_wheel.appendLigament(
            "front_spoke", inchesToMeters(2), 0, 6, self.color_wheel
        )
        self.front_wheel_ligament_2 = self.root_front_wheel.appendLigament(
            "front_spoke_2", inchesToMeters(2), 180, 6, self.color_wheel
        )

        # Hood
        '''Stub: Vertical bar at the rear'''
        # Starting X offset, against the chassis start
        self.start_x = inchesToMeters(5.5)

        # Starts with a vertical section, ~10 in height
        self.root_hood = self.mech_side.getRoot("hood_root", self.start_x, inchesToMeters(4.5))
        self.hood_back_ligament = self.root_hood.appendLigament(
            "hood", inchesToMeters(10), 90, self.hood_line_weight, self.color_chassis
        )
        # then a ~8 inch curve
        self.hood_back_curve = self.hood_back_ligament.appendLigament(
            "hood_back_curve", inchesToMeters(8), 330, self.hood_line_weight, self.color_chassis
        )
        # then a ~8 inch flat section
        self.hood_front_curve = self.hood_back_curve.appendLigament(  
            "hood_front_curve", inchesToMeters(8), 280, self.hood_line_weight, self.color_chassis
        )
        # follwed by a ~14.5 in vertical line
        self.hood_front_ligament = self.hood_front_curve.appendLigament(
            "hood_front", inchesToMeters(14.5), 285, self.hood_line_weight, self.color_chassis
        )

    def _init_intake(self):
        """Stub: Intake pivot and rollers."""
        # Mounting bar: Sticks up 6" from top of drivetrain (Y=4.5).
        # We place it at the front of the chassis (X = start + 27).
        self.root_intake_mount = self.mech_side.getRoot("intake_mount", self.start_x + inchesToMeters(27), inchesToMeters(4.5))
        self.abs_intake_mount = 0 # Roots are 0
        
        # The vertical post
        self.abs_intake_post = 90
        self.intake_post = self.root_intake_mount.appendLigament(
            "intake_post", inchesToMeters(6), self._get_rel_angle(self.abs_intake_post, self.abs_intake_mount), 6, self.color_intake
        )
        
        # The pivoting arm. Pivot is at top of post (Y=10.5).
        # Needs to reach ground (Y=0). Length approx 11-12".
        self.abs_intake_arm_start = -60 # Initial angle
        self.intake_arm_length = inchesToMeters(10)
        self.intake_arm = self.intake_post.appendLigament(
            "intake_arm", self.intake_arm_length, self._get_rel_angle(self.abs_intake_arm_start, self.abs_intake_post), 6, self.color_intake
        )
        
        # Animation: Spacer and Indicator
        self.intake_anim_dist = 0
        self.intake_spacer = self.intake_arm.appendLigament(
            "intake_spacer", inchesToMeters(0.1), 180, 0, self.color_intake # relative angle 180 to go back up arm
        )
        self.intake_indicator = self.intake_spacer.appendLigament(
            "intake_indicator", inchesToMeters(2), 90, 8, self.color_intake # 90 relative to spacer
        )

    def _init_hopper(self):
        """Stub: Storage area."""
        # Attached to chassis middle
        # Using a root here to decouple from chassis rotation if needed, or just attach to chassis
        # Horizontal conveyor
        # Butt up against intake (Right, X=32) and go Left (180 deg)
        self.root_hopper = self.mech_side.getRoot("hopper_root", self.start_x + inchesToMeters(25), inchesToMeters(5))
        self.abs_hopper = 180
        self.hopper_length = inchesToMeters(16)
        self.hopper_tower = self.root_hopper.appendLigament(
            "hopper_belt", self.hopper_length, self._get_rel_angle(self.abs_hopper, 0), self.hopper_line_weight, self.color_hopper
        )
        
        # Animation: A spacer that grows to push the indicator along the belt
        self.hopper_anim_dist = 0
        self.hopper_spacer = self.root_hopper.appendLigament(
            "hopper_spacer", inchesToMeters(0.1), self._get_rel_angle(self.abs_hopper, 0), 0, self.color_hopper # weight 0 = invisible
        )
        self.hopper_indicator = self.hopper_spacer.appendLigament(
            "hopper_indicator", inchesToMeters(2), self._get_rel_angle(90, self.abs_hopper), self.hopper_line_weight, self.color_hopper # 90 abs (Up)
        )

    def _init_indexer(self):
        """Stub: Feeder mechanism."""
        # Attached to top of hopper
        # Continue left (Relative 0 -> Absolute 180)
        self.indexer_1_root = self.hood_front_ligament.appendLigament(
            "indexer_root", inchesToMeters(2), 180, 0, self.color_indexer  # marker
        )
        self.indexer_1 = self.indexer_1_root.appendLigament(
            "indexer", inchesToMeters(0), 180, 0, self.color_indexer
        )
        num_spokes = 4
        radius = inchesToMeters(1.5)  # 1.5'' diameter wheel, but this is exagerated to double
        self.indexer_spokes = []

        # Attaching spokes to the root to make a wheel
        for i in range(num_spokes):
            angle = (360 / num_spokes) * i
            spoke = self.indexer_1.appendLigament(
                f"spoke_{i}", 
                radius, 
                angle, 
                4,
                self.color_indexer
            )
            self.indexer_spokes.append(spoke)

        # Two Indexer wheels above each other
        self.indexer_2_spacer = self.hood_front_curve.appendLigament(
            "indexer_2_spacer", inchesToMeters(4.5), 270, 0, Color8Bit(0,0,0) # marker
        )
        self.indexer_2 = self.indexer_2_spacer.appendLigament(
            "indexer_2", inchesToMeters(0), 180, 0, self.color_indexer
        )

        self.indexer_3_spacer = self.indexer_2_root.appendLigament(
            "indexer_3_spacer", inchesToMeters(2), 200, 0, Color8Bit(0,0,0) # marker
        )
        self.indexer_3 = self.indexer_3_spacer.appendLigament(
            "indexer_3", inchesToMeters(0), 180, 0, self.color_indexer  # marker
        )
        # Adding spokes to the roots
        for i in range(num_spokes):
            angle = (360 / num_spokes) * i
            spoke = self.indexer_2.appendLigament(
                f"spoke_{i}", 
                radius, 
                angle, 
                4,
                self.color_indexer
            )
            self.indexer_spokes.append(spoke)
            spoke = self.indexer_3.appendLigament(
                f"spoke_{i}", 
                radius, 
                angle, 
                4,
                self.color_indexer
            )
            self.indexer_spokes.append(spoke)

    def _init_shooter(self):
        """Stub: Flywheel mechanism."""
        # Attached to indexer or separate tower
        # Left of indexer, angling up/left
        # Parent is Indexer (135). Target is 70 (pointing up/rightish).
        
        # Flywheel (White) - attached to center of shooter
        # Use a spacer to get to the middle (length 4)
        # Root of the flywheel is after the second curve of the shooter
        self.flywheel_center = self.hood_front_curve.appendLigament(
            "flywheel_center", inchesToMeters(0), 0, 0, self.color_shooter  # empty point
        )
        num_spokes = 6  # Six spokes on actual flywheel
        radius = inchesToMeters(3)  # 3'' diameter wheel on the real one, but this is exagerated to double

        for i in range(num_spokes):
            angle = (360 / num_spokes) * i
            self.flywheel_center.appendLigament(
                f"spoke_{i}", 
                radius, 
                angle, 
                4,
                self.color_wheel
            )

        '''Stub: Rollers'''
        # Rollers located after vertical section of the hood and after first curve
        self.roller_1_center = self.hood_back_curve.appendLigament(
            "roller_1_center", inchesToMeters(0), 0, 0, self.color_shooter  # empty point
        )
        self.roller_2_center = self.hood_back_ligament.appendLigament(
            "roller_2_center", inchesToMeters(0), 330, 0, self.color_chassis  # marker, slightly lower
        )
        num_spokes = 4
        radius = inchesToMeters(1)  # 1'' diameter wheel in real life, but this is exagerated to double

        for i in range(num_spokes):
            angle = (360 / num_spokes) * i
            self.roller_1_center.appendLigament(
                f"spoke_{i}", 
                radius, 
                angle, 
                4,
                self.color_wheel
            )
            self.roller_2_center.appendLigament(
                f"spoke_{i}", 
                radius, 
                angle, 
                4,
                self.color_wheel
            )

    '''def _init_climber(self):
        """Stub: Extension arms."""
        # Attached to chassis center/back
        self.climber_root_y = inchesToMeters(2)
        self.root_climber = self.mech_side.getRoot("climber_root", self.start_x + inchesToMeters(2), self.climber_root_y)
        self.abs_climber = 90
        self.climber_stage_1 = self.root_climber.appendLigament(
            "climber_stage_1", inchesToMeters(15), self._get_rel_angle(self.abs_climber, 0), self.line_weight, self.color_climber
        )
        
        # Hook pointing left (180 abs)
        self.abs_hook = 180
        self.climber_hook = self.climber_stage_1.appendLigament(
            "climber_hook", inchesToMeters(4), self._get_rel_angle(self.abs_hook, self.abs_climber), self.line_weight, self.color_wheel
        )
    '''

    def _init_ball(self):
        """Stub: A game piece ball."""
        # Start on floor (Y=3 for 6" diam), to the right of intake
        # Intake tip is approx X=39 (start_x + 27 + ~7). Place ball at X=45 (start_x + 40).
        self.ball_root = self.mech_side.getRoot("ball_root", self.start_x + inchesToMeters(40), inchesToMeters(3))
        
        # "just make one 6" tall 6" wide bar"
        # Root is at center (Y=3). Go down 3 to bottom, then draw up 6.
        
        # Invisible spacer to get to bottom
        self.ball_spacer = self.ball_root.appendLigament(
            "ball_spacer", inchesToMeters(3), -90, 0, self.color_ball
        )
        self.ball_ligament = self.ball_spacer.appendLigament(
            "ball_solid", inchesToMeters(6), 180, 20, self.color_ball
        )

    # ---------------- Update Methods ----------------

    def update_drivetrain(self, speed_mps):
        # Rotate wheels based on speed. 
        # Speed (m/s) -> angular velocity. 
        # Just adding a factor to animate it.
        rotation_step = speed_mps * 10 # arbitrary scaling
        
        new_rear_angle = self.rear_wheel_ligament.getAngle() + rotation_step
        self.rear_wheel_ligament.setAngle(new_rear_angle)
        self.rear_wheel_ligament_2.setAngle(new_rear_angle + 180)
        
        new_front_angle = self.front_wheel_ligament.getAngle() + rotation_step
        self.front_wheel_ligament.setAngle(new_front_angle)
        self.front_wheel_ligament_2.setAngle(new_front_angle + 180)

    def update_intake(self, angle: float, speed: float):
        # Pivot relative to the vertical post (which is at 90 absolute).
        # Stowed: Folded back/up. Relative angle +60 -> Absolute 150.
        # Deployed: Down to ground. Relative angle -200 -> Absolute -110.
        
        # Note: appendLigament angle is relative to parent. Parent is 90 deg (vertical).
        self.intake_arm.setAngle(self._get_rel_angle(angle, self.abs_intake_post))
        
        # Animate bar moving along the intake arm
        if abs(speed) > 0:
            self.intake_anim_dist += abs(speed) * inchesToMeters(0.2)
            if self.intake_anim_dist > self.intake_arm_length:
                self.intake_anim_dist = 0
            self.intake_spacer.setLength(self.intake_anim_dist)

    def update_hopper(self, speed: float):
        # Animate the ball moving left
        if abs(speed) > 0:
            self.hopper_anim_dist += speed * inchesToMeters(0.2)  # Scale speed for animation
            if self.hopper_anim_dist > self.hopper_length:
                self.hopper_anim_dist = 0
            
            self.hopper_spacer.setLength(self.hopper_anim_dist)

    def update_indexer(self, rpm: float):
        # Animate bar moving up the indexer
        if abs(rpm) > 10:

            visual_speed = math.sqrt(abs(rpm))
            
            # Updating positon based on the three roots for the indexer
            rotation_step = math.copysign(visual_speed * 0.2, rpm)  # Uses the magnitude of visual_spped * .2 with the sign of rpm
            current_angle = self.indexer_1.getAngle()  # Change angle of root, spokes will follow
            self.indexer_1.setAngle(current_angle - rotation_step)  # Subtracting to spin clockwise
            current_angle = self.indexer_2.getAngle()  # Change angle of root, spokes will follow
            self.indexer_2.setAngle(current_angle - rotation_step)  # Subtracting to spin clockwise
            current_angle = self.indexer_3.getAngle()  # Change angle of root, spokes will follow
            self.indexer_3.setAngle(current_angle - rotation_step)  # Subtracting to spin clockwise

    def update_shooter(self, rpm: float):
        # Using a sqrt scale to not have aliasing
        # Wagon wheel effect when using a linear scale with 50fps
        if abs(rpm) > 10:

            visual_speed = math.sqrt(abs(rpm))
            
            rotation_step = math.copysign(visual_speed * 0.2, rpm)  # Uses the magnitude of visual_spped * .2 with the sign of rpm
            current_angle = self.flywheel_center.getAngle()  # Change angle of root, spokes will follow
            self.flywheel_center.setAngle(current_angle - rotation_step)  # Subtracting to spin clockwise

    def update_rollers(self, rpm: float):
        # Using a sqrt scale to not have aliasing
        if abs(rpm) > 10:

            visual_speed = math.sqrt(abs(rpm)) 
            
            rotation_step = math.copysign(visual_speed * 0.2, rpm)  # Uses the magnitude of visual_spped * .2 with the sign of rpm
            current_angle = self.roller_1_center.getAngle()  # Change angle of root, spokes will follow
            self.roller_1_center.setAngle(current_angle + rotation_step)  # Adding to spin counterclockwise
            current_angle = self.roller_2_center.getAngle()  # Change angle of root, spokes will follow
            self.roller_2_center.setAngle(current_angle + rotation_step)  # Adding to spin counterclockwise

    '''def update_climber(self, height_from_ground: float):
        # Calculate length: Target Height - Root Height
        length = max(0.0, height_from_ground - self.climber_root_y)
        self.climber_stage_1.setLength(length)
    '''

    def update_ball(self, x, y):
        self.ball_root.setPosition(x, y)
