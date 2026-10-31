"""
A place for the constant values in the code that may be used in more than one place.
This offers a convenient resources to teams who need to make both quick and universal
changes.

chairbot for team 2429 - the blockheads
"""
import math

from rev import SparkMaxConfig

# configuration for chairbot driving
k_thrust_limit = .25
k_twist_limit = .33
k_deadband = 0.05

# control mode for chairbot is either "remote" or "onboard"
k_control_mode = 'remote'

# slew rate limiters take units of 100's of percent per second (weird)
k_thrust_slew_limit = 0.5  # 0.5 means 50 percent per second

# led configuration
k_led_count = 36
k_led_pwm_port = 3

# --------------  SIMULATION AND FIELD INITIALIZATION  ---------------
k_start_x = 1.26  # eventually this will have to be set by the autonomous choices
k_start_y = 7.08

kWheelDiameterMeters = 8 * 0.0254  # 0.203 =  eight inches
kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
# 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
kDrivingMotorReduction = 11.787  # 52/10 * 68/30
kDrivingEncoderPositionFactor = (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction  # meters
kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor / 60.0  # meters per second

# --------------  SPARK MAX CONFIGURATION  ---------------
k_CANID_l1, k_CANID_l2 = 1, 2  # left leader and follower
k_CANID_r1, k_CANID_r2 = 3, 4  # right leader and follower

k_left_config, k_right_config = SparkMaxConfig(), SparkMaxConfig()
for config in [k_left_config, k_right_config]:
    config.encoder.positionConversionFactor(kDrivingEncoderPositionFactor)
    config.encoder.velocityConversionFactor(kDrivingEncoderVelocityFactor)
k_left_config.inverted(True)  # need to see if + pwr moves motor clockwise and therefore robot fwd
k_right_config.inverted(False)

# followers mirror their leader's output, including its inversion
k_follower_config_l2, k_follower_config_r2 = SparkMaxConfig(), SparkMaxConfig()
k_follower_config_l2.follow(k_CANID_l1, invert=False)
k_follower_config_r2.follow(k_CANID_r1, invert=False)