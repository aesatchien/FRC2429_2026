import commands2.button
import constants

driver_controller = commands2.button.CommandXboxController(constants.k_driver_controller_port)
copilot_controller = commands2.button.CommandXboxController(constants.k_co_driver_controller_port)
axis_trigger_threshold = 0.5
# Driver Buttons
driver_a = driver_controller.a()
driver_b = driver_controller.b()
driver_x = driver_controller.x()
driver_y = driver_controller.y()
driver_lb = driver_controller.leftBumper()
driver_rb = driver_controller.rightBumper()
driver_back = driver_controller.back()
driver_start = driver_controller.start()
driver_up = driver_controller.povUp()
driver_down = driver_controller.povDown()
driver_left = driver_controller.povLeft()
driver_right = driver_controller.povRight()
driver_l_trigger = driver_controller.leftTrigger(axis_trigger_threshold)
driver_r_trigger = driver_controller.rightTrigger(axis_trigger_threshold)

# Co-Driver Buttons
copilot_a = copilot_controller.a()
copilot_b = copilot_controller.b()
copilot_x = copilot_controller.x()
copilot_y = copilot_controller.y()
copilot_lb = copilot_controller.leftBumper()
copilot_rb = copilot_controller.rightBumper()
copilot_back = copilot_controller.back()
copilot_start = copilot_controller.start()
copilot_up = copilot_controller.povUp()
copilot_down = copilot_controller.povDown()
copilot_left = copilot_controller.povLeft()
copilot_right = copilot_controller.povRight()
copilot_l_trigger = copilot_controller.leftTrigger(axis_trigger_threshold)
copilot_r_trigger = copilot_controller.rightTrigger(axis_trigger_threshold)

"""
Remember - buttons are 1-indexed, not zero
"""
# The driver's controller
bbox_1 = commands2.button.CommandJoystick(constants.k_bbox_1_port)
bbox_2 = commands2.button.CommandJoystick(constants.k_bbox_2_port)

bbox_TBD1 = bbox_1.button(3)  # top left red 1
bbox_TBD2 = bbox_1.button(4)  # top left red 2

bbox_right = bbox_1.button(1)  # true when selected
bbox_left = bbox_1.button(2)  #  and true when selected

# print('Initializing robot state based on button box joystick:')
# if bbox_right.getAsBoolean():
#     robot_state.set_side(robot_state.Side.RIGHT)
# else:
#     robot_state.set_side(robot_state.Side.LEFT)

bbox_human_left = bbox_1.button(5)
bbox_human_right = bbox_1.button(6)

bbox_AB = bbox_1.button(7)
bbox_CD = bbox_1.button(8)
bbox_EF = bbox_1.button(9)
bbox_GH = bbox_1.button(10)
bbox_IJ = bbox_1.button(11)
bbox_KL = bbox_1.button(12)

bbox_L1 = bbox_2.button(1)
bbox_L2 = bbox_2.button(2)
bbox_L3 = bbox_2.button(3)
bbox_L4 = bbox_2.button(4)
bbox_intake_in = bbox_2.button(5)
bbox_intake_in = bbox_2.button(7)
bbox_climb_up = bbox_2.button(8)  # this is lower down on the box
bbox_climb_down = bbox_2.button(6)

# Co-Driver Sticks (Axes as Triggers)
# copilot_r_stick_positive_x = copilot_controller.axisGreaterThan(4, 0.5)
# copilot_r_stick_negative_x = copilot_controller.axisLessThan(4, -0.5)
# copilot_r_stick_positive_y = copilot_controller.axisGreaterThan(5, 0.5)
# copilot_r_stick_negative_y = copilot_controller.axisLessThan(5, -0.5)