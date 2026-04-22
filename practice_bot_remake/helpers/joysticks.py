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

# renaming some of them for clarity


# Co-Driver Sticks (Axes as Triggers)
# copilot_r_stick_positive_x = copilot_controller.axisGreaterThan(4, 0.5)
# copilot_r_stick_negative_x = copilot_controller.axisLessThan(4, -0.5)
# copilot_r_stick_positive_y = copilot_controller.axisGreaterThan(5, 0.5)
# copilot_r_stick_negative_y = copilot_controller.axisLessThan(5, -0.5)