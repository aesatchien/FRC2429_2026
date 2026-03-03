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

# Co-Driver Sticks (Axes as Triggers)
# copilot_r_stick_positive_x = copilot_controller.axisGreaterThan(4, 0.5)
# copilot_r_stick_negative_x = copilot_controller.axisLessThan(4, -0.5)
# copilot_r_stick_positive_y = copilot_controller.axisGreaterThan(5, 0.5)
# copilot_r_stick_negative_y = copilot_controller.axisLessThan(5, -0.5)