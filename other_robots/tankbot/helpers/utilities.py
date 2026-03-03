# repo for utility functions anyone can use
from typing import Union, List
from rev import ClosedLoopSlot, SparkMaxConfig, SparkFlexConfig

 # ----------------  COMMON FUNCTIONS  ------------------------------------
SparkConfig = Union[SparkMaxConfig, SparkFlexConfig]  # let our functions take either Spark flavor

def set_config_defaults(configs: Union[SparkConfig, List[SparkConfig]]) -> None:
    """
    Applies default configuration settings to a single config object or a list of config objects.
    Args:
        configs: A single configuration object or a list of configuration objects.
    """
    # Check if the input is a list (or any sequence except a string/bytes)
    if isinstance(configs, (list, tuple)):
        config_list = configs
    else:
        config_list = [configs]  # If it's a single item, wrap it in a list for the loop
    for config in config_list:
        config.voltageCompensation(12)
        config.setIdleMode(config.IdleMode.kBrake)
        config.smartCurrentLimit(40)

def _get_motor_state(motor):
    """
    Extracts key configuration and state data from a REV Spark motor.
    Uses the 2025 configAccessor API to retrieve values.
    """
    ca = motor.configAccessor
    s0 = ClosedLoopSlot.kSlot0

    # Follower logic: Return ID or 'None'
    leader_id = ca.getFollowerModeLeaderId()
    follower_status = f"ID {leader_id}" if leader_id > 0 else "None"

    # Using descriptive, snake_case keys (Pythonic)
    return {
        "device_id": motor.getDeviceId(),
        "is_inverted": ca.getInverted(),
        "idle_mode": str(ca.getIdleMode()).split('.')[-1],
        "follower_leader_id": follower_status,
        "smart_current_limit_amps": ca.getSmartCurrentLimit(),
        "position_conversion": ca.encoder.getPositionConversionFactor(),
        "velocity_conversion": ca.encoder.getVelocityConversionFactor(),
        "pid_p_gain_slot_0": ca.closedLoop.getP(s0),
        "pid_i_gain_slot_0": ca.closedLoop.getI(s0),
        "pid_d_gain_slot_0": ca.closedLoop.getD(s0),
        "pid_ff_gain_slot_0": ca.closedLoop.getFF(s0),
        "max_output_slot_0": ca.closedLoop.getMaxOutput(s0),
        "min_output_slot_0": ca.closedLoop.getMinOutput(s0),
    }

def _format_value(value):
    """
    Descriptive Pythonic formatting:
    - 5 decimal places for floats - Strips unnecessary trailing zeros - Leaves strings/ints as is
    """
    if isinstance(value, float):
        # format to 5 decimal places, then strip trailing zeros and potential dot
        return f"{value:.5f}".rstrip('0').rstrip('.')
    return str(value)

def compare_motors(motor_a, motor_b, name_a="Motor A", name_b="Motor B"):
    """
    Compares two REV Spark motors using the 2025 configAccessor API.
    All numeric values are limited to 6 significant digits.
    """

    # 1. Collect Data
    try:
        data_a = _get_motor_state(motor_a)
        data_b = _get_motor_state(motor_b)
    except Exception as e:
        print(f"Error: 2025 API Access Failed. {e}")
        return

    # 2. Table Setup
    col_width_label = 30
    col_width_data = 25

    header = f"{'Setting Name':<{col_width_label}} | {name_a:<{col_width_data}} | {name_b:<{col_width_data}}"
    separator = "-" * len(header)

    print(f"\n{header}\n{separator}")

    # 3. Iterative Comparison
    for key in data_a.keys():
        val_a = _format_value(data_a[key])
        val_b = _format_value(data_b[key])

        # Mark differences for quick scanning
        marker = "!" if val_a != val_b else " "

        # Print row with clean descriptive name
        print(f"{marker} {key:<{col_width_label - 2}} | {val_a:<{col_width_data}} | {val_b:<{col_width_data}}")
    print()

# Example usage:
# compare_motors_2025(left_drive, right_drive, "Left_Master", "Right_Master")