# repo for utility functions anyone can use
import functools
import warnings
from typing import Union, List, Sequence, Tuple

import rev
from rev import ClosedLoopSlot, SparkMaxConfig, SparkFlexConfig, REVLibError

import constants

 # ----------------  COMMON FUNCTIONS  ------------------------------------
SparkConfig = Union[SparkMaxConfig, SparkFlexConfig]  # let our functions take either Spark flavor


def configure_sparks(pairs: Sequence[Tuple[object, SparkConfig]], subsystem_name: str = '') -> bool:
    """
    Push a config to each Spark and REPORT ANY FAILURE.  Returns True if every motor took its config.

    Pass explicit (motor, config) pairs.  Intake, Shooter and Climber each used to do

        rev_errors = [motor.configure(c, resets, persists) for motor, c in zip(self.motors, self.configs)]

    which had two problems: the result was never looked at, so a motor that failed to configure
    booted silently; and `self.motors` and `self.configs` were two hand-maintained lists in two
    different files that had to stay in the same order (the comment in shooter.py said
    "SAME ORDER AS COBSTANTS!").  Inserting one motor mis-paired everything after it - no error,
    just a flywheel running on the roller's current limit.  Pairing them at the call site
    makes that mistake impossible to make silently.
    """
    resets = rev.ResetMode.kResetSafeParameters       # always start from a clean slate
    persists = rev.PersistMode.kPersistParameters if constants.k_burn_flash else rev.PersistMode.kNoPersistParameters

    all_ok = True
    for motor, config in pairs:
        error = motor.configure(config, resets, persists)
        if error != REVLibError.kOk:
            all_ok = False
            print(f'*** CONFIG FAILED: {subsystem_name} CAN id {motor.getDeviceId()} returned {error} ***')

    if all_ok:
        print(f'  configured {len(pairs)} {subsystem_name} sparks (burn_flash={constants.k_burn_flash})')
    return all_ok

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

def compare_motors(obj_a, obj_b, name_a="Motor A", name_b="Motor B"):
    """
    Compares two REV Spark motors or motor states using the 2025 configAccessor API.
    You can pass either the motor objects themselves, or pre-computed state dictionaries.
    """

    # 1. Collect Data
    try:
        data_a = obj_a if isinstance(obj_a, dict) else _get_motor_state(obj_a)
        data_b = obj_b if isinstance(obj_b, dict) else _get_motor_state(obj_b)
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

def deprecated(reason):
    """
    A decorator to mark a class or function as deprecated.
    Emits a warning when the class is instantiated or function is called.
    """
    # This is the outer function that takes the reason string.
    def decorator(cls_or_func):
        # This is the actual decorator that wraps the class or function.
        if isinstance(cls_or_func, type):
            # It's a class
            orig_init = cls_or_func.__init__

            # @functools.wraps preserves the original function's name and docstring.
            @functools.wraps(orig_init)
            def new_init(self, *args, **kwargs):
                # stacklevel=2 makes the warning point to the code that *called*
                # the deprecated class, not to this line in the decorator.
                warnings.warn(
                    f"{cls_or_func.__name__} is deprecated: {reason}",
                    DeprecationWarning,
                    stacklevel=2
                )
                orig_init(self, *args, **kwargs)

            cls_or_func.__init__ = new_init
            return cls_or_func
        else:
            # It's a function
            # @functools.wraps preserves the original function's name and docstring.
            @functools.wraps(cls_or_func)
            def wrapper(*args, **kwargs):
                warnings.warn(
                    f"{cls_or_func.__name__} is deprecated: {reason}",
                    DeprecationWarning,
                    stacklevel=2
                )
                return cls_or_func(*args, **kwargs)
            return wrapper
    return decorator

# Example usage:
# compare_motors_2025(left_drive, right_drive, "Left_Master", "Right_Master")