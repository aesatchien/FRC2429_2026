import random

import commands2
import wpilib

from helpers.log_command import log_command


@log_command(console=True, nt=False, print_init=True, print_end=False)
class CANStatus(commands2.Command):
    """Dump and clear sticky faults for every swerve motor.

    Goes through the motor adapters rather than touching a Spark directly, because the
    two vendors report faults completely differently - REV hands you one bitmask,
    Phoenix hands you 27 separate boolean signals.  motors.py normalises both to a list
    of names, so this command does not care what is bolted to the module.
    """

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('CANStatus')
        self.container = container
        # self.addRequirements()  # deliberately none - this is read-only diagnostics

        # Built from the modules themselves.  This used to be a hand-written dict of CAN ids
        # 20-27 with a TODO asking to derive it; hand-maintained id tables go stale the first
        # time someone re-ids a controller.
        self.motors = {}
        for module in self.container.swerve.swerve_modules:
            self.motors[f'{module.label}_drive'] = module.drive_motor
            self.motors[f'{module.label}_turn'] = module.turn_motor

        self.write_log = False

    def runsWhenDisabled(self) -> bool:
        return True

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        pass  # put in execute so log_command can send the time

        # if self.write_log:
        #     try:
        #         with open('can.pkl', 'rb') as file:
        #             records = pkl.load(file)
        #     except FileNotFoundError:  # start with a header row
        #         header = [f"{key} {self.can_ids[key]['name']}" for key in self.can_ids.keys()]
        #         records = [header]
        #
        #     output = {}
        #     for key in self.can_ids: # can't pickle a motor
        #         output.update({key:{'NAME': self.can_ids[key]['name'], 'STICKY_FAULTS': self.can_ids[key]['sticky_faults'],
        #                             'SET_BITS': self.can_ids[key]['set_bits'], 'FAULT_CODES': self.can_ids[key]['fault_codes']}})
        #
        #     just_faults = [self.can_ids[key]['fault_codes'] for key in self.can_ids.keys()]
        #
        #     records.append(just_faults)
        #
        #     with open('can.pkl', 'wb') as file:
        #         pkl.dump(records, file)
        #     print('Wrote CAN faults to can.pkl')
        #
        #     to read, just do this script
        #     import pickle as pkl
        #     import pandas as pd
        #     with open('can.pkl', 'rb') as file:
        #         data = pkl.load(file)
        #     column_names = data[0]
        #     df = pd.DataFrame(data[1:], columns=column_names)
        #     df

    def execute(self) -> None:
        # single execution and end
        for name, motor in self.motors.items():
            if wpilib.RobotBase.isSimulation():
                # sim has no real faults - make some up so the dashboard path gets exercised
                faults = random.sample(['kBrownout', 'kCANRX', 'kHasReset', 'kStall'],
                                       k=random.randint(0, 2))
            else:
                faults = motor.get_sticky_faults()

            motor.clear_faults()

            vendor = motor.describe()['vendor']
            can_id = motor.can_id
            print(f"CANID {can_id:02d}: {name:13} [{vendor}] sticky_faults: {faults if faults else 'none'}")

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        pass
