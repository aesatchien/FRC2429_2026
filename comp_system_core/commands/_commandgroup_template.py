import commands2

class CommandGroupTemplate(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()
        self.set_name(f'Command Group Template- change me!')  # change this to whatever you named it above
        self.container = container
        self.add_commands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.get_name()} **"))

        # self.addCommands(... your stuff here, call other commands with indent=indent+1 ...)

        self.add_commands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.get_name()} **"))

