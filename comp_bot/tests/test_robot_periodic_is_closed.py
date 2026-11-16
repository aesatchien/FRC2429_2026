"""
robot.py starts things; it is not where features go.

robotPeriodic() runs every 20 ms next to the swerve.  New work belongs in the subsystem that owns it, in that
subsystem's periodic(), behind `self.counter % N` at the slowest rate its consumer needs - see "House rules" in
the CLAUDE.md at the top of the repo.  This test fails when robotPeriodic() starts touching something it did not before.

If you are here because it failed: do not add your name to ALLOWED.  Move the call into a subsystem.
"""

import ast
import pathlib

ALLOWED = {'mech', 'container'}   # what robotPeriodic() used when this was written; this set should only ever shrink


def test_nothing_new_in_robot_periodic():
    tree = ast.parse((pathlib.Path(__file__).parent.parent / 'robot.py').read_text(encoding='utf-8'))
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'robotPeriodic')
    used = {n.attr for n in ast.walk(fn)
            if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name) and n.value.id == 'self'}
    extra = used - ALLOWED
    assert not extra, (f"robotPeriodic() now uses self.{sorted(extra)}.  robot.py is closed to new periodic work: "
                       f"put it in the owning subsystem's periodic() with a rate (CLAUDE.md, House rules).")
