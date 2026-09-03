"""
Catch attribute landmines:  x.foo  where foo does not exist on x.

WHY THIS EXISTS
---------------
This is the exact failure that took the robot down: can_status.py held
module.turningSpark after the module started holding vendor adapters.  It was invisible
because can_status is only built inside RobotContainer.initialize_dashboard().

pyflakes cannot see it - it only finds undefined NAMES, not undefined ATTRIBUTES.
pylint's no-member cannot see it either, and I checked: almost everything in this codebase
reaches a subsystem through an untyped `container` parameter, so pylint has no type to
check against and stays silent.

This works the other way round.  It builds the real RobotContainer, asks each live object
what attributes it actually has, then walks every source file looking for reads that are
not in that set.  No type inference required.

LIMITATION - NAME SHADOWING
A bare name like `module` or `questnav` might not be the thing we think.  Two real examples
in this repo: Questnav.self.questnav is the QuestNav *helper library*, not the subsystem;
and Swerve._check_pathplanner_config's `module` is a PathPlanner ModuleConfig.  So a name is
skipped in any file that assigns to it, since that assignment means it is something local.
That trades a little coverage for near-zero false positives, which is the right way round
for a test that gates deploys.
"""

import ast
import pathlib

import pytest


SKIP_DIRS = {'__pycache__', 'deprecated', 'tests', 'generated', 'build', 'ctre_sim'}


@pytest.fixture(scope='module')
def live_objects(container):
    """name as written in source -> a real instance to interrogate.

    `container` is the session fixture from conftest.py; do NOT build a second
    RobotContainer here (see the note in conftest)."""
    module = container.swerve.swerve_modules[0]
    return {
        'container': container,
        'intake': container.intake,
        'shooter': container.shooter,
        'swerve': container.swerve,
        'targeting': container.targeting,
        'led': container.led,
        'vision': container.vision,
        'robot_state': container.robot_state,
        'questnav': container.questnav,
        'module': module,
        'swerve_module': module,
        'drive_motor': module.drive_motor,
        'turn_motor': module.turn_motor,
    }


def _owner_of(node) -> str | None:
    """Reduce self.container.intake / container.intake / self.intake -> 'intake'."""
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        if isinstance(node.value, ast.Name) and node.value.id in ('self', 'container'):
            return node.attr
        if isinstance(node.value, ast.Attribute):
            return node.attr
    return None


def _rebinds_to_something_else(value, name: str) -> bool:
    """Does this assignment make `name` refer to something OTHER than the object we mapped?

    `self.swerve = swerve` keeps the meaning - that is the pass-a-subsystem-in idiom used by
    nearly every command, and excluding it would gut the test's coverage.
    `self.questnav = Metaquestnav()` and `module = robot_config.moduleConfig` do not.
    """
    if isinstance(value, ast.Name) and value.id == name:
        return False                                  # self.swerve = swerve
    if isinstance(value, ast.Attribute) and value.attr == name:
        return False                                  # self.swerve = container.swerve
    # for module in <...>.swerve_modules  -> still a SwerveModule
    if isinstance(value, ast.Attribute) and value.attr == 'swerve_modules':
        return False
    if isinstance(value, ast.Subscript):
        inner = value.value
        if isinstance(inner, ast.Attribute) and inner.attr == 'swerve_modules':
            return False                              # swerve_modules[0]
    return True


def _shadowed_names(tree) -> set:
    """Names this file rebinds to something other than what we mapped them to."""
    shadowed = set()

    def consider(target, value):
        if isinstance(target, ast.Name):
            name = target.id
        elif isinstance(target, ast.Attribute):
            name = target.attr
        else:
            return
        if _rebinds_to_something_else(value, name):
            shadowed.add(name)

    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                consider(target, node.value)
        elif isinstance(node, (ast.AnnAssign, ast.AugAssign)) and node.value is not None:
            consider(node.target, node.value)
        elif isinstance(node, (ast.For, ast.comprehension)):
            consider(node.target, node.iter)
    return shadowed


def test_no_attribute_landmines(live_objects):
    known = {name: set(dir(obj)) for name, obj in live_objects.items()}
    root = pathlib.Path(__file__).resolve().parent.parent

    hits = []
    unparseable = []
    for path in sorted(root.rglob('*.py')):
        if SKIP_DIRS & set(path.parts):
            continue
        try:
            # utf-8-sig so a byte-order mark does not make the file unreadable.  Windows
            # tooling (PowerShell's Set-Content -Encoding utf8) writes one; Python's own
            # import machinery copes, but ast.parse on a plain utf-8 read does not.
            tree = ast.parse(path.read_text(encoding='utf-8-sig'))
        except SyntaxError as err:
            # NEVER silently skip.  A file this cannot parse is a file this is not checking,
            # and a test that quietly stops covering things is worse than no test.
            unparseable.append(f'{path.relative_to(root)}: {err}')
            continue

        shadowed = _shadowed_names(tree)
        for node in ast.walk(tree):
            if not isinstance(node, ast.Attribute) or isinstance(node.ctx, ast.Store):
                continue
            owner = _owner_of(node.value)
            if owner in known and owner not in shadowed and node.attr not in known[owner]:
                hits.append(f'{path.relative_to(root)}:{node.lineno}  {owner}.{node.attr}')

    assert not unparseable, (
        'these files could not be parsed, so they were NOT checked:\n  ' +
        '\n  '.join(unparseable))

    assert not hits, (
        'attribute(s) that do not exist on the real object:\n  ' + '\n  '.join(hits) +
        '\n\nIf one of these is a false positive, the name is probably shadowed by a local '
        'variable - check LIMITATION in this file.')
