"""
No 2026 (camelCase) WPILib / commands2 / ntcore / REV names in this project.

WHY THIS EXISTS
---------------
a7 renamed everything to snake_case.  PR #13 (Oct 2026) bound a button with

    js.bbox_1_5.onTrue(InstantCommand(...).ignoringDisable(True))

which is right for comp_bot (roboRIO, 2026) and wrong here.  RobotContainer could not be
built, so the robot program would not have started.  test_robot_builds caught it, but the
failure was an AttributeError deep in a traceback.  This test names the mistake directly:
"onTrue is the 2026 spelling - use on_true".

HOW IT DECIDES
--------------
It does not use a hand-written list (that would go stale).  It reads every name the
installed a7 libraries actually expose, works out the old camelCase spelling of each
snake_case one (on_true -> onTrue), and flags any `x.onTrue`-style use of those old
spellings in this project's code.  CamelCase names the PROJECT defines itself
(setDesiredState, PathPlanner's registerCommand, ...) are allowed.  Comments are not code,
so they are not checked - but keep commented-out lines in a7 spelling too, so they work
when someone uncomments them.
"""

import ast
import importlib
import inspect
import pathlib

PROJECT = pathlib.Path(__file__).resolve().parent.parent
SKIP_DIRS = {'__pycache__', 'deprecated', 'tests'}
A7_MODULES = ['wpilib', 'wpilib.simulation', 'commands2', 'commands2.button', 'commands2.cmd',
              'ntcore', 'wpimath', 'rev', 'hal']


def _camel(snake: str) -> str:
    head, *rest = snake.split('_')
    return head + ''.join(part[:1].upper() + part[1:] for part in rest)


def _library_names() -> set[str]:
    names = set()
    for modname in A7_MODULES:
        mod = importlib.import_module(modname)
        for name, obj in vars(mod).items():
            names.add(name)
            if inspect.isclass(obj):
                names.update(dir(obj))
    return names


def _old_spellings(lib_names: set[str]) -> dict[str, str]:
    """{camelCase: snake_case} for every a7 snake_case name whose 2026 spelling differs.
    A camelCase name that ALSO exists in the library (phoenix6_compat aliases a few back
    on for phoenix6's sake) is still wrong for our own code, so it stays in the map."""
    old = {}
    for name in lib_names:
        if '_' in name.strip('_') and name == name.lower():
            camel = _camel(name.strip('_'))
            if camel != name:
                old[camel] = name
    return old


def _source_files():
    for path in sorted(PROJECT.rglob('*.py')):
        rel = path.relative_to(PROJECT)
        if not SKIP_DIRS & set(rel.parts):
            yield rel, ast.parse(path.read_text(encoding='utf-8'))


def _project_defined_names(trees) -> set[str]:
    defined = set()
    for _, tree in trees:
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                defined.add(node.name)
            elif isinstance(node, ast.Attribute) and isinstance(node.ctx, ast.Store):
                defined.add(node.attr)
            elif isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
                defined.add(node.id)
    return defined


def find_old_spellings(trees, old: dict[str, str], allowed: set[str]) -> list[str]:
    hits = []
    for rel, tree in trees:
        for node in ast.walk(tree):
            if (isinstance(node, ast.Attribute) and isinstance(node.ctx, ast.Load)
                    and node.attr in old and node.attr not in allowed):
                hits.append(f"{rel.as_posix()}:{node.lineno}  .{node.attr} is the 2026 spelling"
                            f" - use .{old[node.attr]}")
    return hits


def test_no_2026_camelcase_names():
    trees = list(_source_files())
    old = _old_spellings(_library_names())
    hits = find_old_spellings(trees, old, _project_defined_names(trees))
    assert not hits, ("comp_system_core is WPILib 2027a7 (snake_case).  These are 2026 names "
                      "(that is comp_bot's spelling):\n  " + "\n  ".join(hits))


def test_it_catches_the_pr13_mistake():
    bad = ast.parse("js.bbox_1_5.onTrue(InstantCommand(lambda: q.sync()).ignoringDisable(True))\n")
    old = _old_spellings(_library_names())
    hits = find_old_spellings([(pathlib.Path('robotcontainer.py'), bad)], old, allowed=set())
    assert any('.onTrue' in h and '.on_true' in h for h in hits), hits
    assert any('.ignoringDisable' in h and '.ignoring_disable' in h for h in hits), hits
