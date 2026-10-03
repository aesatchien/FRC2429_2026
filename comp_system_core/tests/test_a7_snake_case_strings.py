"""
a7 renamed WPILib / REV / ntcore to snake_case.  A method name inside a STRING was not
renamed with it, and nothing complains:

    raw_data = value.get_raw() if hasattr(value, 'getRaw') else bytes()

hasattr() swallows the miss, so the guard was simply always False and QuestNav threw away
every frame on a7 - no error anywhere, the Quest just never fed a pose.  The landmine test
cannot see this (it checks attribute READS, not strings), and neither can flake8.

So: no camelCase name may appear as the attribute string of hasattr/getattr/setattr in this
project.  If one is ever genuinely needed (a vendor library still on camelCase), add it to
ALLOWED with the reason.
"""

import ast
import pathlib
import re

PROJECT = pathlib.Path(__file__).resolve().parent.parent
SKIP_DIRS = {'__pycache__', 'deprecated', 'tests'}
CAMEL = re.compile(r'^[a-z]+[A-Z]')
ALLOWED: set[tuple[str, str]] = set()   # {(relative path, 'camelName')}


def camel_case_attr_strings(source: str):
    for node in ast.walk(ast.parse(source)):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id in ('hasattr', 'getattr', 'setattr') and len(node.args) >= 2
                and isinstance(node.args[1], ast.Constant) and isinstance(node.args[1].value, str)
                and CAMEL.match(node.args[1].value)):
            yield node.lineno, node.func.id, node.args[1].value


def test_no_camel_case_names_in_attribute_strings():
    hits = []
    for path in sorted(PROJECT.rglob('*.py')):
        rel = path.relative_to(PROJECT)
        if SKIP_DIRS & set(rel.parts):
            continue
        for line, func, name in camel_case_attr_strings(path.read_text(encoding='utf-8')):
            if (rel.as_posix(), name) not in ALLOWED:
                hits.append(f"{rel.as_posix()}:{line}  {func}(..., {name!r})")
    assert not hits, "camelCase name in an attribute string - a7 is snake_case:\n  " + "\n  ".join(hits)


def test_it_catches_the_questnav_bug():
    bad = "raw = value.get_raw() if hasattr(value, 'getRaw') else bytes()\n"
    assert [name for _, _, name in camel_case_attr_strings(bad)] == ['getRaw']
