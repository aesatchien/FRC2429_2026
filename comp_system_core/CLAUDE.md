# CLAUDE.md - comp_system_core (SystemCore, robotpy 2027.0.0a7)

**The rules for this whole repo are in [`../CLAUDE.md`](../CLAUDE.md): the House rules (where things
go, the loop budget, the dashboard patterns, small changes) and the environment notes. Claude Code
loads that file automatically from the parent folder. Any other assistant or person: read it
before changing anything here. This file only adds what is specific to this folder.**

- Environment: `robotpy==2027.0.0a7` with the exact pins in `pyproject.toml`. Run
  `python -m robotpy test`, never bare `pytest`.
- snake_case API everywhere. Read `docs/a7_migration.md` and the header of `pyproject.toml` before
  a non-trivial change.
- This is what the competition robot runs. A change that also applies to the roboRIO goes into
  `../comp_bot` too, ported by hand (camelCase), in the same commit or said in the message.
- `robot.py`'s `robot_periodic` is closed (`tests/test_robot_periodic_is_closed.py`). New work
  belongs in a subsystem's `periodic()` behind `self.counter % N`.
