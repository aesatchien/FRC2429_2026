# CLAUDE.md - comp_bot (roboRIO, robotpy 2026.2.2)

**The rules for this whole repo are in [`../CLAUDE.md`](../CLAUDE.md): the House rules (where things
go, the loop budget, the dashboard patterns, small changes) and the environment notes. Claude Code
loads that file automatically from the parent folder. Any other assistant or person: read it
before changing anything here. This file only adds what is specific to this folder.**

- Environment: `robotpy==2026.2.2`. Run `python -m robotpy test`, never bare `pytest`.
- camelCase 2026 API. One CAN bus, navX, 5 V analog: do not copy hardware assumptions from
  `../comp_system_core`.
- Maintained, not historical: every roboRIO robot the team has runs this. The competition robot
  does not, so never write "confirmed on the robot" for a change here unless it ran on a roboRIO.
- A change that also applies to the SystemCore goes into `../comp_system_core` too (snake_case).
- `robot.py`'s `robotPeriodic` is closed (`tests/test_robot_periodic_is_closed.py`).
