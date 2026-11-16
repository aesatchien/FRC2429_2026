# CLAUDE.md - gui (PyQt6 dashboard)

**The rules for this whole repo are in [`../CLAUDE.md`](../CLAUDE.md): the House rules (where things
go, the loop budget, the dashboard patterns, small changes) and the environment notes. Claude Code
loads that file automatically from the parent folder. Any other assistant or person: read it
before changing anything here. This file only adds what is specific to this folder.**

- Read `README.md` here for the architecture. The dashboard is config-driven: a widget is an entry
  in `config.py` `WIDGET_CONFIG` plus an `update_style` handled in `ui_updater.py`.
- To show or set something new, in this order: (1) a `WIDGET_CONFIG` entry using an existing
  `update_style`; (2) a small option on an existing style (see `unset_value` on `combo`); (3) a new
  style in `ui_updater.py`. Hand-built widgets with their own timers, topics and popups in
  `dashboard_2026.py` are not accepted.
- No lists or topic paths that duplicate the robot's. The robot publishes its options; the topic
  prefixes at the top of `config.py` must match the robot's `constants.py`.
- No popups or modal dialogs for the drive team. State is shown in place (color, text).
- It talks to both robot versions (2026 `SendableChooser` and 2027a7 `Selectable`), so do not
  assume one.
- `scope2429/` is a separate tool with its own `CLAUDE.md`; the rules above still apply to it.
