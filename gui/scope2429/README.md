# Scope2429: robot logs, live NetworkTables and match video in one dashboard

Scope2429 is team 2429's viewer for `.wpilog` robot logs, a live robot or simulator, and the match video. It is written for the 2026 season (REBUILT): the field, the dashboard replica and the NetworkTables topic names are 2026's. It is meant to live inside the robot repo (for example as `FRC2429_2026/scope2429/`) and to run in the same Python environment as the robot code and the dashboard GUI.

## Install and run

**If you already run the 2429 dashboard GUI (`gui/main.py`) you have everything you need:** Scope2429 uses only `numpy`, `opencv-python` and `pyntcore` (robotpy), plus Tkinter, which comes with Python. It does not use PyQt6. From this folder:

```
python scope2429.py
```

On a fresh machine: `pip install -r requirements.txt` (Python 3.9 or newer, 3.12 is tested; use `python3` / `pip3` on a Mac, and a virtual environment if you like). This also installs `yt-dlp` and `imageio-ffmpeg`, which the match-video features need (a log renders and plays fine without a video, but the app expects them to be there).

`yt-dlp` downloads the video from YouTube and also needs a JavaScript runtime: install Deno (`winget install DenoLand.Deno` on Windows, `brew install deno` on a Mac) and keep yt-dlp current with `pip install -U "yt-dlp[default]"`. `imageio-ffmpeg` is a bundled ffmpeg; skip it if `ffmpeg` is already on your PATH. You can also download the video yourself and use **Local file…**.

If the live connection says `pyntcore` is missing, it is missing from the Python you launched with (an IDE may use a different one than your terminal). The message shows that Python's path and the exact `pip` command to fix it.

The settings, the downloaded-video cache and the field images live in `~/.scope2429` (earlier versions used `~/.logscope` and `~/.frc_match_video`; the folder is renamed on first start). Nothing is written inside the repo.

## What it shows

Scope2429 shows the robot in **one fixed dashboard layout** (described below) from three kinds of source:

| Source | What you do | Match video | FMS info (match clock, scoring) |
|---|---|---|---|
| **Live robot / simulator** | Choose *Live robot / simulator*, type the address (`10.24.29.2` for the roboRIO, `localhost` for a simulator, a host name, `host:port`, or a team number) and press **Connect**. | none, trend plots instead | clock `--` until the Driver Station enables the robot, score 0 |
| **Log file (shop practice, no match)** | Choose *Log files*, open any `.wpilog`. | none, trend plots instead | clock `--`, score 0 |
| **Match log + video** | As before: log, then TBA / YouTube / local video, sync, render. | match video | from the Driver Station / FMS |

The video is always optional. The layout does not change between sources, and a panel whose data the source does not have is left blank in place.

## Live view

* Needs `pyntcore` (in `requirements.txt`). It is a NetworkTables 4 client, so use it against a roboRIO, a WPILib simulator (`localhost`) or any NT4 server. Reconnecting after a robot reboot is automatic.
* Everything the dashboard shows is read from the same NetworkTables topics as a log (`/SmartDashboard/...`, `/QuestNav/...`, `/DriverStation/...`). Plots scroll over the last 60 s. History is kept in memory only; use **Forget history** to clear it. The robot's own logging is unaffected.
* **Driver controller / button box panels** read `DS:joystick*`, which is Driver Station data and is not on NetworkTables, so they show *not connected* live. Replaying a log shows them.
* **Match clock** starts once the Driver Station puts the robot in autonomous (practice mode works too). Until then it shows `--`. **Points** come only from the match video; without one the score is 0 and accuracy is `--`. Shots fired (from flywheel dips) is still counted.
* **Pause display** freezes the picture (data keeps being recorded). **Save frame as PNG…** saves what you see.

## Logs without a match or a video

* A log with no autonomous enable (a shop session) is plotted from the first time the robot was enabled, or from the start of the log if it never was. The whole log is the clip. The match clock tile shows `--` and the scoring panel shows 0 points (scoring needs the match video).
* A log with no video is shown with **trend plots** in the top-right slot. By default these are shooter rpm / target, intake-hopper-indexer rpm, mean wheel speed and the Quest-vs-drive error. Use **Trend plots** in the GUI to plot any numeric entry instead (up to 6; an array entry gets one line per element).
* **Render MP4** works without a video: it renders the same 1920×1080 layout with the trend plots in the slot. **No video** clears a video you loaded.

## Dashboard layout

The **Deviation indicators** box in the GUI sets the three thresholds: motor rpm (default ±500), swerve speed (±0.3 m/s) and swerve angle (±10°). They apply to the preview, live view and rendered video.

Scope2429 makes a 1920×1080 picture (live, in the preview) or MP4. The top row is a replica of the team's own dashboard on the left and the match video (or the trend plots) on the right. The bottom half is a set of telemetry panels, all driven by what the robot logged to NetworkTables and the Driver Station:

**Top left: the 2429 dashboard window.** It is drawn from the real `gui/layout_2026.ui` (same wording, same relative positions and same indicator colors, but in a dark theme to match the rest of the frame), scaled to fit. Only data that is in the log is drawn. Anything else is left blank in place: the camera view, buttons, NT tree and autonomous chooser, plus indicators whose topics are not in the log (CAM 2/3, HSV, PHOTON TAGS, CAN REPORT, REFLASH, GYRO RESET, GAMEPIECE and so on). The match clock is derived from the Driver Station control word, because `match_time` is not logged.

  Because so much of the window is blank in a log replay, the layout is tightened so the field can be big. The Field tab is enlarged to the full width of the left column. The match clock, FMS Connected and Alliance tiles, the Commands and Status group and the PDH / Quest Bat monitors move into the strip under the field, and the hub indicator and Delay box move in under the right-hand groups. Everything else keeps its relative place. The window is scaled up to the width that is left (see `OVERRIDES` in `dash_left.py`).
- The Field tab has the robot icon (`drive_pose`) and the Quest icon with its green border (`quest_pose`). The field is rotated so the **red alliance is on the left**, the same as the YouTube broadcast.
- The Competition tab has the Scoring components (intake / hopper / indexer), Odometry and Cameras (ARDU RIGHT / ARDU BACK target tiles, SHOT DIST, Double Taps), Shooter (shooter RPM, navX heading, ROBOT / QUEST / DELTA pose) and Commands and Status groups.
- Under the tabs are the Cameras counters, the QuestNav indicators (ALIVE, VALID, TRACK, DTAP, ODOMETRY ENABLED, SYNCED, RESET), the active-hub indicator, the auto Delay, and the PDH Voltage / PDH Current / Quest Bat monitors.
- The middle column has the match clock (flashes under 30 s), FMS Connected and Alliance.

**Bottom: telemetry panels**
- **2429 scoring (estimate):** points, shots fired, accuracy and the live Quest-vs-drive odometry error. Shots *fired* come from flywheel-rpm dips while the indexer runs (balls fed from outside the alliance zone are passes under G407 and are left out: they cannot score, and the points that follow them are the partners'). Points *scored* come only from the match video, with no assumed hit rates, and are 0 without it. `scorebug.py` reads the broadcast's own alliance fuel counter ("71 / 240" in the corner box), which is exact but counts the whole alliance. Nothing is counted from the picture: an earlier version also counted yellow fuel flying over the hub, but that depended on the camera angles, which change from one event to the next, and it was removed. Which corner holds which alliance's counter depends on the broadcast layout (red on the left at SoCal Showdown, blue on the left on the Daly Division), and `scorebug.py` reads that from the score bar. `VideoShotModel` in `telemetry.py` measures one delay from this match's data (flywheel dip -> counter tick), then credits 2429 with the counter ticks in the window that follows each of its shooting bursts, capped at the shots fired in that burst. Only bursts shot from inside 2429's own alliance zone count (G407: balls fed from outside it are passes and are left out), and ticks outside those windows are not counted. Partners shooting in the same window cannot be told apart by video, so this is an upper bound. The first render of a match spends about 1 min reading the counter; the result is cached in `~/.scope2429/cache`. If the counter cannot be read the score stays 0.
- **PDH power:** voltage and current plots with a cursor, plus gauges for volts, amps and kW. Under them a third plot shows the **CAN bus utilization** (percent busy) of the three SystemCore buses, one line each: `can_s0` the swerve drive Krakens, `can_s1` the swerve turn SparkFlexes, `can_s2` the shooter, intake and PDH. The robot reads them once a second (`comp_system_core/helpers/can_monitor.py`, since 2026-10-25) and logs them as `NT:/SmartDashboard/CAN/<bus>/...`; its top adapts to the data (5, 10, 25, 50 or 100 %), since a healthy bus is only a few percent; older logs show "not in this log".
- **Swerve modules:** actual and desired `module_states` as arrows around the chassis, with, for each module, an indicator for the speed deviation and one for the angle deviation (actual minus desired): green within the threshold, red above it (faster, or counter-clockwise of the desired), blue below it. The angle difference is wrapped to ±180° so 359° vs 1° is a 2° error, a desired state written as the same wheel velocity the other way round (speed negated, angle + 180°) is not an error, and the angle is ignored while the module is not moving. The module order is LF, RF, LB, RB. The diagram is turned by the navX yaw, lined up with the field heading and drawn from the driver's position (up = the way the driver looks, away from their own alliance wall).
- **Swerve motor temperature:** plots for the 8 drive and turn motors, plus a bar for each.
- **Mechanism 2D:** the `Mech Side View` Mechanism2d drawn from the logged tree, with rpm bars beside it.
- **Driver controller:** the PS5 DualSense on port 0 (buttons, sticks, triggers) and what each pressed button does.
- **Button box:** drawn like the real operator box (photo 2026-10-25): ten arcade buttons numbered 0 to 9 in the box's own grid, blue or red as built, with the white tape labels (ZEROINT, 147 deg, AFTERBURNER, 40A, QUESTOFF, QUEST ON, UNSYNC, STOW, SHOOT, INTAKE), and the three-position toggle that adjusts the shooting power (10 = -50, 11 = +50, middle = no adjustment). A pressed button lights up with a white ring; the toggle's lever moves to its position. The numbers are the Driver Station button indexes on port 2 (0 based). If the box is rewired or relabelled, edit `BBOX_LABELS`, `BBOX_RED` and `BBOX_XY` in `helpers/dash.py`.

The clip runs from 5 s before auto to 5 s after the end of the match. The output is H.264 + AAC (`yuv420p`, faststart). It plays on Windows, on macOS (QuickTime) and in Slack.

**Quest vs shot accuracy** (left half of the strip at the bottom of the top-left quadrant): a timeline of the Quest-vs-drive pose error with one bar per shooting burst for that burst's accuracy (magenta when QuestNav was unsynced, i.e. `questnav_synched` false for most of the burst). Accuracy per burst is the fuel counted in the video divided by shots fired, so the bars need the video. The error is a proxy: it is how far the Quest pose is from the fused drive pose, not how far either is from the truth.

**Motor rpm indicators** (right half of the same strip): one tile per motor (flywheel left / right, roller, hopper, indexer left / right, intake left / right) showing actual minus commanded rpm. The tile is green within the threshold, red when the motor runs more than the threshold over the commanded rpm (in the commanded direction) and blue when it is more than the threshold under. A motor commanded off shows grey `off` (spinning down is not a fault), and so does one whose command just changed by more than the threshold in the last second (`...`, still spinning up). Commanded rpm comes from `Shooter/shooter_rpm`, `roller_rpm`, `hopper_rpm`, `indexer_rpm` and `Intake/intake_rpm`; actual from the `speed_*` entries of each motor.

The old 2-row layout (field + QuestNav status bar beside the video) is still available: set `RenderStyle(dashboard=False)`.

## The video window

The picture (preview, or the live view) is never inside the main window, so the main window holds only the controls and the **Match report…** / **Render MP4** buttons and the status bar always stay in view, even in a small window. The video window opens by itself with the first picture; drag it to another screen, resize it, or double-click / press F11 for full screen. Esc leaves full screen, then closes it, and **Show video window** brings it back (it will not reopen by itself after you close it). In log mode the arrow keys in that window scrub the log: left / right 1 s, with Shift 10 s, with Ctrl one frame. The **time slider** (seconds from match start / first enable, with Start / Auto end / Teleop / End) is at the bottom of the video window, and so are the **sync controls** (Auto-sync, the video offset and the ±1 / ±0.1 / ±0.033 nudges) - but only when the log has a TBA, YouTube or day video: without a video the offset means nothing, so the row is not shown. The main window no longer has either.

## Match report

**Match report…** (next to Render MP4, log mode) analyzes the selected log and opens a window with an Overview of every warning and one tab per area. Double-click a finding that has a time to jump the preview to that moment; The first tab is a **one-page picture** (1600 px wide PNG: the overall verdict, the problems and warnings that need attention in red and orange, and a colored tile for each headline number of health, swerve, odometry and cycles). **Save for Slack (PNG)…** saves it; drag the file into a Slack message and it shows inline. **Save HTML / text…** writes a self-contained, colorized HTML page (or plain text if you give it a `.txt` name; green / orange / red findings, tables, light and dark mode, no external files, easy to share in Slack or email), **Open in browser** shows that page right away, and **Copy as text** copies the plain text. From a script: `report.build_report(tel, ml).to_html()`. Everything is measured from entries in the log; a check is skipped, and the report says so, when its entry is not logged (logs from before 2026-10-25 have no CAN bus data; none of the robot's logs have loop times or Driver Station disconnects).

* **Health:** battery minimum, sag and time below 8 V / 7 V; roboRIO brownouts; peak and average current; per motor peak and average current with a warning when the left / right motors of a pair draw very different current; ArduCam frame dropouts.
* **Swerve modules:** per module speed and angle tracking error (RMS and % of time beyond the deviation thresholds, angle wrapped to ±180°, only while the module moves) and maximum drive / turn temperature. A module is flagged when it is clearly worse than the median of the other three.
* **QuestNav odometry:** time unsynced, error while synced (median, 95th percentile, maximum, time above 0.5 / 1 m), sudden jumps of the error while synced, tracking lost, double taps, reset commands, disconnects, Quest battery, and the error drift per minute over each synced stretch of 10 s or more.
* **Cycles:** shooting bursts from inside the alliance zone as cycles: first shot, balls per burst, shoot / collect / intake-on time, cycle time (median, best, worst per auto and teleop), slow cycles, and balls fed from outside the zone (G407 passes).

The **CAN buses** section lists each bus's mean and maximum utilization, the seconds over 70 %, and any bus-off or TX-queue-full event with its time (bus-off is red, and joins the "events that line up" check). The robot reads each bus with WPILib and, when that call fails (it does on the SystemCore), with Phoenix; the `source` topic says which, and it also logs Phoenix's own number for `can_s0` unchanged, which the section prints next to the mean. The unit is taken as percent and has not been checked against a bus analyzer (Phoenix documents 0-1 but returned about 2.45 on a bus), so use the plot for shape and spikes, not decimals; a log whose values never exceed 1.0 is shown x100.

## CTRE hoot files

A `.hoot` file is written on the robot by the CTRE (Phoenix 6) devices, one file per CAN bus. On the 2026 robot it holds the four swerve drive Krakens (CAN 21 LF, 23 LB, 25 RF, 27 RB): supply and stator current, motor and supply voltage, wheel velocity and position, device faults, and whether the CAN bus was up. The turn motors are REV and are not in it.

* **Conversion** uses CTRE's `owlet` tool, which AdvantageScope already downloads into its own folder (`%APPDATA%\AdvantageScope\owlet` on Windows). Scope2429 finds it there, in `~/.scope2429/bin`, or on your PATH, and picks the build that matches the hoot's log version (a mismatched owlet says "unsupported version"). If none is found it asks you to locate `owlet.exe`. The converted wpilog is cached in `~/.scope2429/cache/hoot`. A hoot that was not closed cleanly (power cut before CTRE logging stopped) still converts: everything readable is used and the report says so.
* **Finding the hoot:** in log mode the **CTRE hoot** box looks under the logs folder (3 levels deep) for a `.hoot` whose file or folder name contains the match (for example `CAEL_Q13`). **Find** repeats the search and **Browse…** picks files by hand; several files (one per CAN bus) are merged.
* **Clock:** the hoot and the robot log have different clocks, so they are lined up on the robot-enable edges (the same events in both). The report shows the offset and how many edges matched.
* **In the match report** a **CTRE hoot** section is added: CAN bus down time, device faults (brownout, boot during enable, rotor, temperature ...) with their times, devices that stopped reporting, per module peak and average supply current, peak stator current, lowest supply voltage, time at full voltage, time pushing against something (high current, wheel not turning), the wheel scale (module speed per motor revolution, which should match across modules; a difference means wheel wear or a wrong ratio) and how much of the battery current the drive motors took at the current peak and at the lowest battery voltage.
* **In the dashboard and the rendered video** (log mode) the hoot is loaded in the background when the log is selected (the box shows `[loaded, clock offset ...]`). The **swerve modules** panel then shows each module's drive supply current next to it (green to red), a red ring and the fault name (`BOOT`, `BROWNOUT`, `ROTOR`, ...) while the device reports a fault (held for 1.5 s, since a fault can be gone in 0.2 s), `NO DATA` when a device stops reporting, and a red banner by the panel title with what is wrong right now (including `CAN BUS DOWN`). The **PDH power** amps plot gets a second trace: the total swerve drive current from the hoot.
* **In the dashboard and the rendered video** (log mode) the hoot is loaded in the background when the log is selected (the box shows `[loaded, clock offset …]`). The **swerve modules** panel then shows each module's drive supply current next to it (green to red), a red ring and the fault name (`BOOT`, `BROWNOUT`, `ROTOR`, ...) while the device reports a fault (held for 1.5 s, since a fault can be gone in 0.2 s), `NO DATA` when a device stops reporting, and a red banner by the panel title with what is wrong right now (including `CAN BUS DOWN`). The **PDH power** amps plot gets a second trace: the total swerve drive current from the hoot.
* **Convert hoot to wpilog…** just converts files and saves the `.wpilog` next to each hoot, for AdvantageScope.

## Whole-day event videos (matches with no video of their own)

Not every match has its own YouTube video, but the event streams the whole day (SoCal Showdown: one 10 hour video per day). In log mode the **Whole-day event videos** box finds those videos and cuts every match 2429 played out of them:

1. **Find on YouTube** searches (edit the search text if you like; it keeps videos longer than 3 hours), select the Day 1 / Day 2 videos and press **Use selected**. They are remembered.
2. **Clip this match** cuts the selected log's match out of them and syncs it (the result is used like any video: preview, sync buttons, Render MP4). **Batch: all matches (auto)** uses a match's own TBA video if there is one, and otherwise cuts it out of the day videos; practice matches that happened before the stream started are reported as not covered.

How it works, so you know what to trust:

* **Where the match is.** The robot log records the computer's clock (`systemTime`), and yt-dlp gives the time the live stream started, so the match is at (auto start - stream start) seconds into the video. That is only a rough guess: the roboRIO clock and the stream start are off by a few minutes (by 3.5 minutes on day 1), but by the same amount for every match of a day. So each day video is first **calibrated**: five matches are searched +-7 minutes and the clock error that most of them agree on wins (this also rules out the look-alike neighbouring matches). Then each match is searched only +-90 s.
* **Fast download.** Only the stretch of the video that is needed is fetched (about 15 minutes for a first search, 5 minutes with the calibration), from YouTube's HLS version of the stream, which downloads at 15x real time or more instead of yt-dlp's real-time section download. Three matches are fetched at a time and the clips are cached in `~/.scope2429/cache/clips`.
* **Finding the exact start, and which match it is.** The match clock in the picture is the reference: it stands at 0:20 before auto starts and shows 0:19 exactly one second after it does. The key frames of the clip (one per 5 s) are scanned for 0:20, and each start is then found frame by frame. A day video is full of look-alike matches (every pair of neighbours is the same distance apart, so a wrong match looks as consistent as the right one: this is how Q13 was once cut as Q14), so in a qualification the white text "2429" must also be in the scoreboard at that moment; the neighbouring matches do not have it. Playoff graphics have no team numbers there, so for playoffs the day's clock error from the qualifications decides. The sound is only a fallback (a day stream's sound runs up to about 6 s off its picture; the picture is what the dashboard sits next to, so the picture is synced and the stream's sound is left as it is).
* **Disk space.** A clip is about 150-200 MB, cached in `~/.scope2429/cache/clips`; delete files there when you no longer need them (they are downloaded again on demand).
* **Clip this match on its own** first learns the day's clock error from the other logs in the folder (a few minutes, once per day video), because a day video is full of look-alike matches: without it Q13 was matched to Q14, which starts 7 minutes later.
* **When it fails** the batch report says why: no day video covers the match, or no start passed the check (check the sync by hand with the preview).
* **YouTube needs Deno** for the audio (see Install). Deno is found on PATH, in `~/.scope2429/bin` or `~/.deno/bin`, or at the path in the `DENO_PATH` environment variable.

## Make one video (match log + match video)

1. **Logs folder.** Pick the folder that holds the `.wpilog` files (or use **Open log file…**). Then click a log in the list.
2. **TBA.** The event key `2026cass` (SoCal Showdown 2026) is already filled in. The tool looks up the match on The Blue Alliance by itself; Qn becomes `qm n` and En becomes `sf n m1` (`f1m…` after 13). A free TBA Read API key, from your TBA account page, makes this step more reliable. Without a key the tool reads the TBA match web page instead. You can also paste any YouTube URL, or use **Local file…**.
3. **Download.** The video is saved once to `~/.scope2429/cache`. **Auto-sync** runs on its own after the download.
4. **Sync.** The *offset* is the time in the YouTube video when autonomous starts.
   * Auto-sync listens for the field sounds at auto start, auto end, teleop start and match end. It lines up their timing with the exact enable and disable times in the log.
   * To check the result, drag the preview slider or use **Start / Auto end / Teleop / End**. Then use the ±1 / ±0.1 / ±0.033 (one frame) buttons, or type an exact number.
5. **Render MP4.** By default the file goes to `<logs folder>/videos/<log name>_sidebyside.mp4` (`_dashboard.mp4` without a video).
   * Compact makes a 1280×720 file. Standard makes 1920×1080. Large is the 1920×1080 layout scaled up to 2560×1440. Expect tens of MB for a full match at Standard.

The offset, URL and title are saved for each log, so you can render a log again later without syncing it again.

**Batch: all matches (auto)** runs the whole chain unattended once you confirm the first dialog: for every log it finds the match video (TBA, or cut out of the whole-day event videos, which is also tried when the match's own video syncs weakly), downloads, auto-syncs and renders the MP4 (with the CTRE hoot if there is one), and saves the one-page match report as `<log name>_report.png` (for Slack) and `_report.html` next to the videos. Videos and reports that already exist are skipped, so a run can be repeated or resumed. Practice or shop logs without a match get a report but no video. At the end it lists every match whose sync confidence was low (also written to `videos/batch_summary.txt`) so you can check those by hand.

## Field image

The field list always offers these choices, best first:

1. **AdvantageScope's own 2026 field.** The tool finds it automatically in `%APPDATA%\AdvantageScope*\autoAssets\Field2d_*2026*`.
2. **2026 REBUILT (WPILib).** This is the official WPILib field drawing, bundled in `fields/`. It shows the hubs, bumps/ramps, trenches, towers, depots, outposts and the starting fuel. It is the default when AdvantageScope's field isn't found.
3. **(generic drawn field).** A clean, labelled drawing of the same elements without the fuel. The positions come from the WPILib 2026 AprilTag layout and the game manual.

You can also click **Download 2026 field** to fetch AdvantageScope's field from GitHub, or **Browse config.json…** to use any AdvantageScope `Field2d_*` folder.

Poses are read as WPILib blue-origin coordinates and mapped onto the image the same way AdvantageScope does it. The view is then rotated, never mirrored, so red is on the left and the blue-origin corner is at the top right. That is what a camera sees from the side of the field where red is on the left. You can turn this off with the "Red alliance on left" checkbox. When a match video is loaded the box sets itself to what the broadcast shows (the left score box of the score bar is the alliance on the left of the field camera: red at SoCal Showdown, blue on the Daly Division at Champs), and the batch render does the same per video. If the video has no readable score bar the box keeps its previous setting. A change you make by hand sticks until the next video is loaded.

## Files

* Next to `scope2429.py` (the entry points): `scope2429.py` is the Tkinter app (run this), `report.py` / `report_window.py` / `report_image.py` and `season_report.py` make the reports, `sysid.py` is the drive feedforward fit. The data folders `assets/`, `fields/` and `report_templates/` are here too.
* Everything else is a library module in `helpers/`. The entry points add `helpers/` to the import path, so the modules still import each other by plain name (`import core`). From your own script do the same: `sys.path.insert(0, "gui/scope2429/helpers")`.
* `helpers/live.py`: the live NetworkTables 4 connection (`LiveSession`), which fills the same `Telemetry` a log does.
* `helpers/trends.py`: the scrolling trend plots that take the video's place when there is no video (`TrendPanel`).
* `helpers/core.py`: wpilog reader (pure Python), field renderer, TBA lookup, yt-dlp download, audio auto-sync, ffmpeg compose. You can also use it from scripts.
* `helpers/console_file.py`: reads the robot console log (`console_<boot UTC>.log`, saved by the robot next to the `.wpilog`: everything it printed, including native warnings, stamped with the wpilog clock) and finds the one from the same boot as a log. The match report's **Robot console** section lists its warnings and errors at their match time.
* `hoot_rates.py`: how often each swerve drive Kraken records each signal in a `.hoot` (`python hoot_rates.py D:\logs`). MotorVoltage should be about 100 Hz for the SysId fit; 4 Hz means the robot code from before `k_drive_hoot_rate_hz` was deployed.
* `helpers/hoot.py`: CTRE hoot conversion with owlet, matching a hoot to a log, clock alignment (`HootLog`).
* `helpers/webcast.py`: whole-day event videos: find them, locate a match in them, fast section download, calibration, sync (`clip_match`, `clip_matches`).
* `report.py` / `report_window.py` / `report_image.py`: the match report (`build_report`, `to_html`, `to_text`), the window that shows it, and the one-page PNG (`render_image`).
* `helpers/telemetry.py`: decodes every logged or live signal the panels need (`Telemetry`) and the shot / points estimates (`ShotModel`, `VideoShotModel`).
* `helpers/balls.py`: one pass over the match video that reads the alliance fuel counter (via `scorebug.py`) and caches it. It no longer counts fuel in the picture.
* `helpers/scorebug.py`: reads the broadcast fuel counter digits (template matching against 11 glyph templates embedded in the file; `make_templates` rebuilds them) and works out which corner is which alliance from the score bar (`red_left`, `detect_red_left`).
* `helpers/dash.py`: draws the bottom panels and composes the full frame (`DashRenderer`).
* `helpers/dash_left.py`: draws the top-left dashboard window (`GuiPanel`) from `gui_layout.py`.
* `helpers/gui_layout.py`: the widget rectangles generated from the team dashboard's `layout_2026.ui` (regenerate it if that layout changes).
* `assets/`: the robot and Quest icons copied from the team dashboard.

## Season report (offline)

`python season_report.py <log folder> --names CAEL=CASS` reads every match log (and CTRE hoot files next to them, if owlet is installed) and writes `Season_Report.html` and `Compare_Robots.html` next to the logs. It needs no network (the pages fall back to system fonts when offline). `--new-event` picks the event treated as the new robot (default: the latest), `--out` the output folder, `--notes` your own resolved / remaining text (default `report_templates/season_notes.html`).
