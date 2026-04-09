# QuestNav Double-Tap Passthrough Fix — Java Implementation Guide

This document breaks down the Python implementation of the Quest double-tap / passthrough
workaround into components that can be ported to a standard Java FRC robot project, plus
a companion PowerShell script that replaces the PyQt dashboard's ADB recovery logic.

---

## Background: What Is the "Passthrough" Problem?

The Meta Quest headset has a system-level gesture (double-tap the side) that yanks the
foreground application into "passthrough" mode — a feature for user safety so they can
see the real world. When this happens mid-match:

1. The `QuestNav` Unity app is suspended.
2. NetworkTables frame delivery from the headset stops immediately.
3. `is_tracking()` goes `False` and `get_all_unread_pose_frames()` returns empty.
4. The robot's pose estimator loses its best absolute-position sensor.

The fix is to detect the blackout quickly, signal the driver-station PC, and use `adb`
to bring the QuestNav app back to the foreground.

---

## Part 1 — Robot-Side Detection (Java Port of `quest.py`)

### 1.1  State Variables to Declare

```java
// In your QuestNav subsystem constructor / field declarations:

private int missedFrameCount = 0;
private static final int K_MAX_MISSED_FRAMES = 14;   // 14 × 20ms = 280ms at 50 Hz
                                                      // (14 in Python at 20ms/loop ≈ same)
private boolean wasTracking = false;
private boolean wasConnected = false;
private int disconnectedCount = 0;
private static final int K_MAX_DISCONNECTED_COUNT = 50; // tune to match qc.k_max_disconnected_count

private int dtapCount = 0;
private double passthruStartTime = 0.0;
private boolean syncedBeforePassthru = false;
private static final double K_MAX_RESYNC_TIME = 3.0;  // seconds — match QuestConstants.k_max_resync_time
```

### 1.2  NetworkTables Entries to Publish

```java
// Declare at class level (use the same NT prefix the GUI/dashboard expects)
private BooleanEntry questPassthroughEntry;      // /QuestNav/quest_in_passthrough
private DoublePublisher questDtapCountPub;       // /QuestNav/quest_dtap_count
private BooleanPublisher questConnectedPub;      // /QuestNav/quest_connected
private BooleanPublisher questTrackingPub;       // /QuestNav/quest_tracking
private BooleanPublisher questAcceptedPub;       // /QuestNav/quest_pose_accepted
private BooleanPublisher questSynchedPub;        // /QuestNav/questnav_synched

// In your init / constructor:
NetworkTableInstance inst = NetworkTableInstance.getDefault();
String questPrefix = "/QuestNav";

questPassthroughEntry = inst.getBooleanTopic(questPrefix + "/quest_in_passthrough").getEntry(false);
questDtapCountPub     = inst.getDoubleTopic(questPrefix  + "/quest_dtap_count").publish();
questConnectedPub     = inst.getBooleanTopic(questPrefix + "/quest_connected").publish();
questTrackingPub      = inst.getBooleanTopic(questPrefix + "/quest_tracking").publish();
questAcceptedPub      = inst.getBooleanTopic(questPrefix + "/quest_pose_accepted").publish();
questSynchedPub       = inst.getBooleanTopic(questPrefix + "/questnav_synched").publish();

// Seed the passthrough flag to False on boot
questPassthroughEntry.set(false);
questDtapCountPub.set(0);
```

### 1.3  The Periodic Detection Loop

This is the core of `quest.py periodic()` — the watchdog that decides whether the
headset has been double-tapped into passthrough.

```java
@Override
public void periodic() {
    boolean isConnected = questNav.isConnected();   // NT heartbeat < 200 ms
    boolean isTracking  = questNav.isTracking();    // SLAM algorithms confident

    // --- Disconnection watchdog (hard unsync after prolonged loss) ---
    if (!isConnected) {
        disconnectedCount++;
        if (disconnectedCount > K_MAX_DISCONNECTED_COUNT && wasConnected) {
            System.out.printf("*** QuestNav connection dropped for %.2fs%n",
                              disconnectedCount / 50.0);
            questUnsyncOdometry();   // wipe synched flag so we don't trust stale data
            wasConnected = false;
        }
    } else {
        disconnectedCount = 0;
        wasConnected = true;
    }

    // --- Frame-level watchdog (faster than the NT heartbeat) ---
    var frames = questNav.getAllUnreadPoseFrames();
    boolean gotNewFrame = isTracking && frames != null && !frames.isEmpty();

    if (gotNewFrame) {
        // ---- Successful frame: reset watchdog, process pose ----
        missedFrameCount = 0;

        var lastFrame = frames.get(frames.size() - 1);
        try {
            // Transform the 3-D Quest pose to a 2-D robot pose
            Pose2d newPose = lastFrame.getQuestPose3d().toPose2d().transformBy(questToRobot);
            questPose = newPose;
            questPoseTimestamp = lastFrame.getDataTimestamp();

            if (!wasTracking) {
                // Tracking just restored
                System.out.printf("Quest tracking restored at %.2fs%n",
                                   Timer.getFPGATimestamp());
                questPassthroughEntry.set(false);   // clear the flag for the dashboard

                double timeInPassthru = Timer.getFPGATimestamp() - passthruStartTime;
                if (timeInPassthru < K_MAX_RESYNC_TIME
                        && syncedBeforePassthru
                        && !questHasSynched) {
                    // Short outage AND we were synced before: soft-resync, don't require manual re-sync
                    System.out.printf("Quest recovered in %.2fs — soft resyncing%n",
                                       timeInPassthru);
                    questSoftResync();
                }
            }
            wasTracking = true;

        } catch (Exception e) {
            System.out.println("Error processing Quest frame: " + e.getMessage());
            wasTracking = false;
        }

    } else {
        // ---- No new frame: increment missed-frame watchdog ----
        missedFrameCount++;

        if (missedFrameCount > K_MAX_MISSED_FRAMES && wasTracking) {
            // *** PASSTHROUGH DETECTED ***
            wasTracking = false;
            passthruStartTime = Timer.getFPGATimestamp();
            syncedBeforePassthru = questHasSynched;

            // Signal the driver-station watcher script
            questPassthroughEntry.set(true);

            dtapCount++;
            questDtapCountPub.set(dtapCount);

            System.out.printf("Detecting a lost QuestNav at FPGA timestamp: %.2fs%n",
                               Timer.getFPGATimestamp());
        }
    }

    // --- Pose acceptance gate ---
    // Two-layer protection:
    //   isConnected  → NT heartbeat, 200 ms window
    //   data_is_fresh → frame watchdog,  100 ms window  (catches passthrough 2× faster)
    boolean dataIsFresh = (missedFrameCount <= 5);   // 5 × 20ms = 100ms
    boolean inBounds    = inFieldBounds(questPose);

    questPoseAccepted = inBounds && isConnected && isTracking && wasTracking && dataIsFresh;

    // Publish telemetry (rate-limited to every 5 loops if desired)
    questConnectedPub.set(isConnected);
    questTrackingPub.set(isTracking);
    questAcceptedPub.set(questPoseAccepted);
}
```

### 1.4  Why Two Layers of Protection?

| Layer | Variable | Timeout | Purpose |
|---|---|---|---|
| NT heartbeat | `isConnected` | ~200 ms | Detects full Wi-Fi / USB loss |
| Frame watchdog | `dataIsFresh` | 100 ms | Catches passthrough **2× faster** — the Quest app suspends before the NT heartbeat can expire |

During a double-tap event, `isConnected` stays `True` for up to 200 ms because the
NT connection is still technically live. `dataIsFresh` fails at frame 6 (~100 ms),
killing odometry updates well before the estimator can drift noticeably.

### 1.5  Soft-Resync vs Hard-Unsync

| Condition | Action | Why |
|---|---|---|
| `timeInPassthru < K_MAX_RESYNC_TIME` AND was synced | `questSoftResync()` — mark synched=true, don't move the pose | Brief outage: Quest SLAM is likely still accurate |
| `timeInPassthru >= K_MAX_RESYNC_TIME` | Leave `questHasSynched = false` | Long outage: Quest may have re-initialized at a different origin |
| `disconnectedCount > K_MAX_DISCONNECTED_COUNT` | `questUnsyncOdometry()` — full hard unsync | NT totally gone: require manual re-sync |

---

## Part 2 — Dashboard-Side Recovery (Python ui_updater equivalent)

### 2.1  What the PyQt Dashboard Does

`ui_updater.py` runs `_check_questnav_passthrough_issue()` every 50 ms (one GUI timer tick).

**Logic flow:**

```
Read /QuestNav/quest_in_passthrough  (boolean)
│
├── True?
│   ├── Record dtap_start_time (first True only)
│   └── If elapsed > QUESTNAV_PASSTHRU_ADB_DELAY (0.05 s)
│       └── If cooldown elapsed (QUESTNAV_ADB_COOLDOWN_S = 2.0 s)
│           └── If retries < QUESTNAV_ADB_MAX_RETRIES (5)
│               └── Fire:  adb.exe -s <ip>:5802 shell am start
│                               -n gg.QuestNav.QuestNav/com.unity3d.player.UnityPlayerGameActivity
│                   Increment retry counter
│                   Reset dtap_start_time (requires another full delay before next shot)
│
└── False?
    └── Reset dtap_start_time = 0, retries = 0
```

Key config values from `config.py`:

```python
QUESTNAV_ADB_ADDRESS        = '10.24.29.200:5802'  # Quest Wi-Fi ADB address
QUESTNAV_PASSTHRU_ADB_DELAY = 0.05   # seconds before first ADB attempt
QUESTNAV_ADB_COOLDOWN_S     = 2.0    # minimum seconds between retries
QUESTNAV_ADB_MAX_RETRIES    = 5      # give up after this many attempts
```

### 2.2  The ADB Command (the actual fix)

```bat
adb.exe -s 10.24.29.200:5802 shell am start -n gg.QuestNav.QuestNav/com.unity3d.player.UnityPlayerGameActivity
```

This uses Android Debug Bridge (`adb`) over Wi-Fi (port 5802, the ADB-over-TCP default
the Quest uses). The `am start` command asks Android to bring the QuestNav activity back
to the foreground, exactly reversing the effect of the double-tap gesture.

The test / simulation command (brings up the Home screen to *cause* a passthrough):
```bat
adb.exe -s 10.24.29.200:5802 shell am start -a android.intent.action.MAIN -c android.intent.category.HOME
```

---

## Part 3 — Replacement: PowerShell NT-Watcher Script

This script replaces the PyQt `_check_questnav_passthrough_issue()` function.  
It can be run on the driver station PC independently of any dashboard.

### 3.1  Prerequisites

- [NT4 / pynetworktables or ntcore Python bindings] OR use raw NT4 WebSocket, OR
  use the **`nt4client` NuGet** (C#), OR just call `nt4.ps1` if available.
- Simplest approach on Windows: install **Python** + `robotpy-ntcore` and run a
  `.py` watcher, or use the PowerShell script below that shells out to a tiny Python
  one-liner for the NT read.
- `adb.exe` must be on `PATH` or at a known path (place it next to the script).

### 3.2  PowerShell Script: `quest_watcher.ps1`

```powershell
<#
.SYNOPSIS
    Monitors the /QuestNav/quest_in_passthrough NetworkTables boolean.
    When True, fires ADB to bring the QuestNav app back to the foreground.

.NOTES
    Requires Python with robotpy-ntcore installed.
    Place adb.exe in the same directory as this script, or ensure it is on PATH.
    Run on the driver-station PC.  Start before each match.

    Usage:  .\quest_watcher.ps1 [-RobotIP 10.24.29.2] [-QuestADB 10.24.29.200:5802]
#>

param(
    [string]$RobotIP   = "10.24.29.2",
    [string]$QuestADB  = "10.24.29.200:5802",
    [double]$AdbDelay  = 0.05,   # seconds after detection before first ADB attempt
    [double]$Cooldown  = 2.0,    # minimum seconds between retries
    [int]   $MaxRetries = 5      # give up after this many attempts per event
)

$ScriptDir  = Split-Path -Parent $MyInvocation.MyCommand.Path
$AdbExe     = Join-Path $ScriptDir "adb.exe"
if (-not (Test-Path $AdbExe)) { $AdbExe = "adb" }   # fall back to PATH

# ---- Tiny Python helper that reads one NT boolean and exits with 0=False / 1=True ----
$PythonReader = @"
import sys, time
from ntcore import NetworkTableInstance
inst = NetworkTableInstance.getDefault()
inst.startClient4('quest_watcher')
inst.setServer('$RobotIP')
time.sleep(0.3)   # allow NT to connect
sub = inst.getBooleanTopic('/QuestNav/quest_in_passthrough').subscribe(False)
time.sleep(0.05)
val = sub.get()
inst.stopClient()
sys.exit(1 if val else 0)
"@

Write-Host "[$(Get-Date -f HH:mm:ss)] Quest Watcher started. Robot=$RobotIP  Quest=$QuestADB"
Write-Host "[$(Get-Date -f HH:mm:ss)] ADB executable: $AdbExe"

$dtapStartTime  = $null
$lastFixTime    = [DateTime]::MinValue
$retries        = 0
$wasInPassthru  = $false

while ($true) {
    # --- Read the NT boolean via Python subprocess ---
    $result = python -c $PythonReader 2>$null
    $isInPassthru = ($LASTEXITCODE -eq 1)

    $now = Get-Date

    if ($isInPassthru) {
        if (-not $wasInPassthru) {
            Write-Host "[$(Get-Date -f HH:mm:ss)] PASSTHROUGH DETECTED — starting ADB recovery timer."
            $dtapStartTime = $now
            $retries = 0
        }

        $elapsed = ($now - $dtapStartTime).TotalSeconds
        $sinceLastFix = ($now - $lastFixTime).TotalSeconds

        if ($elapsed -gt $AdbDelay -and $sinceLastFix -gt $Cooldown) {
            if ($retries -lt $MaxRetries) {
                $retries++
                Write-Host "[$(Get-Date -f HH:mm:ss)] Firing ADB fix attempt $retries / $MaxRetries ..."
                & $AdbExe -s $QuestADB shell am start `
                    -n "gg.QuestNav.QuestNav/com.unity3d.player.UnityPlayerGameActivity"
                $lastFixTime   = $now
                $dtapStartTime = $now   # require another full delay before next shot
            } elseif ($retries -eq $MaxRetries) {
                Write-Host "[$(Get-Date -f HH:mm:ss)] Max retries ($MaxRetries) reached. Giving up on this event."
                $retries++   # prevents repeated 'giving up' messages
            }
        }
        $wasInPassthru = $true

    } else {
        if ($wasInPassthru) {
            Write-Host "[$(Get-Date -f HH:mm:ss)] Passthrough resolved after $retries attempt(s)."
            $retries = 0
        }
        $dtapStartTime = $null
        $wasInPassthru = $false
    }

    Start-Sleep -Milliseconds 100   # poll at ~10 Hz (faster is fine, slower risks missing short events)
}
```

### 3.3  Limitations of the PowerShell Approach

The polling loop spawns a Python subprocess each iteration to read NT.  This is
slow (~300 ms per read) and adds latency relative to the PyQt dashboard which holds a
persistent NT connection.  A better alternative is a **standalone Python script** that
keeps the NT connection open:

### 3.4  Better Alternative: Standalone Python Watcher (`quest_watcher.py`)

```python
"""
quest_watcher.py — Monitors /QuestNav/quest_in_passthrough and fires ADB when True.
Equivalent to PyQt ui_updater._check_questnav_passthrough_issue() but standalone.

Usage:
    python quest_watcher.py [--robot 10.24.29.2] [--quest 10.24.29.200:5802]
"""
import argparse, os, subprocess, sys, time
from ntcore import NetworkTableInstance

# ---- Config (override via CLI args) ----
DEFAULT_ROBOT_IP  = "10.24.29.2"
DEFAULT_QUEST_ADB = "10.24.29.200:5802"
ADB_DELAY_S       = 0.05   # seconds after detection before first attempt
ADB_COOLDOWN_S    = 2.0    # minimum seconds between retries
ADB_MAX_RETRIES   = 5

def get_adb_path():
    here = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.join(here, "adb", "adb.exe")
    return candidate if os.path.exists(candidate) else "adb"

def fire_adb(adb_path, quest_adb):
    cmd = [adb_path, "-s", quest_adb, "shell", "am", "start",
           "-n", "gg.QuestNav.QuestNav/com.unity3d.player.UnityPlayerGameActivity"]
    subprocess.Popen(cmd)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default=DEFAULT_ROBOT_IP)
    ap.add_argument("--quest", default=DEFAULT_QUEST_ADB)
    args = ap.parse_args()

    adb_path = get_adb_path()
    print(f"[{time.time():.1f}] Quest Watcher started. Robot={args.robot}  Quest={args.quest}")
    print(f"[{time.time():.1f}] ADB: {adb_path}")

    inst = NetworkTableInstance.getDefault()
    inst.startClient4("quest_watcher")
    inst.setServer(args.robot)
    time.sleep(0.5)

    sub = inst.getBooleanTopic("/QuestNav/quest_in_passthrough").subscribe(False)

    dtap_start_time = 0.0
    last_fix_time   = 0.0
    retries         = 0
    was_in_passthru = False

    try:
        while True:
            is_in_passthru = sub.get()
            current_time   = time.time()

            if is_in_passthru:
                if not was_in_passthru:
                    print(f"[{current_time:.1f}] PASSTHROUGH DETECTED")
                    dtap_start_time = current_time
                    retries = 0

                elapsed       = current_time - dtap_start_time
                since_last    = current_time - last_fix_time

                if elapsed > ADB_DELAY_S and since_last > ADB_COOLDOWN_S:
                    if retries < ADB_MAX_RETRIES:
                        retries += 1
                        print(f"[{current_time:.1f}] ADB fix attempt {retries}/{ADB_MAX_RETRIES}")
                        fire_adb(adb_path, args.quest)
                        last_fix_time   = current_time
                        dtap_start_time = current_time
                    elif retries == ADB_MAX_RETRIES:
                        print(f"[{current_time:.1f}] Max retries reached — giving up.")
                        retries += 1
                was_in_passthru = True

            else:
                if was_in_passthru:
                    print(f"[{current_time:.1f}] Passthrough resolved after {retries} attempt(s).")
                    retries = 0
                dtap_start_time = 0.0
                was_in_passthru = False

            time.sleep(0.05)   # ~20 Hz poll — matches GUI timer tick

    except KeyboardInterrupt:
        print("Watcher stopped.")
    finally:
        inst.stopClient()

if __name__ == "__main__":
    main()
```

---

## Part 4 — Integration Notes for Java

### 4.1  Shuffleboard / SmartDashboard Visibility

The robot publishes these NT topics for the dashboard to display:

| NT Topic | Type | Meaning |
|---|---|---|
| `/QuestNav/quest_in_passthrough` | Boolean | `True` = passthrough detected |
| `/QuestNav/quest_dtap_count` | Double | cumulative double-tap event counter |
| `/QuestNav/quest_connected` | Boolean | NT heartbeat alive |
| `/QuestNav/quest_tracking` | Boolean | SLAM tracking confident |
| `/QuestNav/quest_pose_accepted` | Boolean | all acceptance criteria met |
| `/QuestNav/questnav_synched` | Boolean | quest has been synced to robot odometry |

In Shuffleboard you can add Boolean Box widgets for the first five.  The
`quest_dtap_count` is useful as a persistent counter — if it climbs during a match
it tells you the double-tap happened.

### 4.2  No Shuffleboard Plugin Needed

Shuffleboard cannot run shell commands natively.  **Do not try to build a plugin for
this.**  The correct split of responsibility is:

- **Robot code** → detects passthrough, publishes NT flag.
- **Driver Station PC** → runs `quest_watcher.py` (or the PowerShell script) in the
  background.  One terminal window, started before the match.

### 4.3  ADB Over Wi-Fi — Setup

The Quest needs Wi-Fi ADB enabled once per boot (it resets on power cycle):

```bat
adb connect 10.24.29.200:5802
```

After the first match-day connect this persists until the headset is rebooted.
The `ping_quest` button in the PyQt GUI automates the discovery step (scans
`10.24.29.200`–`10.24.29.209`) — replicate this as a one-time pre-match step if
not using the Python dashboard.

### 4.4  Timing Summary

| Step | Latency | Source |
|---|---|---|
| Double-tap to passthrough | 0 ms | Hardware |
| Frame watchdog trips | 280 ms (14 frames × 20 ms) | `K_MAX_MISSED_FRAMES` |
| `data_is_fresh` gate kills odometry | 100 ms (5 frames) | Runs every loop |
| Watcher script detects True | +50 ms | NT poll @ 20 Hz |
| ADB command fires | +50 ms (`ADB_DELAY_S`) | After detection |
| Quest app returns to foreground | ~500–1000 ms | Android activity resume |
| Quest SLAM re-acquires | 0–2000 ms | Depends on environment |
| `soft_resync` re-enables odometry | Automatic | If < `K_MAX_RESYNC_TIME` |

**Typical total recovery: ~1–3 seconds.** The odometry estimator runs on wheel
encoders + gyro alone during this window, which is acceptable for most match scenarios.