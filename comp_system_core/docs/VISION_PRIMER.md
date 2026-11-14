# Vision and Simulation Primer

This document explains how the robot simulates gamepieces and vision cameras, and how that simulated data feeds into the actual robot code. This architecture allows us to test autonomous routines and vision logic without a physical robot or real cameras.

## 1. The Architecture Overview

The simulation is designed to be **transparent** to the robot code. The `Vision` subsystem doesn't know if it's reading data from a real Raspberry Pi or from the simulator.

Since 2027 there is no separate physics engine: each simulator is owned by a subsystem and
runs from that subsystem's `simulation_periodic()`, which the command scheduler calls
whenever the code is running in simulation (see `docs/sim_migration_plan.md`).

**Data Flow:**
1. **Ground truth (`Swerve.simulation_periodic()`)**: where the robot *really* is, published on `/SmartDashboard/Sim/ground_truth`.  Both simulators below subscribe to it - a camera sees where the robot is, not where odometry thinks it is.
2. **Gamepiece Sim (`simulation/gamepiece_sim.py`, owned by `RobotState`)**: Tracks where gamepieces are on the field.
3. **Vision Sim (`simulation/vision_sim.py`, owned by `Vision`)**: Calculates what the cameras "see" based on the robot's position.  It reads the game pieces back off the shared Field2d's `Gamepieces` object, so the two simulators never reference each other.
4. **NetworkTables**: The communication bridge. The Sim *publishes* data here, just like a real camera would.
5. **Vision Subsystem (`subsystems/vision.py`)**: *Subscribes* to NetworkTables to read the data.

---

## 2. The Gamepiece Simulator

Located in `simulation/gamepiece_sim.py`.

### Responsibilities
- **Spawning**: Creates a grid of gamepieces (fuel) between x = 7.5 m and 9.0 m, defined in `__init__`.
- **Consumption**: In every simulation step, it checks the distance between the robot and every active gamepiece.
  - If the robot drives over a gamepiece (distance < threshold), it is marked as "consumed" (inactive) and disappears.
- **Reset**: If all gamepieces are consumed, they automatically reset to their starting positions.
- **Visualization**: Updates a `FieldObject` named "Gamepieces" on the dashboard's Field2d widget so you can see them.

---

## 3. The Vision Simulator

Located in `simulation/vision_sim.py`. This is the "Virtual Coprocessor".

### How it works
It takes the **Robot's Truth Pose** (from `/SmartDashboard/Sim/ground_truth`) and the list of **Active Gamepieces** (read off the field).

Note `constants.SimConstants.k_camera_source`: while it is `'real'` (for running real coprocessors against the sim) no camera targets are published at all - only the camera FOV triangles are drawn.

For each camera defined in `constants.CameraConstants.k_cameras`:
1. **Geometry Check**: It calculates the vector from the robot to every gamepiece.
2. **Filtering**:
   - **Distance**: Is the gamepiece closer than `k_cam_distance_limit`?
   - **Field of View (FOV)**: Is the angle to the gamepiece within the camera's view cone? (Taking into account the camera's mounting rotation on the robot).
3. **Selection**: It picks the *closest* valid gamepiece.
4. **Publishing**: It writes the target details to NetworkTables topics, `/Cameras/<topic_name>/<label or type>/...` (e.g. `/Cameras/LogitechFront/yellow/distance`, `/Cameras/ArducamRight/tags/targets`).
   - **Tag cameras** with a tag in view also publish the robot's pose on `/Cameras/<camera>/poses/tag1` (ground truth, no noise). Swerve fuses it into the pose estimate exactly as it does a real tag pose.
   - **Note**: It calculates "Strafe" (lateral offset) and "Rotation" relative to the camera, mimicking real vision processing.

### Visualization
When a camera's `show_fov` toggle is on, it draws a triangle on the Field2d widget representing that camera's view cone.
- **Triangle**: The camera sees targets inside this area.

---

## 4. The Vision Subsystem

Located in `subsystems/vision.py`.

This is the code that runs on the real robot. It does **not** do any simulation math. It simply trusts NetworkTables.

### Key Methods
- `target_available(camera_key)`: Checks if the camera sees a target AND if the data timestamp is fresh.
- `get_distance(...)`, `get_strafe(...)`: Returns the raw numbers from NetworkTables.
- `nearest_to_cam(camera_key)`: Converts one camera's distance and rotation into a **robot-relative** `Pose2d` of its nearest target.
- `nearest_to_robot(camera_list)`: The nearest of those over several cameras.
- `get_vision_target_pose(current_pose, cameras, offset)`: The same target as a **field** pose, or `None`. This is what `DriveToPoseCustomControl` is given (see `docs/DRIVE_TO_POSE_HOWTO.md`).

AprilTag **poses** do not come through this subsystem. `Swerve` subscribes to `/Cameras/<camera>/poses/tag1` itself and feeds them to the pose estimator (`docs/SWERVE_PRIMER.md`, section 2).

> **Crucial Concept**: The subsystem uses the `CameraConstants.k_cameras` dictionary in `constants.py` to know which NetworkTables topics to listen to. This configuration is the "Source of Truth" for both the Sim and the Real Robot.
>
> The real robot has only the two tag cameras (`k_comp_cameras`). In simulation `k_cameras` is `k_sim_cameras`, which adds the two `hsv` (game piece) cameras and the external `front_sim` test camera.

---

## 5. Simulation Features & Flags

You can control the simulation behavior in `constants.py` under `SimConstants`.

There is one choice per sensor - where its data comes from - and the two are independent.

| Setting | Value | What happens |
| :--- | :--- | :--- |
| `k_camera_source` | `'sim'` | The Vision Sim publishes what each camera would see. |
| | `'real'` | The Vision Sim publishes nothing (it still draws FOVs), so real coprocessors own the `/Cameras/...` topics. Use this when a real camera is connected to the simulator (Hardware-in-the-Loop). |
| | `'blink_test'` | Fake data that drops one camera at a time, to test how the dashboard handles lost cameras. |
| `k_quest_source` | `'sim'` | The Quest subsystem fakes its pose from ground truth plus an error (the mock). |
| | `'real'` | A real headset; with none connected the Quest is simply "not connected". |
| `k_snap_to_real_sensors` | `True` | A real camera's AprilTag pose, or a real synced Quest, teleports the simulated robot to where it says. Does nothing without real hardware. |
| `k_kick` | `(m, m, deg)` | How far the sim-only dashboard commands `SimKickRobot` and `SimKickQuest` move the robot (without odometry noticing) or the mock Quest's tracking. Odometry in sim is perfect, so this is how to make the sensors disagree and watch the estimate (`Robot`) recover toward `GroundTruth`. |

The FOV triangles are switched per camera from the dashboard (`<sim_prefix>/FOV/<camera>_show_fov`, see `commands/sim_show_fov.py`), not from a constant.

---

## 6. How to Use It

1. **Run the Simulator**:
   ```bash
   python -m robotpy sim
   ```
   from `comp_system_core/`, with the `robo2027_alpha` environment active.
2. **Open the Dashboard (Glass)**:
   - Add the **Field** widget.
   - You will see:
     - **Robot**: The main robot icon.
     - **Gamepieces**: Small icons (usually boxes) at specific locations.
     - **FOV Cones**: Triangles projecting from the robot.
     - **GroundTruth**: Where the simulated robot really is; `Robot` is the pose estimate.
     - **Ghost Robot**: A translucent robot at `<auto_prefix>/goal_pose` (`simulation/ghost_robot.py`): the drive-to-pose goal while one is running, or the hub aim point while tracking is on.

3. **Drive**:
   - Use the joysticks to drive the robot.
   - Drive into a gamepiece -> It disappears (Consumed).
   - Point a camera at a gamepiece -> `/SmartDashboard/Vision/<camera>_targets_exist` goes true (the camera indicators on the team dashboard).
   - Hold D-pad right / left -> the robot drives onto the piece the front / left camera sees.