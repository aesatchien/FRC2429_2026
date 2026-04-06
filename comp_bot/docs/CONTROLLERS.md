# Robot Controls Reference Manual

This document serves as the primary reference for the Driver (Xbox Controller) and Copilot (Button Box) controls. 

![Xbox Controller Reference](https://upload.wikimedia.org/wikipedia/commons/thumb/4/43/Xbox-console-controller.png/320px-Xbox-console-controller.png)

---

## 🎮 Driver Controls (Xbox Controller)

| Button | Action / Overview | Commands Called (Under the Hood) |
| :--- | :--- | :--- |
| **Y Button** | **Reset Gyro / Sync QuestNav:** Press to reset field-centric forward to the robot's current heading. Hold for 0.5s to sync the QuestNav odometry. | `ResetFieldCentric(angle=0)`<br>*Hold 0.5s:* `quest_sync_odometry()` |
| **A Button** | **Auto Shooting Cycle (Hold):** Slowly spins intake rollers to feed, starts the shooting sequence, and orchestrates the intake dropping/raising to feed the ball. Releases to stow and stop. | *While Held:* `ParallelCommandGroup(ShootingCommand, Intake_Deploy('shoot/shoot2'))` <br>*On Release:* `Intake_Deploy('down')`, `Intake_Set_RPM(0)` |
| **X Button** | **Manual Shoot - 3300 RPM (Hold):** Drops intake to shoot position and fires at 3300 RPM. | *While Held:* `ShootingCommand(3300)`, `Intake_Deploy('shoot')`<br>*On Release:* `Intake_Deploy('down')` |
| **B Button** | **Manual Shoot - 4500 RPM (Hold):** Revs up to 4500 RPM, drops intake to shoot position, and fires. | *On Press:* `set_shooter_rpm(fire_up_speed)`<br>*While Held:* `ShootingCommand(4500)`, `Intake_Deploy('shoot')` |
| **Right Bumper (RB)** | **Auto-Track Target (Hold):** Snaps the robot's rotation to face the target and pre-spins the shooter flywheel. | *On Press:* `start_tracking()`, `set_shooter_rpm(fire_up_speed)`<br>*On Release:* `stop_tracking()` |
| **Left Bumper (LB)** | **Slow Intake:** Spins the intake at 1500 RPM. | `Intake_Set_RPM(1500)` |
| **Left Trigger (LT)** | **Swerve X-Lock (Hold):** Locks the wheels into an X formation to brake and resist defense. | `SwerveSetX()` |
| **D-Pad Up** | **Stow Intake:** Raises the intake to the upward (home) position. | `Intake_Deploy('up')` |
| **D-Pad Down** | **Deploy Intake:** Lowers the intake to the floor. | `Intake_Deploy('down')` |
| **Back (View)** | **Stop Intake:** Stops the intake rollers immediately. | `Intake_Set_RPM(0)` |
| **Start (Menu)** | **Eject / Outtake (Hold):** Forces the intake down and reverses the intake and hopper rollers to spit out a jammed game piece. | *While Held:* `Intake_Deploy('down')`, `Intake_Set_RPM(-2500)`, `set_hopper_rpm(-hopper)` |

---

## 🎛️ Copilot Controls (Button Box)

| Button | Action / Overview | Commands Called (Under the Hood) |
| :--- | :--- | :--- |
| **Button 1** | **Zero Intake Encoder:** Resets and moves the intake to the bottom physical limit. | `zero_intake()` |
| **Button 2** | **Max Intake Angle:** Resets and moves the intake to the top physical limit. | `set_angle_max()` |
| **Button 3** | **Emergency Stop Tracking:** Forces the targeting subsystem to stop tracking. | `stop_tracking()` |
| **Button 4** | **Sync QuestNav:** Forces a manual synchronization of the QuestNav odometry. | `quest_sync_odometry()` |
| **Button 5** | **Disable QuestNav:** Turns OFF QuestNav odometry updates to the Swerve drive. | `quest_enabled_toggle('off')` |
| **Button 6** | **Enable QuestNav:** Turns ON QuestNav odometry updates to the Swerve drive. | `quest_enabled_toggle('on')` |
| **Button 7** | **Unsync QuestNav:** Forces a manual unsync of the QuestNav, ignoring its coordinates. | `quest_unsync_odometry()` |
| **Button 8** | **Stow & Stop Intake (Hold):** Raises the intake and stops the rollers. | *While Held:* `Intake_Deploy('up')`, `Intake_Set_RPM(0)` |
| **Button 9** | **Intake to Shoot Ready:** Moves the intake to the exact shooting angle and runs the rollers slowly (500 RPM). | `set_intake_position(shoot_angle)`, `set_intake_rpm(500)` |
| **Button 10** | **Floor Intake (Hold):** Deploys the intake to the floor and runs rollers at 3000 RPM. | *While Held:* `Intake_Deploy('down')`, `Intake_Set_RPM(3000)` |
| **Button 11** | **Decrease RPM Offset:** Adjusts shooter RPM tuning down by 250 RPM. | `set_shooting_offset(-250)` |
| **Button 12** | **Increase RPM Offset:** Adjusts shooter RPM tuning up by 250 RPM. | `set_shooting_offset(250)` |

---

### 💡 Key Concepts
* **"Hold" Actions (`whileTrue`):** The action is actively running *only* while you are physically pressing the button down. Once you release it, the action stops or reverts to a safe state.
* **"Toggle/Press" Actions (`onTrue`):** The action is a one-shot fire. You just tap it once to change the state.
* **QuestNav Management:** If the Quest loses tracking in a match (e.g. heavy hits), Button 4 (Sync) and Button 7 (Unsync) allow the copilot to manually manage odometry trust.