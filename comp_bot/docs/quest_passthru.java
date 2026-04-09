// QuestNavSubsystem.java
// Drop-in skeleton for FRC Java robots using the Meta Quest / QuestNav headset.
//
// What this does:
//   1. Reads pose frames from the QuestNav NT interface each loop.
//   2. Detects the double-tap / passthrough blackout with a two-layer watchdog.
//   3. Optionally fires ADB from the roboRIO to bring the QuestNav app back.
//   4. Gates the pose from being fed to the pose estimator when data is stale.
//   5. Soft-resyncs automatically if the outage was brief.
//
// Assumes your team's QuestNav NT interface exposes:
//   isConnected(), isTracking(), getAllUnreadPoseFrames()
// as described in the FRC QuestNav library.  Stub those methods if yours differ.
//
// To use Option A (roboRIO fires ADB directly):
//   - Set USE_ROBOT_ADB = true
//   - Deploy a static ARMv7 adb binary to src/main/deploy/adb in your robot project
//   - One-time on the roboRIO: chmod +x /home/lvuser/deploy/adb
//
// To use Option B (external watcher script on the DS PC):
//   - Set USE_ROBOT_ADB = false
//   - Run quest_watcher.py or QuestWatcher.java on the DS PC
//   - The robot still sets /QuestNav/quest_in_passthrough = true as the trigger

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.util.List;

public class QuestNavSubsystem extends SubsystemBase {

    // =========================================================================
    //  CONFIGURATION — tune these to match your robot
    // =========================================================================

    /** NT prefix — must match what the QuestNav Unity app publishes. */
    private static final String QUEST_PREFIX = "/QuestNav";

    /** Transform from Quest headset mounting point to robot center (meters). */
    private static final Transform2d QUEST_TO_ROBOT = new Transform2d(
        new Translation2d(-0.267, -0.212),   // adjust for your mounting position
        new Rotation2d(0.0)
    );

    // --- Watchdog thresholds ---
    /** Loops without a new frame before we declare passthrough (14 x 20ms = 280ms). */
    private static final int    K_MAX_MISSED_FRAMES      = 14;
    /** Loops without NT heartbeat before hard-unsyncing (50 x 20ms = 1 second). */
    private static final int    K_MAX_DISCONNECTED_COUNT = 14;
    /** Missed frames before the pose acceptance gate closes (5 x 20ms = 100ms). */
    private static final int    K_DATA_FRESH_THRESHOLD   = 5;
    /** If the headset recovers within this many seconds, soft-resync automatically. */
    private static final double K_MAX_RESYNC_TIME        = 3.0;

    // --- Field bounds (meters, 2026 Reefscape -- update each season) ---
    private static final double FIELD_LENGTH = 16.54;
    private static final double FIELD_WIDTH  =  8.05;

    // --- ADB recovery (Option A: robot code fires ADB directly) ---
    /** Set true to have the roboRIO fire ADB itself.  Requires adb binary on roboRIO. */
    private static final boolean USE_ROBOT_ADB  = false;
    private static final String  ADB_PATH       = "/home/lvuser/deploy/adb";
    private static final String  QUEST_ADB_ADDR = "10.24.29.200:5802";
    private static final double  ADB_DELAY_S    = 0.05;
    private static final double  ADB_COOLDOWN_S = 2.0;
    private static final int     ADB_MAX_RETRIES = 5;

    // =========================================================================
    //  NETWORKTABLES — publishers and entries
    // =========================================================================

    // Published to NT so the dashboard / watcher script can react
    private final BooleanEntry    questPassthroughEntry; // R/W: robot sets true, watcher clears
    private final DoublePublisher  questDtapCountPub;    // cumulative passthrough event count
    private final BooleanPublisher questConnectedPub;    // NT heartbeat alive
    private final BooleanPublisher questTrackingPub;     // SLAM tracking confident
    private final BooleanPublisher questAcceptedPub;     // all acceptance criteria met
    private final BooleanPublisher questSynchedPub;      // quest synced to robot odometry

    // Subscribed from NT (robot odometry pose for sync target)
    private final StructSubscriber<Pose2d> drivePoseSub;

    // =========================================================================
    //  STATE
    // =========================================================================

    // --- Pose ---
    private Pose2d  questPose          = new Pose2d();
    private double  questPoseTimestamp = 0.0;
    private boolean questPoseAccepted  = false;
    private boolean questHasSynched    = false;

    // --- Disconnection watchdog ---
    private int     disconnectedCount = 0;
    private boolean wasConnected      = false;

    // --- Frame watchdog / passthrough detection ---
    private int     missedFrameCount     = 0;
    private boolean wasTracking          = false;
    private int     dtapCount            = 0;
    private double  passthruStartTime    = 0.0;
    private boolean syncedBeforePassthru = false;

    // --- ADB recovery (Option A) ---
    private double adbStartTime = 0.0;
    private double adbLastFix   = 0.0;
    private int    adbRetries   = 0;

    // --- Loop counter for rate-limiting telemetry ---
    private int counter = 0;

    // =========================================================================
    //  QUESTNAV INTERFACE STUB
    //  Replace these with your team's actual QuestNav library calls.
    //  The FRC QuestNav sample project (github.com/QuestNav/QuestNav) provides
    //  a Java helper class that wraps these NT reads for you.
    // =========================================================================

    /** Returns true if the NT connection to the headset is alive (heartbeat < 200ms). */
    private boolean questIsConnected() {
        // TODO: return yourQuestNavHelper.isConnected();
        return false;
    }

    /** Returns true if the headset's SLAM algorithms are confident. */
    private boolean questIsTracking() {
        // TODO: return yourQuestNavHelper.isTracking();
        return false;
    }

    /**
     * Returns all unread pose frames since the last call.
     * Replace QuestFrame with your library's actual frame type.
     */
    private List<QuestFrame> questGetAllUnreadFrames() {
        // TODO: return yourQuestNavHelper.getAllUnreadPoseFrames();
        return List.of();
    }

    /** Sets the Quest's internal origin so its reported pose matches targetPose. */
    private void questSetPose(Pose2d targetPose) {
        // TODO: yourQuestNavHelper.setPose(new Pose3d(targetPose));
    }

    /** Minimal placeholder so the file compiles as a standalone skeleton. */
    private static class QuestFrame {
        public Pose3d getPose3d()    { return new Pose3d(); }
        public double getTimestamp() { return 0.0; }
    }

    // =========================================================================
    //  CONSTRUCTOR
    // =========================================================================

    public QuestNavSubsystem() {
        super();

        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        // Publishers
        questPassthroughEntry = inst.getBooleanTopic(QUEST_PREFIX + "/quest_in_passthrough").getEntry(false);
        questDtapCountPub     = inst.getDoubleTopic( QUEST_PREFIX + "/quest_dtap_count").publish();
        questConnectedPub     = inst.getBooleanTopic(QUEST_PREFIX + "/quest_connected").publish();
        questTrackingPub      = inst.getBooleanTopic(QUEST_PREFIX + "/quest_tracking").publish();
        questAcceptedPub      = inst.getBooleanTopic(QUEST_PREFIX + "/quest_pose_accepted").publish();
        questSynchedPub       = inst.getBooleanTopic(QUEST_PREFIX + "/questnav_synched").publish();

        // Subscriber: the swerve subsystem's drive pose, used as sync target
        // Update this topic path to match what your swerve subsystem publishes
        drivePoseSub = inst.getStructTopic(
            "/SmartDashboard/Swerve/drive_pose", Pose2d.struct).subscribe(new Pose2d());

        // Seed initial values
        questPassthroughEntry.set(false);
        questDtapCountPub.set(0);
        questSynchedPub.set(false);
    }

    // =========================================================================
    //  PERIODIC — runs every 20ms
    // =========================================================================

    @Override
    public void periodic() {
        counter++;

        boolean isConnected = questIsConnected();
        boolean isTracking  = questIsTracking();

        // -----------------------------------------------------------------
        // Step 1: Disconnection watchdog
        //   Fires a hard unsync if the NT connection is totally gone for
        //   K_MAX_DISCONNECTED_COUNT loops (~1 second).
        // -----------------------------------------------------------------
        if (!isConnected) {
            disconnectedCount++;
            if (disconnectedCount > K_MAX_DISCONNECTED_COUNT && wasConnected) {
                System.out.printf("*** QuestNav connection dropped for %.2fs at %.2fs ***%n",
                                  disconnectedCount / 50.0, Timer.getFPGATimestamp());
                questUnsyncOdometry();
                wasConnected = false;
            }
        } else {
            disconnectedCount = 0;
            wasConnected = true;
        }

        // -----------------------------------------------------------------
        // Step 2: Frame watchdog
        //   Detects passthrough faster than the NT heartbeat (100ms vs 200ms).
        //   On a double-tap the Quest app suspends immediately -- frames stop
        //   arriving even though the NT connection stays technically alive.
        // -----------------------------------------------------------------
        List<QuestFrame> frames   = questGetAllUnreadFrames();
        boolean          gotFrame = isTracking && frames != null && !frames.isEmpty();

        if (gotFrame) {
            missedFrameCount = 0;
            QuestFrame lastFrame = frames.get(frames.size() - 1);

            try {
                Pose2d newPose = lastFrame.getPose3d().toPose2d().transformBy(QUEST_TO_ROBOT);
                questPose          = newPose;
                questPoseTimestamp = lastFrame.getTimestamp();

                if (!wasTracking) {
                    // Tracking restored after a blackout
                    System.out.printf("Quest tracking restored at %.2fs%n",
                                      Timer.getFPGATimestamp());
                    questPassthroughEntry.set(false);   // clear flag for dashboard / watcher

                    double timeInPassthru = Timer.getFPGATimestamp() - passthruStartTime;
                    if (timeInPassthru < K_MAX_RESYNC_TIME
                            && syncedBeforePassthru
                            && !questHasSynched) {
                        // Brief outage: SLAM still accurate, auto-recover
                        System.out.printf("Quest recovered in %.2fs -- soft resyncing%n",
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
            // No new frame this loop
            missedFrameCount++;

            if (missedFrameCount > K_MAX_MISSED_FRAMES && wasTracking) {
                // *** PASSTHROUGH / BLACKOUT DETECTED ***
                wasTracking          = false;
                passthruStartTime    = Timer.getFPGATimestamp();
                syncedBeforePassthru = questHasSynched;

                questPassthroughEntry.set(true);   // signal dashboard or watcher script
                dtapCount++;
                questDtapCountPub.set(dtapCount);

                System.out.printf("QuestNav blackout at %.2fs (event #%d)%n",
                                  Timer.getFPGATimestamp(), dtapCount);
            }
        }

        // -----------------------------------------------------------------
        // Step 3: Pose acceptance gate -- two-layer protection
        //
        //   isConnected  -- NT heartbeat alive      (~200ms window)
        //   dataIsFresh  -- new frame within 100ms  (2x faster than NT timeout)
        //
        // If the Quest double-taps, dataIsFresh trips first and immediately
        // stops feeding the pose estimator, before NT heartbeat can expire.
        // -----------------------------------------------------------------
        boolean dataIsFresh = (missedFrameCount <= K_DATA_FRESH_THRESHOLD);
        boolean inBounds    = inFieldBounds(questPose);

        questPoseAccepted = inBounds && isConnected && isTracking && wasTracking && dataIsFresh;

        // -----------------------------------------------------------------
        // Step 4: ADB recovery (Option A -- roboRIO fires ADB directly)
        //   Only active when USE_ROBOT_ADB = true.
        // -----------------------------------------------------------------
        if (USE_ROBOT_ADB) {
            checkAndFireAdb();
        }

        // -----------------------------------------------------------------
        // Step 5: Publish telemetry, rate-limited to every 5 loops (100ms)
        // -----------------------------------------------------------------
        if (counter % 5 == 0) {
            questConnectedPub.set(isConnected);
            questTrackingPub.set(isTracking);
            questAcceptedPub.set(questPoseAccepted);
        }
    }

    // =========================================================================
    //  SYNC / UNSYNC HELPERS
    // =========================================================================

    /**
     * Full sync: tell the Quest its current pose equals the robot's odometry pose.
     * Call this from a button binding once the robot is in a known location.
     */
    public void questSyncOdometry() {
        Pose2d drivePose = drivePoseSub.get();
        questSetPose(drivePose);
        questHasSynched = true;
        questSynchedPub.set(true);
        System.out.printf("Quest synced to (%.2f, %.2f) at %.2fs%n",
                          drivePose.getX(), drivePose.getY(), Timer.getFPGATimestamp());
    }

    /**
     * Soft resync: accept the Quest's current pose as valid without moving its origin.
     * Used after a brief passthrough recovery where SLAM is assumed still accurate.
     */
    public void questSoftResync() {
        questHasSynched = true;
        questSynchedPub.set(true);
        System.out.printf("Quest soft-resynced at %.2fs%n", Timer.getFPGATimestamp());
    }

    /**
     * Hard unsync: stop trusting the Quest pose until a manual sync is performed.
     * Called automatically on prolonged disconnection or by operator command.
     */
    public void questUnsyncOdometry() {
        questHasSynched = false;
        questSynchedPub.set(false);
        System.out.printf("Quest unsynced at %.2fs%n", Timer.getFPGATimestamp());
    }

    /**
     * Reset the Quest's origin to a known field position (e.g., subwoofer start).
     * Automatically unsyncs so the operator must explicitly re-sync afterward.
     */
    public void questResetOdometry(Pose2d targetPose) {
        questSetPose(targetPose);
        System.out.printf("Quest reset to (%.2f, %.2f) at %.2fs%n",
                          targetPose.getX(), targetPose.getY(), Timer.getFPGATimestamp());
        questUnsyncOdometry();
    }

    // =========================================================================
    //  PUBLIC ACCESSORS  (call from your swerve / pose estimator subsystem)
    // =========================================================================

    /** Latest robot-frame Pose2d from the Quest. */
    public Pose2d getQuestPose() { return questPose; }

    /** Timestamp of the latest frame in FPGA seconds, for latency compensation. */
    public double getQuestPoseTimestamp() { return questPoseTimestamp; }

    /** True when all acceptance criteria pass -- safe to feed to the pose estimator. */
    public boolean isPoseAccepted() { return questPoseAccepted; }

    /** True when the Quest has been synced and its data is currently trusted. */
    public boolean isQuestSynced() { return questHasSynched; }

    /** True when the NT heartbeat to the headset is alive. */
    public boolean isQuestConnected() { return questIsConnected(); }

    // =========================================================================
    //  OPTION A: roboRIO fires ADB directly
    //
    //  Prerequisites:
    //    1. Place a statically-linked ARMv7 adb binary at src/main/deploy/adb
    //       WPILib copies deploy/ to /home/lvuser/deploy/ at deploy time.
    //    2. Once after deploying:  ssh lvuser@10.TE.AM.2  then
    //       chmod +x /home/lvuser/deploy/adb
    //    3. Set USE_ROBOT_ADB = true above.
    //
    //  The roboRIO and the Quest are on the same field subnet, so it can reach
    //  the Quest's ADB TCP port (5802) just like the driver-station PC can.
    // =========================================================================

    private void checkAndFireAdb() {
        boolean isInPassthru = questPassthroughEntry.get();
        double  now          = Timer.getFPGATimestamp();

        if (isInPassthru) {
            if (adbStartTime == 0.0) {
                adbStartTime = now;
            }

            double elapsed   = now - adbStartTime;
            double sinceLast = now - adbLastFix;

            if (elapsed > ADB_DELAY_S && sinceLast > ADB_COOLDOWN_S) {
                if (adbRetries < ADB_MAX_RETRIES) {
                    adbRetries++;
                    System.out.printf("ADB fix attempt %d/%d at %.2fs%n",
                                      adbRetries, ADB_MAX_RETRIES, now);
                    try {
                        new ProcessBuilder(
                            ADB_PATH, "-s", QUEST_ADB_ADDR,
                            "shell", "am", "start",
                            "-n", "gg.QuestNav.QuestNav/com.unity3d.player.UnityPlayerGameActivity"
                        ).start();  // fire-and-forget; roboRIO doesn't wait for ADB to complete
                    } catch (IOException e) {
                        System.out.println("ADB launch failed: " + e.getMessage());
                    }
                    adbLastFix   = now;
                    adbStartTime = now;  // require another full ADB_DELAY_S before next attempt

                } else if (adbRetries == ADB_MAX_RETRIES) {
                    System.out.printf("ADB max retries (%d) reached -- giving up.%n",
                                      ADB_MAX_RETRIES);
                    adbRetries++;  // prevent repeated log spam
                }
            }

        } else {
            if (adbRetries > 0) {
                System.out.printf("Passthrough resolved after %d ADB attempt(s).%n", adbRetries);
            }
            adbStartTime = 0.0;
            adbRetries   = 0;
        }
    }

    // =========================================================================
    //  PRIVATE HELPERS
    // =========================================================================

    private boolean inFieldBounds(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        return x > 0 && x < FIELD_LENGTH && y > 0 && y < FIELD_WIDTH;
    }
}