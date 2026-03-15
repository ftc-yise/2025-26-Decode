package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;

public class ShooterExecutionClass {

    public enum State {
        JITTER,
        IDLE,
        MOVE_TO_SILO,
        SPIN_WAIT,
        SPIN_UP_SHOOTER,
        FIRE_LIFT_UP,
        FIRE_LIFT_DOWN,
        NEXT_SILO,
        COMPLETE
    }

    private State state = State.IDLE;

    private final lifter lifter;
    private final Spindexer spindexer;
    private final ShooterClass shooter;
    private final ElapsedTime timer = new ElapsedTime();
    private final double LIFTER_MOVE_TIMEOUT = 1.2; // seconds
    public int shots = 0;

    public boolean dpad = false;

    public int shotsFired = 0;
    private int totalShots = 0;        // dynamically computed at cycle start
    public int currentSiloIndex = -1; // currently active silo
    public boolean jittered = false;

    // --- Force-mode flag (temporary override) ---
    public boolean forceShooting = false;

    // --- simple constants ---
    private final double LIFT_UP = 0.75;
    private final double LIFT_DOWN = 0.00;

    // --- Pattern integration fields (NEW) ---
    private ShotPatternManager patternMgr = null;   // set by caller
    private boolean patternMode = false;            // true when we are executing a pattern plan
    private final int[] firingPlan = new int[ShotPatternManager.MAX_SHOTS]; // silo indices
    private int firingCount = 0;                    // how many entries in firingPlan
    private int firingIndex = 0;                    // next index to run

    // ------------------ TIMING & AVERAGES (NEW) ------------------
    // Time tracking for a single button-press cycle
    private long pressStartTimeMs = 0L;
    private final double[] currentPressShotTimes = new double[] { Double.NaN, Double.NaN, Double.NaN };
    private int currentPressShotCount = 0;
    private boolean pressActive = false;

    // Last completed press (sticky values; start at 0.0)
    private final double[] lastCompletedPressShotTimes = new double[] { 0.0, 0.0, 0.0 };
    private double lastCompletedPressAverage = 0.0;


    // Global accumulators for averages (only count presses/shots that actually occurred)
    private long totalPresses = 0L;
    private double sumShot1 = 0.0;
    private long countShot1 = 0L;
    private double sumShot2 = 0.0;
    private long countShot2 = 0L;
    private double sumShot3 = 0.0;
    private long countShot3 = 0L;
    private double sumAllShotTimes = 0.0;
    private long countAllShots = 0L;
    private double lastMoveToSiloSec = 0.0;
    private double lastSpinWaitSec = 0.0;
    private double lastSpinUpSec = 0.0;
    private double lastFireLiftUpSec = 0.0;
    private double lastFireLiftDownSec = 0.0;

    private int lastTargetSiloIndex = -1;
    private double lastTargetAngleDeg = 0.0;


    // -----------------------------------------------------------

    public ShooterExecutionClass(Spindexer spin, ShooterClass shooter, HardwareMap hardwareMap, lifter lift) {
        this.spindexer = spin;
        this.shooter = shooter;
        this.lifter = lift;
        spin.initSilos();
        // keep previous default init (optional)
        lifter.setPresetPositions(0.0, 1.0);
        lifter.setCalibration(0.488, 0, 1.5, 1);

        // init plan to -1
        for (int i = 0; i < firingPlan.length; i++) firingPlan[i] = -1;
    }

    // ---------------- PATTERN API (NEW) ----------------
    public void setPatternManager(ShotPatternManager mgr) {
        this.patternMgr = mgr;
    }

    // ---------------- START CYCLE ----------------
    public void startCycle() {
        if (state != State.IDLE) return;

        // mark press start for timing
        beginPressTiming();

        // Read sensors now and prevent updates while we compute plan
        spindexer.sampleSensorsNow();
        spindexer.disableSensorUpdates();

        // Try to build a firing plan from the pattern manager if one exists
        if (patternMgr != null && patternMgr.hasShots()) {
            if (buildFiringPlanFromPattern()) {
                // patternMode and firingPlan set by buildFiringPlanFromPattern()
                shotsFired = 0;
                totalShots = firingCount;
                firingIndex = 0;
                currentSiloIndex = firingPlan[0];
                // go to the first planned silo
                goToSiloIndex(currentSiloIndex);
                timer.reset();
                state = State.MOVE_TO_SILO;
                timer.reset();
                return;
            } else {
                // Couldn't make a plan (e.g., pattern contains colors not present)
                // fall through to original behavior (fastest order)
                patternMode = false;
            }
        }

        // Original fallback behaviour (fastest order)
        totalShots = 0;
        Spindexer.BallColor[] colors = spindexer.getTelemetry().siloColors;
        for (Spindexer.BallColor color : colors) {
            if (color != Spindexer.BallColor.NONE) totalShots++;
        }

        if (totalShots == 0) {
            if (!jittered) {
                // jitter once to settle
                jittered = true;
                timer.reset();
                state = State.JITTER;
            } else {
                spindexer.enableSensorUpdates();
                state = State.IDLE;
                jittered = true;
            }
            return;
        }

        // start normal cycle (original)
        shotsFired = 0;
        timer.reset();
        state = State.MOVE_TO_SILO;
        moveToNextFullSilo();
        timer.reset();
    }

    // ---------------- FORCED MODE API ----------------
    public void startForcedCycle() {
        if (state != State.IDLE) {
            // If already running, do nothing (but set flag)
            forceShooting = true;
            return;
        }
        forceShooting = true;

        // mark press start for timing (forced mode triggered by button hold)
        beginPressTiming();

        spindexer.disableSensorUpdates(); // avoid conflicting sensor updates
        shotsFired = 0;
        totalShots = Integer.MAX_VALUE; // effectively "until stopped"
        // Start from next (or 0) silo so mechanism cycles
        currentSiloIndex = 0;
        spindexer.goToSilo1();
        timer.reset();
        state = State.MOVE_TO_SILO;
        timer.reset();
    }

    // Stop forced-fire and re-enable normal behavior
    public void stopForcedCycle() {
        // finalize any press in progress (e.g., user released while mid-shots)
        finalizePressIfActive();

        forceShooting = false;
        spindexer.enableSensorUpdates();
        // gracefully finish this cycle (let update() put us back to IDLE)
        state = State.COMPLETE;
    }

    // ---------------- UPDATE LOOP ----------------
    public void update() {
        lifter.update();

        switch (state) {
            case JITTER: {
                return;
            }

            case IDLE:
                if (!dpad) {
                    if (org.firstinspires.ftc.teamcode.yise.Parameters.autonomous == org.firstinspires.ftc.teamcode.yise.Parameters.AUTONOMOUS.YES) {
                        shooter.update(false, false, true);
                    } else {
                        shooter.update(false, false, false);
                    }
                }
                return;

            case MOVE_TO_SILO:
                // If forced, accept looser tolerance and keep moving between silos
                double angleErr = Math.abs(spindexer.getTelemetry().angleError);
                if (timer.seconds() > 1.25) {
                    if (angleErr < 1.8) {
                        lastMoveToSiloSec = timer.seconds();
                        spindexer.sampleSensorsNow();
                        spindexer.setNeutral();
                        state = State.SPIN_WAIT;
                        timer.reset();
                    } else if (timer.seconds() > 5) { // watchdog
                        spindexer.sampleSensorsNow();
                        state = State.SPIN_WAIT;
                        lastMoveToSiloSec = timer.seconds();
                        timer.reset();
                    }
                }
                break;

            case SPIN_WAIT:
                if (timer.seconds() > .05) {
                    lastSpinWaitSec = timer.seconds();
                    state = State.SPIN_UP_SHOOTER;
                    timer.reset();
                }
                break;

            case SPIN_UP_SHOOTER:
                if (shooter.getTelemetry().errorRPM < 0) {
                    lastSpinUpSec = timer.seconds();
                    lifter.setUp();
                    timer.reset();
                    state = State.FIRE_LIFT_UP;
                }
                break;

            case FIRE_LIFT_UP:
                if (lifter.isUp() || timer.seconds() > LIFTER_MOVE_TIMEOUT) {
                    if (timer.seconds() > .18) {
                        lastFireLiftUpSec = timer.seconds();
                        lifter.setDown();
                        timer.reset();
                        state = State.FIRE_LIFT_DOWN;
                    }
                }
                break;

            case FIRE_LIFT_DOWN:
                // On forced mode we don't decrement totalShots; we only stop when user calls stopForcedCycle()
                if (lifter.isDown() || timer.seconds() > (LIFTER_MOVE_TIMEOUT + 0.3)) {
                    shotsFired++;
                    shots++;

                    // record shot timing relative to press start (if a press is active)
                    recordShotTimingForCurrentPress();

                    // SAFETY: read current color at that silo BEFORE we possibly clear it.
                    Spindexer.BallColor firedColor = Spindexer.BallColor.NONE;
                    if (currentSiloIndex >= 0 && currentSiloIndex < 3) {
                        firedColor = spindexer.getTelemetry().siloColors[currentSiloIndex];
                    }

                    // Clear the fired silo only if not in force-mode (avoid hiding state)
                    if (!forceShooting && currentSiloIndex != -1) {
                        spindexer.clearSilo(currentSiloIndex);
                    }

                    // If we executed a pattern, CONSUME its head entry only if it matches the color we actually fired.
                    // This prevents consuming pattern entries when the robot couldn't satisfy them.
                    if (patternMode && patternMgr != null && patternMgr.hasShots()) {
                        // find first queued color (head) in the manager snapshot
                        Spindexer.BallColor[] queued = patternMgr.snapshot();
                        Spindexer.BallColor head = Spindexer.BallColor.NONE;
                        for (int i = 0; i < queued.length; i++) {
                            if (queued[i] != Spindexer.BallColor.NONE) { head = queued[i]; break; }
                        }
                        // Only consume if the head equals what we actually fired (robust against mis-read or mismatch)
                        if (head != Spindexer.BallColor.NONE && head == firedColor) {
                            patternMgr.getNext();
                        } else {
                            // If mismatch: do NOT consume. This preserves pattern integrity
                            // Optionally: log a telemetry flag or increment an internal counter for debugging.
                        }
                    }

                    if (!forceShooting) {
                        if (shotsFired >= totalShots && lifter.isDown()) {
                            // finalize press if it hasn't been finalized yet
                            finalizePressIfActive();
                            lastFireLiftDownSec = timer.seconds();

                            state = State.COMPLETE;
                        } else {
                            lastFireLiftDownSec = timer.seconds();

                            state = State.NEXT_SILO;
                        }
                    } else {
                        lastFireLiftDownSec = timer.seconds();

                        // forced -> continue cycling
                        state = State.NEXT_SILO;
                    }
                }
                break;

            case NEXT_SILO:
                moveToNextFullSilo(); // this function has been made forced-aware below
                timer.reset();
                state = State.MOVE_TO_SILO;
                break;

            case COMPLETE:
                // restore sensor updates (if not forced)
                if (!forceShooting) spindexer.enableSensorUpdates();
                spindexer.sampleSensorsNow();
                // clear pattern mode when done
                patternMode = false;
                state = State.IDLE;

                // finalize press in case we reached COMPLETE without 3 shots (safety)
                finalizePressIfActive();
                break;
        }
    }

    // ---------------- HELPER: Move to next silo (forced-aware + pattern-aware) ----------------
    private void moveToNextFullSilo() {
        if (forceShooting) {
            // blind-cycle: simply step to next index and go there
            currentSiloIndex = (currentSiloIndex + 1) % 3;
            goToSiloIndex(currentSiloIndex);
            return;
        }

        // If we have a firing plan, follow it
        if (patternMode && firingCount > 0) {
            firingIndex++;
            if (firingIndex < firingCount) {
                currentSiloIndex = firingPlan[firingIndex];
                goToSiloIndex(currentSiloIndex);
            } else {
                // done with plan
                currentSiloIndex = -1;
                state = State.COMPLETE;
            }
            return;
        }

        // normal behavior: pick next non-empty silo (keeps existing API)
        spindexer.sampleSensorsNow();
        Spindexer.BallColor[] colors = spindexer.getTelemetry().siloColors;
        int nextIndex = (currentSiloIndex + 1) % 3;

        for (int i = 0; i < 3; i++) {
            int idx = (nextIndex + i) % 3;
            if (colors[idx] != Spindexer.BallColor.NONE) {
                currentSiloIndex = idx;
                goToSiloIndex(idx);
                return;
            }
        }

        // none found -> finish
        currentSiloIndex = -1;
        state = State.COMPLETE;
    }

    // Small helper: call the correct go-to function for an index
    private void goToSiloIndex(int idx) {
        lastTargetSiloIndex = idx;
        switch (idx) {
            case 0: spindexer.goToSilo1(); break;
            case 1: spindexer.goToSilo2(); break;
            case 2: spindexer.goToSilo3(); break;
            default: break;
        }

        // capture whatever target the spindexer currently reports
        lastTargetAngleDeg = spindexer.getTelemetry().targetAngle;
    }


    // Build firing plan from the queued pattern (non-destructive)
    // Returns true if a valid plan (>=1 entries) was built, false otherwise.
    private boolean buildFiringPlanFromPattern() {
        patternMode = false;
        firingCount = 0;
        // snapshot of queued colors (do not consume yet)
        Spindexer.BallColor[] queued = patternMgr.snapshot();

        // determine number of queued (contiguous at front)
        int queuedCount = 0;
        for (int i = 0; i < queued.length; i++) {
            if (queued[i] == Spindexer.BallColor.NONE) break;
            queuedCount++;
        }
        if (queuedCount == 0) return false;

        // get current silo colors
        Spindexer.BallColor[] silos = spindexer.getTelemetry().siloColors;
        boolean[] used = new boolean[silos.length];

        ArrayList<Integer> plan = new ArrayList<>();

        // for each desired color in queue (in order) try to find a silo that contains it
        for (int q = 0; q < queuedCount; q++) {
            Spindexer.BallColor desired = queued[q];
            int found = -1;
            for (int s = 0; s < silos.length; s++) {
                if (!used[s] && silos[s] == desired) {
                    found = s;
                    break;
                }
            }
            if (found != -1) {
                plan.add(found);
                used[found] = true;
            } else {
                // If a desired color is not present, we *cannot* fully satisfy the pattern in-order.
                // Policy: use the partial plan we were able to build (if any).
                if (plan.size() == 0) {
                    // no usable entries -> fail
                    return false;
                } else {
                    // partial plan exists: use the partial plan
                    break;
                }
            }
        }

        if (plan.isEmpty()) return false;

        // copy plan to firingPlan array
        firingCount = plan.size();
        for (int i = 0; i < firingCount; i++) {
            firingPlan[i] = plan.get(i);
        }
        // mark unused slots -1
        for (int i = firingCount; i < firingPlan.length; i++) firingPlan[i] = -1;

        patternMode = true;
        return true;
    }

    // ---------------- IS BUSY ----------------
    public boolean isBusy() {
        return state != State.IDLE;
    }

    // ---------------- TIMING UTILITIES (NEW) ----------------

    // called at the start of a button-press triggered cycle or forced-cycle start
    private void beginPressTiming() {
        pressStartTimeMs = System.currentTimeMillis();
        Arrays.fill(currentPressShotTimes, Double.NaN);
        currentPressShotCount = 0;
        pressActive = true;
    }

    // called when a shot completes to capture time relative to press
    private synchronized void recordShotTimingForCurrentPress() {
        if (!pressActive) return;

        long now = System.currentTimeMillis();
        int shotIndex = shotsFired - 1; // shotsFired was incremented already
        double elapsedSec = (now - pressStartTimeMs) / 1000.0;

        // only record up to the first three shots (indices 0..2)
        if (shotIndex >= 0 && shotIndex < 3) {
            currentPressShotTimes[shotIndex] = elapsedSec;
            currentPressShotCount = Math.max(currentPressShotCount, shotIndex + 1);
        }

        // If we've recorded three shots, finalize the press
        if (currentPressShotCount >= 3) {
            finalizePress();
        }
    }

    // finalize press when either 3 shots fired OR press ended (release) OR cycle complete
    private synchronized void finalizePress() {
        if (!pressActive) return;

        // compute press average across whatever shots we recorded this press
        int n = currentPressShotCount;
        if (n == 0) {
            pressActive = false;
            return;
        }

        double pressSum = 0.0;
        for (int i = 0; i < n; i++) {
            double t = currentPressShotTimes[i];
            if (!Double.isNaN(t)) pressSum += t;
        }
        double pressAvg = pressSum / (double) n;

        // update last completed press (for opmode reading / logging)
        for (int i = 0; i < 3; i++) {
            lastCompletedPressShotTimes[i] = Double.isNaN(currentPressShotTimes[i]) ? 0.0 : currentPressShotTimes[i];
        }
        lastCompletedPressAverage = pressAvg;


        // update global accumulators
        totalPresses++;
        if (!Double.isNaN(currentPressShotTimes[0])) {
            sumShot1 += currentPressShotTimes[0];
            countShot1++;
            sumAllShotTimes += currentPressShotTimes[0];
            countAllShots++;
        }
        if (n >= 2 && !Double.isNaN(currentPressShotTimes[1])) {
            sumShot2 += currentPressShotTimes[1];
            countShot2++;
            sumAllShotTimes += currentPressShotTimes[1];
            countAllShots++;
        }
        if (n >= 3 && !Double.isNaN(currentPressShotTimes[2])) {
            sumShot3 += currentPressShotTimes[2];
            countShot3++;
            sumAllShotTimes += currentPressShotTimes[2];
            countAllShots++;
        }

        // deactivate press
        pressActive = false;
    }

    // finalize press if active (used by stopForcedCycle() and COMPLETE)
    private synchronized void finalizePressIfActive() {
        if (pressActive) finalizePress();
    }

    // ---------------- GETTERS for logging (thread-safe copies) ----------------

    // Last completed press shot times (may be NaN for missing shots)
    public synchronized double[] getLastCompletedPressShotTimes() {
        return Arrays.copyOf(lastCompletedPressShotTimes, 3);
    }

    public synchronized double getLastCompletedPressAverage() {
        return lastCompletedPressAverage;
    }

    // Global averages (returns NaN if no data)
    public synchronized double getGlobalAverageShot1() {
        return (countShot1 > 0) ? (sumShot1 / (double) countShot1) : 0.0;
    }


    public synchronized double getGlobalAverageShot2() {
        return (countShot2 > 0) ? (sumShot2 / (double) countShot2) : 0.0;
    }

    public synchronized double getGlobalAverageShot3() {
        return (countShot3 > 0) ? (sumShot3 / (double) countShot3) : 0.0;
    }

    public synchronized double getGlobalOverallAverage() {
        return (countAllShots > 0) ? (sumAllShotTimes / (double) countAllShots) : 0.0;
    }


    public synchronized long getTotalPressesCounted() {
        return totalPresses;
    }
    public synchronized double getLastMoveToSiloSec() { return lastMoveToSiloSec; }
    public synchronized double getLastSpinWaitSec() { return lastSpinWaitSec; }
    public synchronized double getLastSpinUpSec() { return lastSpinUpSec; }
    public synchronized double getLastFireLiftUpSec() { return lastFireLiftUpSec; }
    public synchronized double getLastFireLiftDownSec() { return lastFireLiftDownSec; }

    public synchronized int getLastTargetSiloIndex() { return lastTargetSiloIndex; }
    public synchronized double getLastTargetAngleDeg() { return lastTargetAngleDeg; }

}