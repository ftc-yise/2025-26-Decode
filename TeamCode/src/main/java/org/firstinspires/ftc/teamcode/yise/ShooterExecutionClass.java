package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

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

    public ShooterExecutionClass(Spindexer spin, ShooterClass shooter, HardwareMap hardwareMap, lifter lift) {
        this.spindexer = spin;
        this.shooter = shooter;
        this.lifter = lift;
        spin.initSilos();
        // keep previous default init (optional)
        lifter.setPresetPositions(0.0, 1.0);
        lifter.setCalibration(1.41, 0, 2.5, 1);

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
                double t = timer.seconds();
                if (t < 0.22) {
                    spindexer.setManual(0.3);
                } else if (t < 0.44) {
                    spindexer.setManual(-0.2);
                } else {
                    spindexer.setManual(0.0);
                    spindexer.enableSensorUpdates();
                    spindexer.goToSilo1();
                    if (t > 1.5 && lifter.isDown()) {
                        timer.reset();
                        state = State.COMPLETE;
                    }
                }
                return;
            }

            case IDLE:
                if (Parameters.autonomous == Parameters.AUTONOMOUS.YES) {
                    shooter.update(false, false, true);
                } else {
                    shooter.update(false, false, false);
                }
                return;

            case MOVE_TO_SILO:
                // If forced, accept looser tolerance and keep moving between silos
                double angleErr = Math.abs(spindexer.getTelemetry().angleError);
                if (timer.seconds() > 1.25) {
                    if (angleErr < 1.8) {
                        spindexer.sampleSensorsNow();
                        spindexer.setNeutral();
                        state = State.SPIN_WAIT;
                        timer.reset();
                    } else if (timer.seconds() > 5) { // watchdog
                        spindexer.sampleSensorsNow();
                        state = State.SPIN_WAIT;
                        timer.reset();
                    }
                }
                break;

            case SPIN_WAIT:
                if (timer.seconds() > .15) {
                    state = State.SPIN_UP_SHOOTER;
                    timer.reset();
                }
                break;

            case SPIN_UP_SHOOTER:
                if (shooter.getTelemetry().errorRPM < 0) {
                    lifter.setUp();
                        timer.reset();
                        state = State.FIRE_LIFT_UP;
                }
                break;

            case FIRE_LIFT_UP:
                if (lifter.isUp() || timer.seconds() > LIFTER_MOVE_TIMEOUT) {
                    if (timer.seconds() > .18) {
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
                            state = State.COMPLETE;
                        } else {
                            state = State.NEXT_SILO;
                        }
                    } else {
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
        switch (idx) {
            case 0: spindexer.goToSilo1(); break;
            case 1: spindexer.goToSilo2(); break;
            case 2: spindexer.goToSilo3(); break;
            default: break;
        }
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

}
