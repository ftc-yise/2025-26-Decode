package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.yise.Parameters;

import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.Arrays;

/**
 * Spindexer — positional-servo version (pose-driven for silos)
 *
 * SILO POSITIONS:
 *  - Silo1 -> 0.0
 *  - Silo2 -> 0.5
 *  - Silo3 -> 1.0
 *
 * Other behavior preserved (sensors, logging, manual cycle). TARGET mode (setTarget(angle))
 * still maps 0..360° -> 0..1 as a fallback, but silo commands bypass angle math entirely.
 */
public class Spindexer {

    // ─────────────────────────────────────────────────────────────────────
    // MODES
    // ─────────────────────────────────────────────────────────────────────
    public BallColor[] siloColors = new BallColor[3];
    private int[] lastSeenSiloBySensor = {-1, -1, -1};

    // smoothing / filtering (position-specific)
    private double lastPositionFiltered = 0.0;

    public enum Mode {
        NEUTRAL,
        SILO_1,
        SILO_2,
        SILO_3,
        MANUAL,     // cycles through fixed poses
        TARGET,     // map arbitrary angle -> position (fallback)
        SEQUENCE
    }
    public enum BallColor {
        GREEN,
        PURPLE,
        NONE
    }

    private final BallColor[] silos = {
            BallColor.NONE,
            BallColor.NONE,
            BallColor.NONE
    };

    public static double silo1;
    public static double silo2;
    public static double silo3;

    private static final double SILO_1_TARGET_DEG = 64.5;
    private static final double SILO_2_TARGET_DEG = 180.0;
    private static final double SILO_3_TARGET_DEG = 294.0;

    // keep your existing angle-based silo definitions (not used for direct silo commands now,
    // but retained for backward compatibility / initSilos)
    public static double[] SILO_ANGLES = {
            silo1,   // SILO_1
            silo2,   // SILO_2
            silo3    // SILO_3
    };

    // fixed positions for the three silos: silo1 -> 0.0, silo2 -> 0.5, silo3 -> 1.0
    public static double[] SILO_POSITIONS = {
            0.0,   // SILO_1 -> servo position 0.0
            0.5,   // SILO_2 -> servo position 0.5
            1.0    // SILO_3 -> servo position 1.0
    };

    private static final double[] SENSOR_OFFSETS = {
            0.0,    // middle
            120.0,  // backLeft
            240.0   // backRight
    };

    public Mode mode = Mode.NEUTRAL;

    // Hardware
    private Servo spindexer;              // positional servo
    private AnalogInput encoder;
    private ColorSensor middle = null;
    private ColorSensor backLeft = null;
    private ColorSensor backRight = null;

    // Angle variables (kept for fallback)
    public double targetAngleDeg = 0;

    // Direct target position (0..1) used for silo/manual modes
    private double targetPosition = 0.0;

    // Constants
    private final double MAX_VOLTAGE = 3.3;

    // reading gate
    private boolean sensorUpdatesEnabled = true;

    // Velocity / angle helpers (still used for some checks)
    private double lastAngle = 0.0;            // last measured angle (deg)
    private long lastTimeMs = System.currentTimeMillis();

    // Position rate limiting (keeps position changes smooth)
    private double lastPosition = 0.0;
    private final double MAX_POSITION_DELTA = 0.12; // maximum allowed position change per loop

    // Sequencing state
    private int siloStep = 0;
    private boolean sequenceActive = false;

    // Manual cycling state
    private boolean manualCycleActive = false;
    private int manualCycleIndex = 0;
    private long lastManualCycleMs = 0;
    private static final long MANUAL_CYCLE_MS = 700; // ms between pose switches in manual cycle

    // Angle tolerance for various checks (kept for color/silo-angle fallback)
    private final double ANGLE_TOLERANCE = 0.5;
    private final double ANGLE_TOLERANCE_COLOR = 15;

    // Position arrival tolerance (for latch)
    private final double POSITION_TOLERANCE = 0.02; // ~2% of travel

    // --- arrival latch ---
    private boolean atTargetLatched = false;


    // ─────────────────────────────────────────────────────────────────────
    // TELEMETRY STRUCT
    // ─────────────────────────────────────────────────────────────────────
    public static class TelemetryPacket {
        public Mode mode;
        public double voltage;
        public double currentAngle;
        public double targetAngle;
        public double angleError;
        public double appliedPower;   // repurposed as applied position (0..1)
        public double manualPower;
        public double pidP;
        public double pidI;
        public double pidD;
        public BallColor[] siloColors = new BallColor[3];
    }

    private TelemetryPacket t = new TelemetryPacket();

    // CSV Logging
    private FileWriter csv;
    private DecimalFormat df = new DecimalFormat("0.00");

    // Constructor
    public Spindexer(HardwareMap hardwaremap) {
        spindexer = hardwaremap.get(Servo.class, "spin");
        encoder = hardwaremap.get(AnalogInput.class, "spinInput");

        middle = hardwaremap.get(ColorSensor.class, "middlecolorsensor");
        backLeft = hardwaremap.get(ColorSensor.class, "BLcolorsensor");
        backRight = hardwaremap.get(ColorSensor.class, "BRcolorsensor");

        // init lastAngle so first velocity estimate is small
        double v = encoder.getVoltage();
        lastAngle = normalize((v / MAX_VOLTAGE) * 360.0);
        lastPositionFiltered = angleToPosition(lastAngle);     // initialize position filter
        lastTimeMs = System.currentTimeMillis();

        // initialize servo to current position
        spindexer.setPosition(lastPositionFiltered);
        lastPosition = lastPositionFiltered;
        targetPosition = lastPositionFiltered;
    }

    // ─────────────────────────────────────────────────────────────────────
    // MODE COMMANDS
    // ─────────────────────────────────────────────────────────────────────
    public void setNeutral() {
        mode = Mode.NEUTRAL;
        sequenceActive = false;
        manualCycleActive = false;
    }

    // direct silo commands: set the fixed position and mode
    public void goToSilo1() {
        targetPosition = SILO_POSITIONS[0];
        targetAngleDeg = SILO_1_TARGET_DEG;
        mode = Mode.SILO_1;
    }

    public void goToSilo2() {
        targetPosition = SILO_POSITIONS[1];
        targetAngleDeg = SILO_2_TARGET_DEG;
        mode = Mode.SILO_2;
    }

    public void goToSilo3() {
        targetPosition = SILO_POSITIONS[2];
        targetAngleDeg = SILO_3_TARGET_DEG;
        mode = Mode.SILO_3;
    }

    /**
     * Start a non-blocking manual cycle through poses:
     * Silo1 -> Silo2 -> Silo3 -> repeat.
     */
    public void startManualCycle() {
        manualCycleActive = true;
        manualCycleIndex = 0;
        lastManualCycleMs = System.currentTimeMillis();
        targetPosition = SILO_POSITIONS[manualCycleIndex];
        mode = Mode.MANUAL;
    }

    public void stopManualCycle() {
        manualCycleActive = false;
        mode = Mode.NEUTRAL;
    }

    // set an arbitrary angle target (kept as fallback if you want to use angle mapping)
    public void setTarget(double angleDeg) {
        targetAngleDeg = normalize(angleDeg);
        mode = Mode.TARGET;
    }

    // allow direct pose commands (convenience)
    public void goToPose(double val) {
        targetPosition = Range.clip(val, 0.0, 1.0);
        mode = Mode.TARGET; // use TARGET so update() writes the mapped pos (we check targetPosition first)
    }

    // ─────────────────────────────────────────────────────────────────────
    // AUTOMATIC SILO SEQUENCE
    // ─────────────────────────────────────────────────────────────────────
    public void startSequence() {
        siloStep = 0;
        sequenceActive = true;
        mode = Mode.SEQUENCE;
    }

    private void runSequence() {
        if (!sequenceActive) return;

        switch (siloStep) {
            case 0:
                goToSilo2();           // now uses fixed position 0.5
                if (atTarget()) siloStep++;
                break;

            case 1:
                goToSilo3();           // now uses fixed position 1.0
                if (atTarget()) siloStep++;
                break;

            case 2:
                goToSilo1();           // now uses fixed position 0.0
                if (atTarget()) siloStep++;
                break;

            case 3:
                sequenceActive = false;
                setNeutral();
                break;
        }
    }



    // ─────────────────────────────────────────────────────────────────────
    // UPDATE LOOP — CALL EVERY LOOP
    // ─────────────────────────────────────────────────────────────────────
    public void update() {

        // --- read encoder / current angle ---
        double voltage = encoder.getVoltage();
        double current = normalize((voltage / MAX_VOLTAGE) * 360.0);

        // ALWAYS update sensors (telemetry must never starve)
        if (sensorUpdatesEnabled) {
            mapSensorsToSilos(current);
        }

        // run sequence if needed
        if (mode == Mode.SEQUENCE) runSequence();

        // Manual cycling handling (non-blocking): advance to next silo every MANUAL_CYCLE_MS
        if (manualCycleActive) {
            long now = System.currentTimeMillis();
            if (now - lastManualCycleMs >= MANUAL_CYCLE_MS) {
                manualCycleIndex = (manualCycleIndex + 1) % 3;
                targetPosition = SILO_POSITIONS[manualCycleIndex];
                lastManualCycleMs = now;
            }
        }

        double appliedPosition;

        // Mode -> position mapping (silo modes directly map to fixed positions)
        if (mode == Mode.NEUTRAL) {
            appliedPosition = angleToPosition(current); // hold current mapped position
            atTargetLatched = false;
        } else if (mode == Mode.SILO_1) {
            appliedPosition = SILO_POSITIONS[0];
        } else if (mode == Mode.SILO_2) {
            appliedPosition = SILO_POSITIONS[1];
        } else if (mode == Mode.SILO_3) {
            appliedPosition = SILO_POSITIONS[2];
        } else if (mode == Mode.MANUAL) {
            appliedPosition = targetPosition;
        } else if (mode == Mode.SEQUENCE) {
            // sequence uses goToSiloX() so targetPosition already set appropriately
            appliedPosition = targetPosition;
        } else { // Mode.TARGET -> fallback mapping from angle to position
            // If a direct targetPosition was set (via goToPose or similar), prefer it
            appliedPosition = targetPosition;
            // If targetPosition left at default (we'll detect a negative or equals lastPositionFiltered),
            // fallback to mapping targetAngle
            // (we initially set targetPosition to lastPositionFiltered, so this branch uses the explicit pos)
            if (Double.isNaN(appliedPosition) || appliedPosition < 0.0 || appliedPosition > 1.0) {
                appliedPosition = angleToPosition(targetAngleDeg);
            }
        }

        // smooth / rate-limit position changes then apply
        double outputPosition = positionRateLimit(appliedPosition);
        spindexer.setPosition(outputPosition);

        // arrival latch (position-based)
        double posErr = Math.abs(outputPosition - lastPositionFiltered);
        if (posErr < POSITION_TOLERANCE) {
            atTargetLatched = true;
        } else {
            atTargetLatched = false;
        }

        // --- telemetry packet ---
        t.mode = mode;
        t.voltage = voltage;
        t.currentAngle = current;
        t.targetAngle = targetAngleDeg;
        t.angleError = smallestAngleError(targetAngleDeg, current);
        t.appliedPower = outputPosition; // repurposed as "position"
        t.manualPower = 0.0;

        for (int i = 0; i < 3; i++) {
            t.siloColors[i] = silos[i];
        }

        writeCSV(current, targetAngleDeg, t.angleError);

        if (mode == Mode.NEUTRAL) {
            Arrays.fill(lastSeenSiloBySensor, -1);
        }
    }

    // Map angle (0..360) to servo position (0..1) — fallback when target isn't a named silo
    private double angleToPosition(double angleDeg) {
        double normalized = normalize(angleDeg); // 0..360
        return normalized / 360.0;
    }

    // Rate limit position changes (simple clamped delta per loop)
    private double positionRateLimit(double requested) {
        double diff = requested - lastPosition;
        double allowed = MAX_POSITION_DELTA;
        if (Math.signum(requested) != Math.signum(lastPosition)) {
            // allow quicker reversal proportionally (not strictly needed but safe)
            allowed *= 4.0;
        }
        if (Math.abs(diff) > allowed) {
            requested = lastPosition + Math.signum(diff) * allowed;
        }
        lastPosition = requested;
        // filter for smoother output
        lastPositionFiltered = 0.55 * lastPositionFiltered + (1.0 - 0.55) * requested;
        return Range.clip(lastPositionFiltered, 0.0, 1.0);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Map sensors -> angle -> silo index (ownership fixer) — unchanged
    // ─────────────────────────────────────────────────────────────────────
    private void mapSensorsToSilos(double currentAngle) {
        ColorSensor[] sensors = { middle, backLeft, backRight };

        for (int sensorIdx = 0; sensorIdx < sensors.length; sensorIdx++) {
            ColorSensor s = sensors[sensorIdx];
            // compute the world angle that this sensor is "looking at"
            double sensorAngle = normalize(currentAngle + SENSOR_OFFSETS[sensorIdx]);

            // find nearest silo index for that angle (with hysteresis fallback)
            int siloIndex = angleToSilo(sensorAngle);
            if (siloIndex == -1) {
                int prev = lastSeenSiloBySensor[sensorIdx];
                if (prev != -1) {
                    double prevErr = Math.abs(smallestAngleError(SILO_ANGLES[prev], sensorAngle));
                    if (prevErr < ANGLE_TOLERANCE * 3.0) siloIndex = prev;
                }
            }

            if (siloIndex != -1) {
                // write the sensor reading into the silo slot
                silos[siloIndex] = detectBall(s, sensorIdx);
                lastSeenSiloBySensor[sensorIdx] = siloIndex;
            } else {
                lastSeenSiloBySensor[sensorIdx] = -1;
            }
        }
    }

    // Call ONLY when aligned at a read/fire position
    public void sampleSensorsNow() {
        double voltage = encoder.getVoltage();
        double current = normalize((voltage / MAX_VOLTAGE) * 360.0);

        // map sensors -> silos
        mapSensorsToSilos(current);

        // copy into telemetry packet so callers reading getTelemetry() see the update immediately
        for (int i = 0; i < 3; i++) {
            t.siloColors[i] = silos[i];
        }
    }


    // HELPERS
    private static double normalize(double angle) {
        angle %= 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    private boolean atTarget() {
        return atTargetLatched;
    }

    private double smallestAngleError(double target, double current) {
        double diff = target - current;
        if (diff > 180) diff -= 360;
        if (diff < -180) diff += 360;
        return diff;
    }

    private double clamp(double p, double min, double max) {
        return Math.max(min, Math.min(max, p));
    }

    // CSV logger
    private void writeCSV(double angle, double target, double error) {
        if (csv == null) return;
        try {
            csv.write(System.currentTimeMillis() + "," +
                    df.format(angle) + "," +
                    df.format(target) + "," +
                    df.format(error) + "," +
                    df.format(t.pidP) + ",0,0,");
            csv.flush();
        } catch (IOException ignored) {}
    }

    public TelemetryPacket getTelemetry() { return t; }

    public void stop() {
        // stop motion: set neutral and hold current position
        spindexer.setPosition(lastPosition); // hold last position
        mode = Mode.NEUTRAL;
        manualCycleActive = false;
        sequenceActive = false;
    }

    // Return a BallColor using sensor thresholds (sensorIndex: 0=middle, 1=backLeft, 2=backRight)
    private BallColor detectBall(ColorSensor s, int sensorIndex) {
        int g = s.green();
        int b = s.blue();

        if (sensorIndex == 0) { // middle sensor
            if (b > 350 && b > g) return BallColor.PURPLE;
            else if (g > 300) return BallColor.GREEN;
            return BallColor.NONE;
        } else if (sensorIndex == 1) { // backLeft
            if (b > 225  && b > g) return BallColor.PURPLE;
            else if (g > 350) return BallColor.GREEN;
            return BallColor.NONE;
        } else { // sensorIndex == 2 -> backRight
            if (b > 175  && b > g) return BallColor.PURPLE;
            if (g > 250) return BallColor.GREEN;
            return BallColor.NONE;
        }
    }


    public void clearSilo(int index) {
        if (index >= 0 && index < silos.length) {
            silos[index] = BallColor.NONE;
        }
    }

    private int angleToSilo(double angle) {
        for (int i = 0; i < 3; i++) {
            double error = smallestAngleError(SILO_ANGLES[i], angle);
            if (Math.abs(error) < ANGLE_TOLERANCE_COLOR) return i;
        }
        return -1;
    }

    public void enableSensorUpdates() {
        sensorUpdatesEnabled = true;
    }

    public void disableSensorUpdates() {
        sensorUpdatesEnabled = false;
    }

    public void goToSilo(int index) {
        switch (index) {
            case 0: goToSilo1(); break;
            case 1: goToSilo2(); break;
            case 2: goToSilo3(); break;
        }
    }

    public void initSilos() {
        silo1 = normalize(Parameters.spinLocation - 120);
        silo2 = normalize(Parameters.spinLocation);
        silo3 = normalize(Parameters.spinLocation + 120);

        // keep your existing angle array updated (not used for direct silo commands)
        SILO_ANGLES = new double[]{
                silo1,   // SILO_1
                silo2,   // SILO_2
                silo3    // SILO_3
        };
    }

}
