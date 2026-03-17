package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveClass {

    // -----------------------------
    // Telemetry output variables
    // -----------------------------
    private double t_rawX, t_rawY, t_rawTurn;
    private double t_txField, t_tyField, t_rotationCmd;
    private double t_robotX, t_robotY;
    private double t_lf, t_rf, t_lb, t_rb;
    private double t_headingDeg;

    private double t_appliedLF, t_appliedRF, t_appliedLB, t_appliedRB;
    private boolean t_fieldOrientedEnabled;
    private boolean t_dpadActive;
    private double t_dt;
    private double t_currentSpeed;
    private SparkFunOTOS.Pose2D t_pose;

    // -----------------------------
    // Hardware
    // -----------------------------
    private final DcMotor leftFrontDrive;
    private final DcMotor leftBackDrive;
    private final DcMotor rightFrontDrive;
    private final DcMotor rightBackDrive;
    private final SparkFunOTOS myOtos;

    // -----------------------------
    // Speed control
    // -----------------------------
    private final double fullSpeed = 1.0;
    private final double slowSpeed = 0.45;
    private double currentSpeed = fullSpeed;
    private boolean yPreviouslyPressed = false;

    // -----------------------------
    // Input shaping
    // -----------------------------
    private final double transDeadband = 0.06;
    private final double rotDeadband = 0.08;

    // Lower than 3.0 so the robot responds sooner near center
    private final double stickExpo = 1.75;

    // Small strafing compensation
    private final double strafeScale = 1.06;

    // -----------------------------
    // Motion smoothing
    // -----------------------------
    // Faster rise than before, but still not jerky
    private final double accelUpStep = 0.12;

    private double lastLF = 0.0, lastRF = 0.0, lastLB = 0.0, lastRB = 0.0;

    // -----------------------------
    // Heading trim (not full PID)
    // -----------------------------
    private boolean headingHoldValid = false;
    private double holdHeadingDeg = 0.0;
    private final double headingKp = 0.012;
    private final double maxHeadingTrim = 0.18;

    // -----------------------------
    // Periodic wheel compensation
    // -----------------------------
    // These are intentionally mild starting values.
    // Tune them from logs, do not jump too high at first.
    private final double biasTicksPerCycle = 560.0;
    private final double biasA1 = 0.035;
    private final double biasB1 = 0.020;
    private final double biasA2 = 0.012;
    private final double biasB2 = 0.006;
    private final double biasGain = 0.65;

    // Small output deadband
    private final double outputDeadband = 0.02;

    private long lastLoopTime = System.nanoTime();

    public DriveClass(HardwareMap hardwareMap) {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "otos");

        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Keep braking always on so stopping feels crisp
        setBrakeMode(true);

        myOtos.resetTracking();
        t_pose = myOtos.getPosition();

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void handleSpeedToggle(Gamepad gamepad) {
        if (gamepad.y && !yPreviouslyPressed) {
            currentSpeed = (currentSpeed == fullSpeed) ? slowSpeed : fullSpeed;
            yPreviouslyPressed = true;
        } else if (!gamepad.y) {
            yPreviouslyPressed = false;
        }
    }

    public void stopAllMotors() {
        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);

        lastLF = 0.0;
        lastRF = 0.0;
        lastLB = 0.0;
        lastRB = 0.0;
    }

    public void updateMotors(Gamepad gamepad, boolean reverse) {
        double dir = reverse ? -1.0 : 1.0;

        long now = System.nanoTime();
        t_dt = (now - lastLoopTime) / 1e9;
        lastLoopTime = now;
        if (t_dt <= 0.0 || t_dt > 0.2) {
            t_dt = 0.02;
        }

        SparkFunOTOS.Pose2D pose = myOtos.getPosition();
        if (pose == null) {
            pose = new SparkFunOTOS.Pose2D();
        }
        t_pose = pose;

        double headingDeg = pose.h;
        t_headingDeg = headingDeg;

        // Raw driver inputs
        double rawX = gamepad.left_stick_x * dir;       // strafe
        double rawY = -gamepad.left_stick_y * dir;      // forward positive
        double rawTurn = -gamepad.right_stick_x * dir;  // rotation

        // Deadband
        double x = applyDeadband(rawX, transDeadband);
        double y = applyDeadband(rawY, transDeadband);
        double turn = applyDeadband(rawTurn, rotDeadband);

        // Shape inputs
        x = shape(x, stickExpo) * strafeScale;
        y = shape(y, stickExpo);
        turn = shape(turn, stickExpo);

        // D-pad override for clean linear motion
        boolean dpadActive = false;
        double dpadX = 0.0;
        double dpadY = 0.0;

        if (gamepad.dpad_up)    { dpadY = 1.0; dpadActive = true; }
        if (gamepad.dpad_down)  { dpadY = -1.0; dpadActive = true; }
        if (gamepad.dpad_right) { dpadX = 1.0; dpadActive = true; }
        if (gamepad.dpad_left)  { dpadX = -1.0; dpadActive = true; }

        if (dpadActive) {
            x = dpadX;
            y = dpadY;
            turn = 0.0;
        }

        // Small heading trim only when translating and not actively turning
        boolean driving = Math.abs(x) + Math.abs(y) > 0.05;
        boolean noTurnCommand = Math.abs(turn) < 0.02;

        if (driving && noTurnCommand) {
            if (!headingHoldValid) {
                holdHeadingDeg = headingDeg;
                headingHoldValid = true;
            }

            double headingError = normalizeDeg(holdHeadingDeg - headingDeg);
            double headingTrim = clamp(headingKp * headingError, -maxHeadingTrim, maxHeadingTrim);
            turn += headingTrim;
        } else {
            headingHoldValid = false;
            holdHeadingDeg = headingDeg;
        }

        // Robot-centric mecanum mix
        double lf = y + x + turn;
        double rf = y - x - turn;
        double lb = y - x + turn;
        double rb = y + x - turn;

        // Normalize
        double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));
        lf /= max;
        rf /= max;
        lb /= max;
        rb /= max;

        // Scale by speed mode
        lf *= currentSpeed;
        rf *= currentSpeed;
        lb *= currentSpeed;
        rb *= currentSpeed;

        // Add periodic wheel compensation
        double translationMag = clamp(Math.hypot(x, y), 0.0, 1.0);
        if (translationMag > 0.03 && Math.abs(turn) < 0.35) {
            double bias = periodicBias() * translationMag;

            // Apply as a differential left-vs-right correction
            lf -= bias;
            lb -= bias;
            rf += bias;
            rb += bias;

            // Re-normalize after compensation
            max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }

        // Asymmetric slew: slow on accel, fast on decel
        lf = slewAsymmetric(lastLF, lf);
        rf = slewAsymmetric(lastRF, rf);
        lb = slewAsymmetric(lastLB, lb);
        rb = slewAsymmetric(lastRB, rb);

        // Small output deadband
        if (Math.abs(lf) < outputDeadband) lf = 0.0;
        if (Math.abs(rf) < outputDeadband) rf = 0.0;
        if (Math.abs(lb) < outputDeadband) lb = 0.0;
        if (Math.abs(rb) < outputDeadband) rb = 0.0;

        lastLF = lf;
        lastRF = rf;
        lastLB = lb;
        lastRB = rb;

        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);

        // -----------------------------
        // Telemetry capture for logger
        // -----------------------------
        t_rawX = rawX;
        t_rawY = rawY;
        t_rawTurn = rawTurn;

        // Robot-oriented: keep logger fields meaningful and consistent
        t_txField = x;
        t_tyField = y;
        t_rotationCmd = turn;

        t_robotX = x;
        t_robotY = y;

        t_lf = lf;
        t_rf = rf;
        t_lb = lb;
        t_rb = rb;

        t_appliedLF = lf;
        t_appliedRF = rf;
        t_appliedLB = lb;
        t_appliedRB = rb;

        t_fieldOrientedEnabled = false;
        t_dpadActive = dpadActive;
        t_currentSpeed = currentSpeed;
    }

    public void setAutoPower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public DriveTelemetry getDriveTelemetry() {
        DriveTelemetry d = new DriveTelemetry();
        d.rawX = t_rawX;
        d.rawY = t_rawY;
        d.rawTurn = t_rawTurn;

        d.tx_field = t_txField;
        d.ty_field = t_tyField;
        d.rotationCmd = t_rotationCmd;

        d.robotX = t_robotX;
        d.robotY = t_robotY;

        d.lf = t_lf;
        d.rf = t_rf;
        d.lb = t_lb;
        d.rb = t_rb;

        d.headingDeg = t_headingDeg;
        d.pose = t_pose;

        d.currentSpeed = t_currentSpeed;

        d.appliedLF = t_appliedLF;
        d.appliedRF = t_appliedRF;
        d.appliedLB = t_appliedLB;
        d.appliedRB = t_appliedRB;

        d.fieldOrientedEnabled = t_fieldOrientedEnabled;
        d.dpadActive = t_dpadActive;
        d.dt = t_dt;

        return d;
    }

    // -----------------------------
    // Helpers
    // -----------------------------
    private void setBrakeMode(boolean enabled) {
        DcMotor.ZeroPowerBehavior mode = enabled ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        leftFrontDrive.setZeroPowerBehavior(mode);
        rightFrontDrive.setZeroPowerBehavior(mode);
        leftBackDrive.setZeroPowerBehavior(mode);
        rightBackDrive.setZeroPowerBehavior(mode);
    }

    private double applyDeadband(double val, double threshold) {
        return Math.abs(val) > threshold ? val : 0.0;
    }

    private double shape(double v, double expo) {
        if (expo == 1.0) return v;
        double s = Math.signum(v);
        return s * Math.pow(Math.abs(v), expo);
    }

    private double slewAsymmetric(double last, double target) {
        if (Math.abs(target) < Math.abs(last)) {
            return target; // fast stop
        }
        double delta = target - last;
        if (Math.abs(delta) <= accelUpStep) return target;
        return last + Math.signum(delta) * accelUpStep;
    }

    private double periodicBias() {
        double avgTicks =
                (Math.abs(leftFrontDrive.getCurrentPosition()) +
                        Math.abs(leftBackDrive.getCurrentPosition()) +
                        Math.abs(rightFrontDrive.getCurrentPosition()) +
                        Math.abs(rightBackDrive.getCurrentPosition())) / 4.0;

        double phase = 2.0 * Math.PI * ((avgTicks % biasTicksPerCycle) / biasTicksPerCycle);

        double raw =
                biasA1 * Math.sin(phase) +
                        biasB1 * Math.cos(phase) +
                        biasA2 * Math.sin(2.0 * phase) +
                        biasB2 * Math.cos(2.0 * phase);

        return clamp(raw * biasGain, -0.10, 0.10);
    }

    private double normalizeDeg(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg < -180.0) deg += 360.0;
        return deg;
    }


    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // -----------------------------
    // Telemetry container
    // -----------------------------
    public static class DriveTelemetry {
        public double rawX, rawY, rawTurn;
        public double tx_field, ty_field, rotationCmd;
        public double robotX, robotY;
        public double lf, rf, lb, rb;
        public double headingDeg;
        public SparkFunOTOS.Pose2D pose;

        public double currentSpeed;

        public double appliedLF, appliedRF, appliedLB, appliedRB;
        public boolean fieldOrientedEnabled;
        public boolean dpadActive;
        public double dt;
    }
}