package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * DriveClass handles all drivetrain movement for the robot.
 *
 * It converts gamepad stick input into mecanum wheel motor power,
 * applies speed scaling, and stores telemetry so other parts of the
 * program can read what the drive system is doing.
 */
public class DriveClass {

    // ---------------- Telemetry ----------------
    // These values are copied every loop so the rest of the robot code
    // can inspect the latest drive input/output values without directly
    // reading the motors or gamepad again.
    private double t_rawX, t_rawY, t_rawTurn;
    private double t_lf, t_rf, t_lb, t_rb;
    private double t_headingDeg;
    private double t_dt;
    private boolean t_dpadActive;
    private double t_currentSpeed;
    private SparkFunOTOS.Pose2D t_pose;

    // ---------------- Hardware ----------------
    // Drivetrain motors and OTOS odometry sensor.
    private final DcMotorEx leftFrontDrive;
    private final DcMotorEx leftBackDrive;
    private final DcMotorEx rightFrontDrive;
    private final DcMotorEx rightBackDrive;
    private final SparkFunOTOS myOtos;

    // ---------------- Speed Modes ----------------
    // The driver can toggle between full speed and slow mode.
    private final double fullSpeed = 1.0;
    private final double slowSpeed = 0.45;
    private double currentSpeed = fullSpeed;

    // Used to detect the rising edge of the Y button so the speed mode
    // only toggles once per press instead of every loop while held down.
    private boolean yPreviouslyPressed = false;

    // Used to measure loop timing for telemetry/logging purposes.
    private long lastLoopTime = System.nanoTime();

    private int t_encLF, t_encRF, t_encLB, t_encRB;
    private double t_velLF, t_velRF, t_velLB, t_velRB;

    /**
     * Constructor:
     * - maps hardware
     * - sets motor directions
     * - enables brake mode
     * - resets odometry
     * - initializes motors to run without encoders
     */
    public DriveClass(HardwareMap hardwareMap) {

        // SparkFun OTOS odometry sensor used for robot pose tracking
        myOtos = hardwareMap.get(SparkFunOTOS.class, "otos");

        // Drivetrain motor mapping from the Robot Controller configuration
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "LeftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "RightBackDrive");

        // Reverse left motors so positive power drives all wheels in the same physical direction
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Hold position when power is zero instead of coasting
        setBrakeMode(true);

        // Reset odometry so the pose starts from a known reference point
        myOtos.resetTracking();
        t_pose = myOtos.getPosition();

        // The drivetrain is controlled directly by motor power, not motor encoders
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ---------------- Speed Toggle ----------------
    /**
     * Toggles between full speed and reduced speed when the driver presses Y.
     *
     * This method uses edge detection so the speed only changes once per press.
     */
    public void handleSpeedToggle(Gamepad gamepad) {
        if (gamepad.y && !yPreviouslyPressed) {
            currentSpeed = (currentSpeed == fullSpeed) ? slowSpeed : fullSpeed;
            yPreviouslyPressed = true;
        } else if (!gamepad.y) {
            yPreviouslyPressed = false;
        }
    }

    // ---------------- Main Drive ----------------
    /**
     * Reads driver input and updates the four drivetrain motors.
     *
     * Parameters:
     * - gamepad: the current gamepad state
     * - reverse: if true, flips the driving direction
     *
     * This method:
     * 1. reads the sticks
     * 2. computes mecanum wheel mixing
     * 3. normalizes power values
     * 4. applies speed scaling
     * 5. writes motor power
     * 6. stores telemetry snapshots
     */
    public void updateMotors(Gamepad gamepad, boolean reverse) {

        // Reverse drive direction when requested
        double dir = reverse ? -1.0 : 1.0;

        // Measure loop delta time in seconds
        long now = System.nanoTime();
        t_dt = (now - lastLoopTime) / 1e9;
        lastLoopTime = now;

        // Read the current pose from OTOS if available
        SparkFunOTOS.Pose2D pose = myOtos.getPosition();
        if (pose != null) {
            t_pose = pose;
            t_headingDeg = pose.h;
        }

        // Raw stick input:
        // left stick x = strafe
        // left stick y = forward/back
        // right stick x = turn
        double x = gamepad.left_stick_x * dir;
        double y = -gamepad.left_stick_y * dir;
        double turn = -gamepad.right_stick_x * dir;

        // Save raw inputs for telemetry
        t_rawX = x;
        t_rawY = y;
        t_rawTurn = turn;

        // Basic mecanum drive mix:
        // combine forward/back, strafe, and turn into each wheel output
        double lf = y + x - turn;
        double rf = y - x + turn;
        double lb = y - x - turn;
        double rb = y + x + turn;

        // Normalize all wheel powers so none exceed magnitude 1.0
        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.max(Math.abs(lb), Math.abs(rb)))));

        lf /= max;
        rf /= max;
        lb /= max;
        rb /= max;

        // Apply driver-selected speed scaling
        lf *= currentSpeed;
        rf *= currentSpeed;
        lb *= currentSpeed;
        rb *= currentSpeed;

        // Send power to each motor
        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);

        t_encLF = leftFrontDrive.getCurrentPosition();
        t_encRF = rightFrontDrive.getCurrentPosition();
        t_encLB = leftBackDrive.getCurrentPosition();
        t_encRB = rightBackDrive.getCurrentPosition();

        t_velLF = leftFrontDrive.getVelocity();
        t_velRF = rightFrontDrive.getVelocity();
        t_velLB = leftBackDrive.getVelocity();
        t_velRB = rightBackDrive.getVelocity();

        // Store applied power values for telemetry
        t_lf = lf;
        t_rf = rf;
        t_lb = lb;
        t_rb = rb;

        // Store extra state for logging/telemetry
        t_currentSpeed = currentSpeed;
        t_dpadActive = false;
    }

    // ---------------- Stop ----------------
    /**
     * Immediately stops all drivetrain motors.
     */
    public void stopAllMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    // ---------------- Helpers ----------------
    /**
     * Sets zero-power behavior for all drivetrain motors.
     *
     * BRAKE holds position more firmly.
     * FLOAT allows the robot to coast.
     */
    private void setBrakeMode(boolean enabled) {
        DcMotor.ZeroPowerBehavior mode =
                enabled ? DcMotor.ZeroPowerBehavior.BRAKE :
                        DcMotor.ZeroPowerBehavior.FLOAT;

        leftFrontDrive.setZeroPowerBehavior(mode);
        rightFrontDrive.setZeroPowerBehavior(mode);
        leftBackDrive.setZeroPowerBehavior(mode);
        rightBackDrive.setZeroPowerBehavior(mode);
    }

    // ---------------- Telemetry Packet ----------------
    /**
     * Returns a snapshot of the drivetrain state for telemetry and logging.
     *
     * This keeps the rest of the code from needing direct access to the motors.
     */
    public DriveTelemetry getDriveTelemetry() {
        DriveTelemetry d = new DriveTelemetry();
        d.encLF = t_encLF;
        d.encRF = t_encRF;
        d.encLB = t_encLB;
        d.encRB = t_encRB;

        d.velLF = t_velLF;
        d.velRF = t_velRF;
        d.velLB = t_velLB;
        d.velRB = t_velRB;

        d.rawX = t_rawX;
        d.rawY = t_rawY;
        d.rawTurn = t_rawTurn;

        d.lf = t_lf;
        d.rf = t_rf;
        d.lb = t_lb;
        d.rb = t_rb;

        d.headingDeg = t_headingDeg;
        d.pose = t_pose;

        d.currentSpeed = t_currentSpeed;
        d.dpadActive = t_dpadActive;
        d.dt = t_dt;

        // Keep compatibility with your logger
        d.tx_field = t_rawX;
        d.ty_field = t_rawY;
        d.rotationCmd = t_rawTurn;

        d.robotX = t_rawX;
        d.robotY = t_rawY;

        d.appliedLF = t_lf;
        d.appliedRF = t_rf;
        d.appliedLB = t_lb;
        d.appliedRB = t_rb;

        // This drive class uses robot-centric drive, not field-oriented drive
        d.fieldOrientedEnabled = false;

        return d;
    }

    /**
     * Data container for drive telemetry.
     *
     * This is intentionally a plain structure so other classes can easily read
     * drivetrain state without needing to know how the drive code works.
     */
    public static class DriveTelemetry {
        public int encLF, encRF, encLB, encRB;
        public double velLF, velRF, velLB, velRB;
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