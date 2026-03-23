package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.yise.DriveClass;
import org.firstinspires.ftc.teamcode.yise.Hood;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.lifter;
import org.firstinspires.ftc.teamcode.yise.Ledclass;
import org.firstinspires.ftc.teamcode.yise.ShotPatternManager;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Main teleop program for the BallBot robot.
 *
 * This class ties together:
 * - driving
 * - shooting
 * - turret control
 * - spindexer / silo logic
 * - intake and wall wheels
 * - hood and lifter positioning
 * - LED feedback
 * - data logging to CSV
 *
 * The goal is to centralize all robot behavior in one driver-controlled mode.
 */
@TeleOp(name = "BallBot", group = "Ball Bot")
public class BallBotMainDrive extends LinearOpMode {

    // =========================================================
    // TURRET SNAP / HOMING STATE MACHINE
    // =========================================================

    /**
     * These states control the turret when it is doing an automatic motion:
     * - moving left or right to a limit
     * - "pushing" after reaching a limit to settle/mechanically press
     * - returning to home position
     * - running a homing routine using the limit switch
     */
    private enum SnapState {
        INACTIVE,
        SNAPPING_LEFT,
        PUSHING_LEFT,
        SNAPPING_RIGHT,
        PUSHING_RIGHT,
        GOING_HOME,
        HOMING_ROUTINE
    }

    // Current turret snap state
    private SnapState currentSnapState = SnapState.INACTIVE;

    // Timers used for homing and turret snap timing
    private ElapsedTime homingTimer = new ElapsedTime();
    private ElapsedTime snapTimer = new ElapsedTime();

    // Prevents a mode toggle from triggering repeatedly while a button is held
    private boolean modeTogglePressed = false;

    // Left turret encoder limit and home target
    private static final int LEFT_LIMIT = -1370;
    public boolean once = true;
    private static final int CENTER_TARGET = -685;

    // Distance threshold used when deciding whether the turret is "close enough" to home
    private static final int TOLERANCE = 10;

    // Tracks whether the robot is currently in a shooting sequence
    private boolean shooting = false;

    // =========================================================
    // CSV LOGGING FIELDS
    // These store the most recent timing/average values so the CSV
    // can keep a stable record even if a getter returns NaN.
    // =========================================================

    private double csvPress1 = 0.0;
    private double csvPress2 = 0.0;
    private double csvPress3 = 0.0;
    private double csvPressAvg = 0.0;

    private double csvAvg1 = 0.0;
    private double csvAvg2 = 0.0;
    private double csvAvg3 = 0.0;
    private double csvOverall = 0.0;

    private double csvMoveToSiloSec = 0.0;
    private double csvSpinWaitSec = 0.0;
    private double csvSpinUpSec = 0.0;
    private double csvFireLiftUpSec = 0.0;
    private double csvFireLiftDownSec = 0.0;

    private int csvTargetSilo = -1;
    private boolean Limit;
    private double csvTargetAngle = 0.0;

    // =========================================================
    // HARDWARE REFERENCES
    // =========================================================

    // Intake motor
    private DcMotor intake = null;

    // Wall wheels that help feed balls into the robot / shooting path
    private CRServo walleft = null;
    private CRServo wallright = null;

    // Additional motor used by the robot (appears to be a "foot" mechanism)
    private DcMotor foot = null;

    // Turret limit switch
    public DigitalChannel limit;

    // Top color sensors
    private ColorSensor middleT = null;
    private ColorSensor backLeftT = null;
    private ColorSensor backRightT = null;

    // Bottom color sensors
    private ColorSensor middleB = null;
    private ColorSensor backLeftB = null;
    private ColorSensor backRightB = null;

    // LED indicators used to show robot state to drivers
    public Ledclass led1;
    public Ledclass led2;
    public Ledclass led3;

    // =========================================================
    // BUTTON EDGE DETECTION
    // These flags let us detect the moment a button was first pressed
    // rather than reacting continuously every loop.
    // =========================================================

    private boolean prevA = false;
    private boolean prevX = false;
    boolean a;
    boolean x;

    // Timers used for runtime, LED timeout, and log write interval
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime ledTimer = new ElapsedTime();
    private final ElapsedTime logTimer = new ElapsedTime();

    // CSV logging output
    private PrintWriter logWriter = null;
    private String logFilePath = null;

    // How often to write a row to the CSV
    private double logInterval = 0.05;

    // Cached runtime in seconds
    public double time = runtime.seconds();

    /**
     * Converts AprilTag IDs into a known shot pattern.
     * Different tags correspond to different ordered silo sequences.
     */
    private ShotPatternManager.ShotPattern patternFromTag(int tagId) {
        switch (tagId) {
            case 21: return ShotPatternManager.ShotPattern.GPP;
            case 22: return ShotPatternManager.ShotPattern.PGP;
            case 23: return ShotPatternManager.ShotPattern.PPG;
            default: return null;
        }
    }

    // Currently active pattern from vision / tag detection
    private ShotPatternManager.ShotPattern activePattern = null;

    boolean firstRun = true;

    // Alliance color used by the turret and vision logic
    Turret.turretAlliance alliance = Turret.turretAlliance.RED;

    @Override
    public void runOpMode() throws InterruptedException {

        // =========================================================
        // SUBSYSTEM CONSTRUCTION
        // Each subsystem owns its own hardware and internal logic.
        // =========================================================

        DriveClass drive = new DriveClass(hardwareMap);
        ShotPatternManager patternMgr = new ShotPatternManager();
        ShooterClass shooter = new ShooterClass(hardwareMap);
        Spindexer spin = new Spindexer(hardwareMap);
        Hood hood = new Hood(hardwareMap);
        lifter lifter = new lifter(hardwareMap);
        ShooterExecutionClass autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap, lifter);

        // ShooterExecutionClass uses ShotPatternManager to decide the order of shots
        autoShoot.setPatternManager(patternMgr);

        // Parameter object appears to carry global robot configuration
        Parameters parem = new Parameters();
        parem.autonomous = Parameters.AUTONOMOUS.NO;

        // Pick alliance-dependent turret mode
        if (Parameters.allianceColor == Parameters.Color.RED) {
            alliance = Turret.turretAlliance.RED;
        } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = Turret.turretAlliance.BLUE;
        }

        // Turret subsystem with alliance and telemetry access
        Turret turret = new Turret(hardwareMap, alliance, telemetry);

        // =========================================================
        // HARDWARE MAPPING
        // Connect Java variables to the robot controller configuration names.
        // =========================================================

        walleft = hardwareMap.get(CRServo.class, "WallWheelLeft");
        wallright = hardwareMap.get(CRServo.class, "WallWheelRight");
        foot = hardwareMap.get(DcMotor.class, "foot");

        // Reverse one wall wheel so both spin in the intended feed direction
        wallright.setDirection(CRServo.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");

        // Top color sensors
        middleT = hardwareMap.get(ColorSensor.class, "middlecolorsensorT");
        backLeftT = hardwareMap.get(ColorSensor.class, "BLcolorsensorT");
        backRightT = hardwareMap.get(ColorSensor.class, "BRcolorsensorT");

        // Bottom color sensors
        middleB = hardwareMap.get(ColorSensor.class, "middlecolorsensorB");
        backLeftB = hardwareMap.get(ColorSensor.class, "BLcolorsensorB");
        backRightB = hardwareMap.get(ColorSensor.class, "BRcolorsensorB");

        // LED objects tied to named outputs
        led1 = new Ledclass(hardwareMap, "led1");
        led2 = new Ledclass(hardwareMap, "led2");
        led3 = new Ledclass(hardwareMap, "led3");

        // Turret limit switch
        limit = hardwareMap.get(DigitalChannel.class, "limit");

        // Start the hood and lift in a safe resting state
        hood.stop();
        lifter.setDown();

        // Wait for the referee / driver station to start the match
        waitForStart();
        runtime.reset();

        // =========================================================
        // MAIN TELEOP LOOP
        // Runs continuously while the op mode is active.
        // =========================================================
        while (opModeIsActive()) {
            time = runtime.seconds();

            // Alliance can be changed globally elsewhere, so refresh it every loop
            if (Parameters.allianceColor == Parameters.Color.RED) {
                alliance = Turret.turretAlliance.RED;
            } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
                alliance = Turret.turretAlliance.BLUE;
            }

            // -----------------------------------------------------
            // DRIVE CONTROL
            // -----------------------------------------------------
            drive.handleSpeedToggle(gamepad1);
            drive.updateMotors(gamepad1, false);

            // -----------------------------------------------------
            // PATTERN-AWARE SHOOTING LOGIC
            //
            // A and X are treated as shot triggers:
            // - A = one shooting configuration
            // - X = another shooting configuration
            //
            // The code first tries to use a shot pattern if the current
            // silo colors can satisfy it. If not, it falls back to a
            // forced cycle while the button is held.
            // -----------------------------------------------------

            // Detect first press of A or X
            if (!a && !x && gamepad2.a && !prevX && !prevA) {
                a = true;
                shooting = true;
            } else if (!a && !x && gamepad2.x && !prevX && !prevA) {
                x = true;
                shooting = true;
            }

            // Reset flags when buttons are released and enough shots have completed
            if (!gamepad2.a && autoShoot.shots > 2) {
                a = false;
                shooting = false;
            }
            if (!gamepad2.x && autoShoot.shots > 2) {
                x = false;
                shooting = false;
            }

            // Once all shots are done and no buttons are pressed, reset shot count
            if (!a && !x && !gamepad2.x && !prevX && !prevA && !gamepad2.a && autoShoot.shots >= 3){
                autoShoot.shots = 0;
            }

            // -----------------------------
            // A button: high-goal shot mode
            // -----------------------------
            if (a && !prevA) {
                // On the rising edge, attempt a pattern-aware cycle
                if (canSatisfyPattern(patternMgr, spin) && !autoShoot.isBusy()) {
                    shooter.update(false, false, true);
                    hood.setTarget(60);
                    autoShoot.startCycle();
                } else {
                    // If the current pattern cannot be satisfied, begin forced shooting
                    shooter.update(false, false, true);
                    hood.setTarget(60);
                    if (!autoShoot.forceShooting && !autoShoot.isBusy()) {
                        autoShoot.startForcedCycle();
                    }
                }
            } else if (a) {
                // While held, keep forced shooting alive
                shooter.update(false, false, true);
                hood.setTarget(60);
                if (!autoShoot.forceShooting && !autoShoot.isBusy()) {
                    autoShoot.startForcedCycle();
                }
            }

            // -----------------------------
            // X button: lower-goal shot mode
            // -----------------------------
            if (x && !prevX) {
                if (canSatisfyPattern(patternMgr, spin) && !autoShoot.isBusy()) {
                    shooter.update(false, true, false);
                    hood.setTarget(80);
                    autoShoot.startCycle();
                } else {
                    shooter.update(false, true, false);
                    hood.setTarget(80);
                    if (!autoShoot.forceShooting && !autoShoot.isBusy()) {
                        autoShoot.startForcedCycle();
                    }
                }
            } else if (x) {
                shooter.update(false, true, false);
                hood.setTarget(80);
                if (!autoShoot.forceShooting && !autoShoot.isBusy()) {
                    autoShoot.startForcedCycle();
                }
            }

            // If neither A nor X is held, stop forced shooting
            if (!a && !x && autoShoot.forceShooting) {
                autoShoot.stopForcedCycle();
            }

            // Update the shooter state machine once every loop
            autoShoot.update();

            // Save current edge states for next loop iteration
            prevA = a;
            prevX = x;

            // -----------------------------------------------------
            // VISION / PATTERN DETECTION
            //
            // gamepad1.y appears to switch the limelight into a tag-read
            // pipeline. When not pressed, the pipeline is set according to
            // alliance color.
            // -----------------------------------------------------
            if (gamepad1.y) {
                turret.limelight.pipelineSwitch(2);
                turret.runto(430); // keep this per your request

                int tagId = turret.getID();
                ShotPatternManager.ShotPattern p = patternFromTag(tagId);
                if (p != null) {
                    patternMgr.clear();
                    patternMgr.addPattern(p.sequence);
                }
            } else {
                if (Parameters.allianceColor == Parameters.Color.RED) {
                    alliance = Turret.turretAlliance.RED;
                    turret.limelight.pipelineSwitch(4);
                } else {
                    alliance = Turret.turretAlliance.BLUE;
                    turret.limelight.pipelineSwitch(3);
                }
            }

            // -----------------------------------------------------
            // LED STATUS
            //
            // Blue: turret sees a tag associated with target IDs 20/24
            // Green: actively shooting
            // Off: default idle state after a timeout
            // -----------------------------------------------------
            if ((turret.getID() == 20 || turret.getID() == 24) && !shooting) {
                led1.setBlue();
                led2.setBlue();
                led3.setBlue();
            } else if (time > 1) {
                led1.setOff();
                led2.setOff();
                led3.setOff();
                ledTimer.reset();
            } else if (shooting){
                led1.setGreen();
                led2.setGreen();
                led3.setGreen();
            }

            // -----------------------------------------------------
            // INTAKE + WALL WHEELS
            //
            // Right trigger: intake forward
            // Left trigger: intake reverse
            // Right bumper: partial reverse / eject behavior
            //
            // The wall wheels are driven to help manage ball flow.
            // -----------------------------------------------------
            if (gamepad1.right_trigger > 0.75 && !shooting) {
                intake.setPower(1);
                led1.setRed();
                led2.setRed();
                led3.setRed();
                walleft.setPower(1);
                wallright.setPower(1);
                spin.startManualCycle();
                once = true;

            } else if (gamepad1.left_trigger > .75 && !shooting) {
                intake.setPower(-1);
                walleft.setPower(-1);
                wallright.setPower(-1);
                once = true;
            } else if (gamepad1.right_bumper && !shooting) {
                intake.setPower(-.6);
                walleft.setPower(-1);
                wallright.setPower(-1);
                once = true;
            } else {
                // Idle intake state
                intake.setPower(0);
                walleft.setPower(0.25);
                wallright.setPower(0.25);

                // When the shooter is not busy and triggers are released,
                // allow the spindexer and lifter to return to a safe default.
                if (!autoShoot.isBusy() && gamepad1.right_trigger < 0.75 && gamepad1.left_trigger < 0.75) {
                    if (gamepad2.right_stick_button && gamepad2.left_stick_button){
                        Parameters.spinLocation = spin.getTelemetry().currentAngle;
                        spin.initSilos();
                    }

                    if (once) {
                        lifter.setDown();
                        once = false;
                        spin.setNeutral();
                        shooting = false;
                    }
                }
            }

            // -----------------------------------------------------
            // HOOD / FOOT / SPINDEXER MANUAL POSITIONS
            // -----------------------------------------------------

            // These d-pad controls are for direct operator override
            if (gamepad1.dpad_down) {
                // foot.setPower(0.557);
                // spin.goToPose(0);
            }
            else if (gamepad1.dpad_left) {
                spin.goToPose(0);
            }
            else if (gamepad1.dpad_up) {
                foot.setPower(-0.25);
                // spin.goToPose(0.5);
            } else if (gamepad1.dpad_right) {
                foot.setPower(-0.25);
                spin.goToPose(1);
            }

            // Operator can manually raise/lower the hood and lifter when not shooting
            if (gamepad2.dpad_up && !autoShoot.isBusy()) {
                hood.setTarget(60);
                lifter.setUp();
                shooting = true;
            }
            else if (gamepad2.dpad_down && !autoShoot.isBusy()) {
                hood.setTarget(80);
                lifter.setDown();
                shooting = true;
            }

            // =========================================================
            // TURRET STATE MACHINE
            //
            // This block manages turret behavior:
            // - manual mode
            // - auto mode
            // - homing routine
            // - snapping to left/right limits
            // - returning to center
            // =========================================================

            // 1. Toggle turret mode with Y
            if (gamepad2.y && !modeTogglePressed) {
                turret.changeMode();
                currentSnapState = SnapState.INACTIVE;
                modeTogglePressed = true;
            }
            if (!gamepad2.y) {
                modeTogglePressed = false;
            }

            // 2. Share button starts the homing routine
            if (gamepad2.share) {
                currentSnapState = SnapState.HOMING_ROUTINE;
                homingTimer.reset();
            }

            // 3. Manual control only applies when turret is in MANUAL mode
            if (turret.mode == Turret.turretMode.MANUAL) {
                double turretManualTrigger = (gamepad2.left_trigger - gamepad2.right_trigger) * -.8;

                // Direct trigger-based turret motion
                if (Math.abs(turretManualTrigger) > 0.05) {
                    currentSnapState = SnapState.INACTIVE;
                    turret.manualControl(turretManualTrigger);
                }
                // Both bumpers pressed = return turret to center
                else if (gamepad2.left_bumper && gamepad2.right_bumper) {
                    currentSnapState = SnapState.GOING_HOME;
                }
                // Right bumper = snap right
                else if (gamepad2.right_bumper && currentSnapState != SnapState.SNAPPING_RIGHT && currentSnapState != SnapState.PUSHING_RIGHT) {
                    currentSnapState = SnapState.SNAPPING_RIGHT;
                }
                // Left bumper = snap left
                else if (gamepad2.left_bumper && currentSnapState != SnapState.SNAPPING_LEFT && currentSnapState != SnapState.PUSHING_LEFT) {
                    currentSnapState = SnapState.SNAPPING_LEFT;
                }
            }

            // 4. Execute the currently selected turret state
            if (turret.mode == Turret.turretMode.AUTO) {
                turret.autoMode();
                turret.mode = Turret.turretMode.AUTO;
            }
            else if (currentSnapState == SnapState.HOMING_ROUTINE) {
                // Homing: move toward the limit switch until it is reached
                if (gamepad2.share) {
                    if (!turret.limit.getState()) {
                        // Limit switch hit: reset encoder and move gently
                        turret.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        turret.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        turret.manualControl(0.4);
                    } else {
                        // While approaching the switch, reduce power gradually
                        double curvePower = 0.9 - (homingTimer.seconds() * 0.33);
                        turret.manualControl(Range.clip(curvePower, 0.4, 0.9));
                    }
                } else {
                    currentSnapState = SnapState.INACTIVE;
                    turret.stop();
                }
            }
            else if (currentSnapState == SnapState.GOING_HOME) {
                // Drive turret back to a known center encoder position
                int currentPos = turret.turret.getCurrentPosition();
                int error = CENTER_TARGET - currentPos;

                if (Math.abs(error) > TOLERANCE) {
                    double homePower = (error > 0) ? 1.0 : -1.0;
                    if (Math.abs(error) < 200) homePower *= 0.4;
                    turret.manualControl(homePower);
                } else {
                    turret.stop();
                    currentSnapState = SnapState.INACTIVE;
                }
            }
            else if (currentSnapState == SnapState.SNAPPING_RIGHT) {
                // Run right until the limit switch is hit
                turret.manualControl(1.0);
                if (!turret.limit.getState()) {
                    snapTimer.reset();
                    currentSnapState = SnapState.PUSHING_RIGHT;
                }
            }
            else if (currentSnapState == SnapState.PUSHING_RIGHT) {
                // Briefly keep pushing after the switch is hit
                if (snapTimer.seconds() < 1.0) turret.manualControl(0.5);
                else {
                    turret.stop();
                    currentSnapState = SnapState.INACTIVE;
                }
            }
            else if (currentSnapState == SnapState.SNAPPING_LEFT) {
                // Drive left until the left encoder limit is reached
                int currentPos = turret.turret.getCurrentPosition();
                turret.manualControl(-1.0);
                if (currentPos <= LEFT_LIMIT) {
                    snapTimer.reset();
                    currentSnapState = SnapState.PUSHING_LEFT;
                }
            }
            else if (currentSnapState == SnapState.PUSHING_LEFT) {
                if (snapTimer.seconds() < 1.0) turret.manualControl(-0.5);
                else {
                    turret.stop();
                    currentSnapState = SnapState.INACTIVE;
                }
            }
            else if (turret.mode == Turret.turretMode.MANUAL &&
                    currentSnapState == SnapState.INACTIVE &&
                    Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) <= 0.05) {
                turret.stop();
            }

            // =========================================================
            // CSV LOG FILE INITIALIZATION
            // Creates a timestamped CSV in /sdcard/FIRST and writes header row once.
            // =========================================================
            if (logWriter == null) {
                try {
                    File dir = new File("/sdcard/FIRST");
                    if (!dir.exists()) dir.mkdirs();

                    String timestamp = new SimpleDateFormat(
                            "yyyyMMdd_HHmmss",
                            Locale.US
                    ).format(new Date());

                    logFilePath = "/sdcard/FIRST/telem_log_" + timestamp + ".csv";
                    logWriter = new PrintWriter(new FileWriter(logFilePath));

                    // CSV header: every column is a named measurement/flag
                    logWriter.println(
                            "time_s," +
                                    "input_x,input_y,input_turn," +
                                    "trans_x,trans_y,rotation_cmd," +
                                    "lf,rf,lb,rb,total_power," +
                                    "enc_lf,enc_rf,enc_lb,enc_rb," +
                                    "vel_lf,vel_rf,vel_lb,vel_rb," +
                                    "applied_lf,applied_rf,applied_lb,applied_rb," +
                                    "drive_field_mode,drive_dpad_active,drive_dt,battery_volt," +
                                    "pose_x,pose_y,pose_h," +
                                    "sh_mode,sh_targetRPM,sh_currentRPM,sh_errorRPM,sh_volt,sh_pose,sh_spinupTime," +
                                    "turret_mode,turret_power,turret_pose,turret_id,turret_pipeline," +
                                    "lift_pose,lift_volt,lift_err,lift_mode,lift_up,lift_down," +
                                    "hood_mode,hood_volt,hood_angle,hood_target,hood_error,hood_power," +
                                    "shoot_state," +
                                    "spx_mode,spx_currentAngle,spx_targetAngle,spx_error,spx_appliedPower," +
                                    "spx_silo1,spx_silo2,spx_silo3," +
                                    "btn_a,btn_x,btn_g1_rt,btn_g1_lt,btn_g1_rb,btn_g2_y," +
                                    "press_shot1,press_shot2,press_shot3,press_avg," +
                                    "avg_shot1,avg_shot2,avg_shot3,avg_overall," +
                                    "action_target_silo,action_target_angle,move_to_silo_s,spin_wait_s,spin_up_s,fire_lift_up_s,fire_lift_down_s," +
                                    "limit_switch," +
                                    "silo_order,pattern,pattern_valid," +
                                    "mT_r,mT_g,mT_b,blT_r,blT_g,blT_b,brT_r,brT_g,brT_b," +
                                    "mB_r,mB_g,mB_b,blB_r,blB_g,blB_b,brB_r,brB_g,brB_b"
                    );

                    logWriter.flush();
                    logTimer.reset();

                    telemetry.addData("LOG", "Started");
                    telemetry.addData("File", logFilePath);
                    telemetry.update();

                    sleep(300); // Small pause to avoid log startup issues
                } catch (IOException e) {
                    telemetry.addData("LOG ERROR", e.getMessage());
                    telemetry.update();
                }
            }

            // Clear shot pattern manually with gamepad1 A
            if (gamepad1.a) {
                patternMgr.clear();
            }

            // =========================================================
            // SPINDEXER UPDATE
            // The spindexer samples sensors first, then runs its logic,
            // then exposes a telemetry snapshot.
            // =========================================================
            spin.sampleSensorsNow();
            spin.update();
            Spindexer.TelemetryPacket spina = spin.getTelemetry();

            // Drive telemetry snapshot
            DriveClass.DriveTelemetry d = drive.getDriveTelemetry();
            double appliedLF = d.appliedLF;
            double appliedRF = d.appliedRF;
            double appliedLB = d.appliedLB;
            double appliedRB = d.appliedRB;

            int encLF = d.encLF;
            int encRF = d.encRF;
            int encLB = d.encLB;
            int encRB = d.encRB;

            double velLF = d.velLF;
            double velRF = d.velRF;
            double velLB = d.velLB;
            double velRB = d.velRB;

            boolean driveFieldMode = d.fieldOrientedEnabled;
            boolean driveDpadActive = d.dpadActive;
            double driveDt = d.dt;

            // Update hood and pull telemetry snapshots
            hood.update();
            Hood.TelemetryPacket H = hood.getTelemetry();
            lifter.TelemetryPacket l = lifter.getTelemetry();

            // -----------------------------------------------------
            // TELEMETRY
            // -----------------------------------------------------
// SHOOTER
            /*
            shooter.updateTelemetry();
            ShooterClass.ShooterTelemetry s = shooter.getTelemetry();
            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Mode", s.mode);
            telemetry.addData("Target RPM", "%.2f", s.targetRPM);
            telemetry.addData("Current RPM", "%.1f", s.currentRPM);
            telemetry.addData("Error RPM", "%.1f", s.errorRPM);
            telemetry.addData("volt", s.motorPower);
            telemetry.addData("pose", s.pose);
            telemetry.addData("spin up time", s.spinupTimeSec);

// DRIVE
            // --- SHOOTER TIMING TELEMETRY ---
            double[] lastShotsTelemetry = autoShoot.getLastCompletedPressShotTimes();
            telemetry.addLine("=== SHOOT TIMING ===");
            telemetry.addData("Press Shot1", "%.3f", lastShotsTelemetry[0]);
            telemetry.addData("Press Shot2", "%.3f", lastShotsTelemetry[1]);
            telemetry.addData("Press Shot3", "%.3f", lastShotsTelemetry[2]);
            telemetry.addData("Press Avg", "%.3f", autoShoot.getLastCompletedPressAverage());

            telemetry.addLine("--- GLOBAL AVERAGES ---");
            telemetry.addData("Avg Shot1", "%.3f", autoShoot.getGlobalAverageShot1());
            telemetry.addData("Avg Shot2", "%.3f", autoShoot.getGlobalAverageShot2());
            telemetry.addData("Avg Shot3", "%.3f", autoShoot.getGlobalAverageShot3());
            telemetry.addData("Avg Overall", "%.3f", autoShoot.getGlobalOverallAverage());

            // optional: show which buttons are pressed (safety)
            telemetry.addLine("--- BUTTONS ---");
            telemetry.addData("A", "%b", a);
            telemetry.addData("X", "%b", x);
            telemetry.addData("G1 RT>", "%b", gamepad1.right_trigger > 0.75);
            telemetry.addData("G1 LT>", "%b", gamepad1.left_trigger > 0.75);
            telemetry.addData("G1 RB", "%b", gamepad1.right_bumper);
            telemetry.addData("G2 Y", "%b", gamepad2.y);

            telemetry.addLine("=== FIELD DRIVE ===");
            telemetry.addData("Speed Mode", d.currentSpeed);
            telemetry.addData("Heading (deg)", "%.2f", d.headingDeg);
            telemetry.addData("Inputs (raw)", "x:%.2f y:%.2f t:%.2f", d.rawX, d.rawY, d.rawTurn);
            telemetry.addData("FieldCmd (tx,ty)", "%.3f, %.3f", d.tx_field, d.ty_field);
            telemetry.addData("RobotCmd (rx,ry)", "%.3f, %.3f", d.robotX, d.robotY);
            telemetry.addData("Motor LF/RF/LB/RB", "%.3f / %.3f / %.3f / %.3f", d.lf, d.rf, d.lb, d.rb);
            telemetry.addData("Pose (x,y,h)", "%.2f, %.2f, %.2f", d.pose.x, d.pose.y, d.pose.h);

// SPINDEXER
            telemetry.addLine("=== SPINDEXER ===");
            telemetry.addData("Mode", spina.mode);
            telemetry.addData("Voltage", "%.3f", spina.voltage);
            telemetry.addData("Angle", "%.1f°", spina.currentAngle);
            telemetry.addData("Target", "%.1f°", spina.targetAngle);
            telemetry.addData("Error", "%.1f°", spina.angleError);
            telemetry.addData("Power", "%.2f", spina.appliedPower);
            telemetry.addData("A", a);
            telemetry.addData("X", x);
            telemetry.addData("shots", autoShoot.shots);

// TURRET
            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Mode", turret.mode);
            telemetry.addData("Power", turret.turretPower);
            telemetry.addData("pose", turret.getPose());
            telemetry.addData("id", turret.getID());
            telemetry.addData("pipeline", turret.limelight.getStatus().getPipelineIndex());

// SILOS
            telemetry.addLine("=== SILOS ===");
            Spindexer.BallColor[] silos = spina.siloColors;
            for (int i = 0; i < silos.length; i++) {
                String label = "Silo " + (i+1);
                // Highlight the current silo
                telemetry.addData(label, silos[i]);
            }

// COLOR SENSORS

            telemetry.addLine("=== COLOR SENSORS ===");
            telemetry.addLine("MiddleTop");
            telemetry.addData("Blue", middleT.blue());
            telemetry.addData("Red", middleT.red());
            telemetry.addData("Green", middleT.green());

            telemetry.addLine("Back Left Top");
            telemetry.addData("Blue", backLeftT.blue());
            telemetry.addData("Red", backLeftT.red());
            telemetry.addData("Green", backLeftT.green());

            telemetry.addLine("Back Right Top");
            telemetry.addData("Blue", backRightT.blue());
            telemetry.addData("Red", backRightT.red());
            telemetry.addData("Green", backRightT.green());

            telemetry.addLine("Middle Bottom");
            telemetry.addData("Blue", middleB.blue());
            telemetry.addData("Red", middleB.red());
            telemetry.addData("Green", middleB.green());

            telemetry.addLine("Back Left Bottom");
            telemetry.addData("Blue", backLeftB.blue());
            telemetry.addData("Red", backLeftB.red());
            telemetry.addData("Green", backLeftB.green());

            telemetry.addLine("Back Right Bottom");
            telemetry.addData("Blue", backRightB.blue());
            telemetry.addData("Red", backRightB.red());
            telemetry.addData("Green", backRightB.green());

//lift

            telemetry.addLine("=== LIFT ===");
            telemetry.addData("pose", l.position);
            telemetry.addData("volt", l.voltage);
            telemetry.addData("err", l.error);
            telemetry.addData("mode", l.mode);
            telemetry.addData("up", lifter.isUp());
            telemetry.addData("down", lifter.isDown());

//hood
            */
            telemetry.addLine("=== HOOD ===");
            telemetry.addData("Mode", H.mode);
            telemetry.addData("Voltage", "%.3f", H.voltage);
            telemetry.addData("Angle", "%.1f°", H.currentAngle);
            telemetry.addData("Target", "%.1f°", H.targetAngle);
            telemetry.addData("Error", "%.1f°", H.angleError);
            telemetry.addData("Power", "%.2f", H.appliedPower);
            /*


             */
            telemetry.addData("limiit", limit.getState());
            telemetry.update();



            // =========================================================
            // CSV LOGGING
            // Write one line every logInterval seconds.
            // =========================================================
            if (logWriter != null && logTimer.seconds() > 0.1) {

                double now = runtime.seconds();

                Spindexer.TelemetryPacket spx = spin.getTelemetry();

                // Silo order = which silo is currently "first" in the rotation
                String siloOrder = buildSiloOrder(spx);

                // Current queued pattern, shown as something like GREEN-PURPLE-GREEN
                String patternStr = buildPatternString(patternMgr);

                // Whether the current silo colors can actually satisfy the queued pattern
                boolean patternValid = canSatisfyPattern(patternMgr, spin);

                shooter.updateTelemetry();
                ShooterClass.ShooterTelemetry s = shooter.getTelemetry();

                String turretPose = String.valueOf(turret.getPose());
                String shPose = String.valueOf(s.pose);

                int turretPipeline = turret.limelight.getStatus().getPipelineIndex();
                int turretId = turret.getID();
                double turretPower = turret.turretPower;

                String shootState = autoShoot.getStateName();

                double totalPower =
                        Math.abs(d.lf) + Math.abs(d.rf) +
                                Math.abs(d.lb) + Math.abs(d.rb);

                double[] lastShots = autoShoot.getLastCompletedPressShotTimes();
                double lastPressAvg = autoShoot.getLastCompletedPressAverage();

                double globalAvg1 = autoShoot.getGlobalAverageShot1();
                double globalAvg2 = autoShoot.getGlobalAverageShot2();
                double globalAvg3 = autoShoot.getGlobalAverageShot3();
                double globalOverall = autoShoot.getGlobalOverallAverage();

                if (!Double.isNaN(lastShots[0])) csvPress1 = lastShots[0];
                if (!Double.isNaN(lastShots[1])) csvPress2 = lastShots[1];
                if (!Double.isNaN(lastShots[2])) csvPress3 = lastShots[2];
                if (!Double.isNaN(lastPressAvg)) csvPressAvg = lastPressAvg;

                if (!Double.isNaN(globalAvg1)) csvAvg1 = globalAvg1;
                if (!Double.isNaN(globalAvg2)) csvAvg2 = globalAvg2;
                if (!Double.isNaN(globalAvg3)) csvAvg3 = globalAvg3;
                if (!Double.isNaN(globalOverall)) csvOverall = globalOverall;

                csvTargetSilo = autoShoot.getLastTargetSiloIndex() + 1;
                csvTargetAngle = autoShoot.getLastTargetAngleDeg();
                csvMoveToSiloSec = autoShoot.getLastMoveToSiloSec();
                csvSpinWaitSec = autoShoot.getLastSpinWaitSec();
                csvSpinUpSec = autoShoot.getLastSpinUpSec();
                csvFireLiftUpSec = autoShoot.getLastFireLiftUpSec();
                csvFireLiftDownSec = autoShoot.getLastFireLiftDownSec();

                Limit = limit.getState();

                boolean btnA = a;
                boolean btnX = x;
                boolean g1Rt = gamepad1.right_trigger > 0.75;
                boolean g1Lt = gamepad1.left_trigger > 0.75;
                boolean g1Rb = gamepad1.right_bumper;
                boolean g2Y  = gamepad2.y;

                double batteryVolt = getBatteryVoltage();

                String row = String.join(",",
                        csv3(now),

                        csv(d.rawX), csv(d.rawY), csv(d.rawTurn),
                        csv(d.tx_field), csv(d.ty_field), csv(d.rotationCmd),

                        csv(d.lf), csv(d.rf), csv(d.lb), csv(d.rb), csv(totalPower),

                        csv(encLF), csv(encRF), csv(encLB), csv(encRB),
                        csv(velLF), csv(velRF), csv(velLB), csv(velRB),

                        csv(appliedLF), csv(appliedRF), csv(appliedLB), csv(appliedRB),
                        csv(driveFieldMode), csv(driveDpadActive), csv(driveDt), csv3(batteryVolt),

                        csv(d.pose.x), csv(d.pose.y), csv(d.pose.h),

                        csv(s.mode.toString()), csv(s.targetRPM), csv(s.currentRPM), csv(s.errorRPM), csv(s.motorPower), csv(s.pose), csv(s.spinupTimeSec),

                        csv(turret.mode.toString()), csv(turretPower), csv(turretPose), csv(turretId), csv(turretPipeline),

                        csv(l.position), csv(l.voltage), csv(l.error), csv(l.mode.toString()), csv(lifter.isUp()), csv(lifter.isDown()),

                        csv(H.mode.toString()), csv(H.voltage), csv(H.currentAngle), csv(H.targetAngle), csv(H.angleError), csv(H.appliedPower),

                        csv(shootState),

                        csv(spx.mode.toString()), csv(spx.currentAngle), csv(spx.targetAngle), csv(spx.angleError), csv(spx.appliedPower),

                        csv(String.valueOf(spx.siloColors[0])),
                        csv(String.valueOf(spx.siloColors[1])),
                        csv(String.valueOf(spx.siloColors[2])),

                        csv(btnA), csv(btnX), csv(g1Rt), csv(g1Lt), csv(g1Rb), csv(g2Y),

                        csv(csvPress1), csv(csvPress2), csv(csvPress3), csv(csvPressAvg),
                        csv(csvAvg1), csv(csvAvg2), csv(csvAvg3), csv(csvOverall),

                        csv(csvTargetSilo), csv(csvTargetAngle), csv(csvMoveToSiloSec), csv(csvSpinWaitSec), csv(csvSpinUpSec), csv(csvFireLiftUpSec), csv(csvFireLiftDownSec),

                        csv(Limit),

                        csv(siloOrder), csv(patternStr), csv(patternValid),

                        csv(middleT.red()), csv(middleT.green()), csv(middleT.blue()),
                        csv(backLeftT.red()), csv(backLeftT.green()), csv(backLeftT.blue()),
                        csv(backRightT.red()), csv(backRightT.green()), csv(backRightT.blue()),

                        csv(middleB.red()), csv(middleB.green()), csv(middleB.blue()),
                        csv(backLeftB.red()), csv(backLeftB.green()), csv(backLeftB.blue()),
                        csv(backRightB.red()), csv(backRightB.green()), csv(backRightB.blue())
                );

                logWriter.println(row);
                logWriter.flush();
                logTimer.reset();
            }
        } // end while opModeIsActive

        // =========================================================
        // CLEANUP
        // Close the CSV log file when the op mode ends.
        // =========================================================
        if (logWriter != null) {
            logWriter.flush();
            logWriter.close();
            telemetry.addData("LOG", "Saved");
            telemetry.addData("File", logFilePath);
            telemetry.update();
        }
    }

    /**
     * Checks whether the currently queued shot pattern can be completed
     * using the current silo colors without reusing a silo.
     *
     * This prevents starting a pattern-based shot cycle when the required
     * ball colors are not available in the right order.
     */
    private boolean canSatisfyPattern(ShotPatternManager patternMgr, Spindexer spin) {
        if (patternMgr == null || !patternMgr.hasShots()) return false;

        Spindexer.BallColor[] queued = patternMgr.snapshot();

        // Count how many shot slots are actually queued at the front of the pattern
        int queuedCount = 0;
        for (int i = 0; i < queued.length; i++) {
            if (queued[i] == Spindexer.BallColor.NONE) break;
            queuedCount++;
        }
        if (queuedCount == 0) return false;

        // Pull current silo colors from the spindexer
        Spindexer.TelemetryPacket spx = spin.getTelemetry();
        Spindexer.BallColor[] silos = spx.siloColors;
        boolean[] used = new boolean[silos.length];

        // For every queued color, find a matching unused silo
        for (int q = 0; q < queuedCount; q++) {
            Spindexer.BallColor desired = queued[q];
            int found = -1;

            for (int s = 0; s < silos.length; s++) {
                if (!used[s] && silos[s] == desired) {
                    found = s;
                    break;
                }
            }

            // If any required color cannot be found, the pattern cannot be satisfied
            if (found == -1) {
                return false;
            }

            used[found] = true;
        }

        return true;
    }

    /**
     * Returns the robot battery voltage.
     *
     * FTC hardware often exposes multiple voltage sensors, so this method
     * chooses the highest valid reading as the best estimate of battery voltage.
     */
    private double getBatteryVoltage() {
        double best = 0.0;
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            double v = vs.getVoltage();
            if (v > best) best = v;
        }
        return best;
    }

    private String csv(double v) {
        return Double.isFinite(v) ? String.format(Locale.US, "%.4f", v) : "";
    }

    private String csv3(double v) {
        return Double.isFinite(v) ? String.format(Locale.US, "%.3f", v) : "";
    }

    private String csv2(double v) {
        return Double.isFinite(v) ? String.format(Locale.US, "%.2f", v) : "";
    }

    private String csv(int v) {
        return Integer.toString(v);
    }

    private String csv(boolean v) {
        return v ? "true" : "false";
    }

    private String csv(String s) {
        if (s == null) return "\"\"";
        return "\"" + s.replace("\"", "\"\"") + "\"";
    }

    private String buildSiloOrder(Spindexer.TelemetryPacket spx) {
        double angle = ((spx.currentAngle % 360) + 360) % 360;
        int frontIndex = (int) Math.floor((angle + 60.0) / 120.0) % 3;

        int s1 = frontIndex + 1;
        int s2 = ((frontIndex + 1) % 3) + 1;
        int s3 = ((frontIndex + 2) % 3) + 1;

        return s1 + ">" + s2 + ">" + s3;
    }

    private String buildPatternString(ShotPatternManager mgr) {
        if (mgr == null) return "NONE";

        Spindexer.BallColor[] snap = mgr.snapshot();
        if (snap == null || snap.length == 0) return "NONE";

        StringBuilder sb = new StringBuilder();

        for (int i = 0; i < snap.length; i++) {
            Spindexer.BallColor color = snap[i];

            // stop at the first empty slot
            if (color == null || color == Spindexer.BallColor.NONE) break;

            if (sb.length() > 0) sb.append('-');
            sb.append(color.name());
        }

        return sb.length() == 0 ? "NONE" : sb.toString();
    }
}