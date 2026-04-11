package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.Hood;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.ShotPatternManager;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.lifter;

import java.util.Objects;

@Config
@Autonomous(name = "autoFarLoadingZone", group = "Linear Opmode")

public class autoFarLoadingZone extends LinearOpMode {
    // default alliance is red
    public Turret.turretAlliance alliance = Turret.turretAlliance.RED;
    // this will hold the trajectoryAction we select based on alliance color
    public DcMotor intake;
    public Pose2d initialPose;
    private CRServo walleft = null;
    private CRServo wallright = null;
    Turret.turretAlliance alliance2 = Turret.turretAlliance.RED;
    private final ElapsedTime runtime = new ElapsedTime();
    // Top color sensors
    private ColorSensor middleT = null;
    private ColorSensor backLeftT = null;
    private ColorSensor backRightT = null;

    // Bottom color sensors
    private ColorSensor middleB = null;
    private ColorSensor backLeftB = null;
    private ColorSensor backRightB = null;
    private ShotPatternManager.ShotPattern patternFromTag(int tagId) {
        switch (tagId) {
            case 21: return ShotPatternManager.ShotPattern.GPP;
            case 22: return ShotPatternManager.ShotPattern.PGP;
            case 23: return ShotPatternManager.ShotPattern.PPG;
            default: return null;
        }
    }
    // this will hold the trajectoryAction we select based on alliance color
    public Action trajectoryActionChosen;
    public Action traj_1 = null;
    public Action traj_2 = null;
    public Action traj_3 = null;

    @Override
    public void runOpMode() throws InterruptedException {

        ShotPatternManager patternMgr = new ShotPatternManager();
        ShooterClass shooter = new ShooterClass(hardwareMap);
        Spindexer spin = new Spindexer(hardwareMap);
        Hood hood = new Hood(hardwareMap);
        lifter lifter = new lifter(hardwareMap);
        ShooterExecutionClass autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap, lifter);
        intake = hardwareMap.get(DcMotor.class, "intake");
        // Top color sensors
        middleT = hardwareMap.get(ColorSensor.class, "middlecolorsensorT");
        backLeftT = hardwareMap.get(ColorSensor.class, "BLcolorsensorT");
        backRightT = hardwareMap.get(ColorSensor.class, "BRcolorsensorT");

        // Bottom color sensors
        middleB = hardwareMap.get(ColorSensor.class, "middlecolorsensorB");
        backLeftB = hardwareMap.get(ColorSensor.class, "BLcolorsensorB");
        backRightB = hardwareMap.get(ColorSensor.class, "BRcolorsensorB");

        Turret turret = new Turret(hardwareMap, alliance2, telemetry);
        turret.limelight.pipelineSwitch(1);
        walleft = hardwareMap.get(CRServo.class, "WallWheelLeft");
        wallright = hardwareMap.get(CRServo.class, "WallWheelRight");

        // Reverse one wall wheel so both spin in the intended feed direction
        wallright.setDirection(CRServo.Direction.REVERSE);

        if (Parameters.allianceColor == Parameters.Color.RED) {
            alliance = Turret.turretAlliance.RED;
            alliance2 = Turret.turretAlliance.RED;
        } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = Turret.turretAlliance.BLUE;
            alliance2 = Turret.turretAlliance.BLUE;
        }


        Pose2d initialPose = null;

        if (Objects.equals(alliance, Turret.turretAlliance.RED)) {
            initialPose = new Pose2d(65, 19, Math.toRadians(90));
        }else if (Objects.equals(alliance, Turret.turretAlliance.BLUE)) {
            initialPose = new Pose2d(65, -19, Math.toRadians(270));
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        drive.localizer.setPose(initialPose);

        //red side
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(62,58));
        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(62,19));

        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()
                .strafeTo(new Vector2d(62,38));

        //blue side now
        TrajectoryActionBuilder tab4 =drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(62,-58));

        TrajectoryActionBuilder tab5 =tab4.endTrajectory().fresh()
                .strafeTo(new Vector2d(62,-19));

        TrajectoryActionBuilder tab6 = tab5.endTrajectory().fresh()
                .strafeTo(new Vector2d(62,-38));

        while (!isStarted() && !isStopRequested()) {
            turret.mode = Turret.turretMode.AUTO;
            int tagId = turret.getID();
            ShotPatternManager.ShotPattern p = patternFromTag(tagId);
            if (p != null) {
                patternMgr.clear();
                patternMgr.addPattern(p.sequence);
            }
            spin.sampleSensorsNow();
            spin.update();
            telemetry.addLine("=== SILOS ===");
            Spindexer.BallColor[] silos = spin.siloColors;
            for (int i = 0; i < silos.length; i++) {
                String label = "Silo " + (i+1);
                // Highlight the current silo
                telemetry.addData(label, silos[i]);
            }

            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Mode", turret.mode);
            telemetry.addData("Power", turret.turretPower);
            telemetry.addData("pose", turret.getPose());
            telemetry.addData("id", turret.getID());
            telemetry.addData("pipeline", turret.limelight.getStatus().getPipelineIndex());

            telemetry.addLine();
            telemetry.addLine();
            telemetry.addLine("=== COLOR SENSORS ===");
            telemetry.addLine("MiddleTop");
            telemetry.addData("Blue", middleT.blue());
            telemetry.addData("Red", middleT.red());
            telemetry.addData("Green", middleT.green());

            telemetry.addLine();
            telemetry.addLine("Back Left Top");
            telemetry.addData("Blue", backLeftT.blue());
            telemetry.addData("Red", backLeftT.red());
            telemetry.addData("Green", backLeftT.green());

            telemetry.addLine();
            telemetry.addLine("Back Right Top");
            telemetry.addData("Blue", backRightT.blue());
            telemetry.addData("Red", backRightT.red());
            telemetry.addData("Green", backRightT.green());

            telemetry.addLine();
            telemetry.addLine("Middle Bottom");
            telemetry.addData("Blue", middleB.blue());
            telemetry.addData("Red", middleB.red());
            telemetry.addData("Green", middleB.green());

            telemetry.addLine();
            telemetry.addLine("Back Left Bottom");
            telemetry.addData("Blue", backLeftB.blue());
            telemetry.addData("Red", backLeftB.red());
            telemetry.addData("Green", backLeftB.green());

            telemetry.addLine();
            telemetry.addLine("Back Right Bottom");
            telemetry.addData("Blue", backRightB.blue());
            telemetry.addData("Red", backRightB.red());
            telemetry.addData("Green", backRightB.green());


            telemetry.update();
            turret.limelight.pipelineSwitch(1);
        }
        if (isStopRequested()) return;

        // set our trajectoryAction based on alliance color
        if (alliance == Turret.turretAlliance.RED) {
            traj_1 = tab1.build();
            traj_2 = tab2.build();
            traj_3 = tab3.build();
        } else if (alliance == Turret.turretAlliance.BLUE) {
            traj_1 = tab4.build();
            traj_2 = tab5.build();
            traj_3 = tab6.build();
        }

        runtime.reset();
        //==================================================
        //Close shooting method
        //==================================================
        while (opModeIsActive() && (runtime.seconds() < 14)) {

            if (alliance2 == Turret.turretAlliance.RED) {
                turret.limelight.pipelineSwitch(4);
            } else if (alliance2 == Turret.turretAlliance.BLUE) {
                turret.limelight.pipelineSwitch(2);
            }
            // false,false,true = FAR false,true,false = CLOSE
            // hood.setTarget(50) = FAR hood.setTarget(35) = CLOSE

            //==================================================
            //updates
            //==================================================
            turret.autoMode();
            turret.mode = Turret.turretMode.AUTO;
            hood.update();
            spin.update();
            autoShoot.update();
            lifter.update();

            if (canSatisfyPattern(patternMgr, spin) && !autoShoot.isBusy()) {
                shooter.update(false, false, true);
                hood.setTarget(45);
                autoShoot.startCycle();
            } else {
                shooter.update(false, false, true);
                hood.setTarget(45);
                if (!autoShoot.forceShooting && !autoShoot.isBusy()) {
                    autoShoot.startForcedCycle();
                }

            }
        }
        intake.setPower(1);
        walleft.setPower(1);
        wallright.setPower(1);
        Actions.runBlocking(traj_1);//drive to loading zone and back
        intake.setPower(-1);
        Actions.runBlocking(traj_2);
        runtime.reset();
        //==================================================
        //Close shooting method
        //==================================================
        while (opModeIsActive() && (runtime.seconds() < 9)) {

            if (alliance2 == Turret.turretAlliance.RED) {
                turret.limelight.pipelineSwitch(4);
            } else if (alliance2 == Turret.turretAlliance.BLUE) {
                turret.limelight.pipelineSwitch(2);
            }
            // false,false,true = FAR false,true,false = CLOSE
            // hood.setTarget(50) = FAR hood.setTarget(35) = CLOSE

            //==================================================
            //updates
            //==================================================
            turret.autoMode();
            turret.mode = Turret.turretMode.AUTO;
            hood.update();
            spin.update();
            autoShoot.update();
            lifter.update();

            if (canSatisfyPattern(patternMgr, spin) && !autoShoot.isBusy()) {
                shooter.update(false, false, true);
                hood.setTarget(45);
                autoShoot.startCycle();
            } else {
                shooter.update(false, false, true);
                hood.setTarget(45);
                if (!autoShoot.forceShooting && !autoShoot.isBusy()) {
                    autoShoot.startForcedCycle();
                }

            }
        }
        Actions.runBlocking(traj_3);
    }
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
}
