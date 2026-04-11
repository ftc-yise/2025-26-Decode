package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.Hood;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.ShotPatternManager;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.lifter;

import java.util.Objects;

@Config
@Autonomous(name = "autoClose", group = "Linear Opmode")

public class autoClose extends LinearOpMode {
    // default alliance is red
    public String alliance = "RED";
    // this will hold the trajectoryAction we select based on alliance color
    public DcMotor intake;
    public Pose2d initialPose;
    Turret.turretAlliance alliance2 = Turret.turretAlliance.RED;
    private final ElapsedTime runtime = new ElapsedTime();
    public Action traj_1 = null;
    public Action traj_2 = null;
    public Action traj_3 = null;
    public Action traj_4 = null;
    private ShotPatternManager.ShotPattern patternFromTag(int tagId) {
        switch (tagId) {
            case 21: return ShotPatternManager.ShotPattern.GPP;
            case 22: return ShotPatternManager.ShotPattern.PGP;
            case 23: return ShotPatternManager.ShotPattern.PPG;
            default: return null;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        ShotPatternManager patternMgr = new ShotPatternManager();
        ShooterClass shooter = new ShooterClass(hardwareMap);
        Spindexer spin = new Spindexer(hardwareMap);
        Hood hood = new Hood(hardwareMap);
        lifter lifter = new lifter(hardwareMap);
        ShooterExecutionClass autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap, lifter);
        intake = hardwareMap.get(DcMotor.class, "intake");

        if (Parameters.allianceColor == Parameters.Color.RED) {
            alliance = "RED";
            alliance2 = Turret.turretAlliance.RED;
        } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = "BLUE";
            alliance2 = Turret.turretAlliance.BLUE;
        }
        Turret turret = new Turret(hardwareMap, alliance2, telemetry);
        turret.limelight.pipelineSwitch(1);


        runtime.reset();


        Pose2d initialPose = null;

        if (Objects.equals(alliance, "RED")) {
            initialPose = new Pose2d(-64, 43, Math.toRadians(270));
        }else if (Objects.equals(alliance, "BLUE")) {
            initialPose = new Pose2d(-64, -43, Math.toRadians(90));
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        drive.localizer.setPose(initialPose);

        // RED Alliance Trajectories
        TrajectoryActionBuilder r_tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-36,33))
                .turn(Math.toRadians(180));

        TrajectoryActionBuilder r_tab2 = r_tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(-17,35));

        TrajectoryActionBuilder r_tab3 = r_tab2.endTrajectory().fresh()
                .strafeTo(new Vector2d(-17,50))
                .strafeTo(new Vector2d(-36,33));

        TrajectoryActionBuilder r_tab4 = r_tab3.endTrajectory().fresh()
                .strafeTo(new Vector2d(-16,36));

        //  BLUE Alliance Trajectories
        TrajectoryActionBuilder b_tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-36,-33))
                .turn(Math.toRadians(180));

        TrajectoryActionBuilder b_tab2 = b_tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(-17,-35));

        TrajectoryActionBuilder b_tab3 = b_tab2.endTrajectory().fresh()
                .strafeTo(new Vector2d(-17,-50))
                .strafeTo(new Vector2d(-36,-33));

        TrajectoryActionBuilder b_tab4 = b_tab3.endTrajectory().fresh()
                .strafeTo(new Vector2d(-16,-36));

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
            telemetry.update();
        }

        if (isStopRequested()) return;

        // set our trajectoryAction based on alliance color
        if (Objects.equals(alliance, "RED")) {
            traj_1 = r_tab1.build();
            traj_2 = r_tab2.build();
            traj_3 = r_tab3.build();
            traj_4 = r_tab4.build();
            turret.limelight.pipelineSwitch(4);
        } else if (Objects.equals(alliance, "BLUE")) {
            traj_1 = b_tab1.build();
            traj_2 = b_tab2.build();
            traj_3 = b_tab3.build();
            traj_4 = b_tab4.build();
            turret.limelight.pipelineSwitch(2);
        }

        Actions.runBlocking(traj_1);//drive to shooting spot

        //shoot initial 3 balls
        runtime.reset();
        //==================================================
        //Close shooting method
        //==================================================
        while (opModeIsActive() && (runtime.seconds() < 9)) {
            // false,false,true = FAR false,true,false = CLOSE
            // hood.setTarget(50) = FAR hood.setTarget(35) = CLOSE

            //==================================================
            //updates
            //==================================================
            turret.autoMode();
            turret.mode = Turret.turretMode.AUTO;
            hood.update();
            spin.sampleSensorsNow();
            spin.update();
            autoShoot.update();
            lifter.update();

            if (canSatisfyPattern(patternMgr, spin) && !autoShoot.isBusy()) {
                shooter.update(false, true, false);
                hood.setTarget(35);
                autoShoot.startCycle();
            } else {
                shooter.update(false, true, false);
                hood.setTarget(35);
                if (!autoShoot.forceShooting && !autoShoot.isBusy()) {
                    autoShoot.startForcedCycle();
                }

            }
        }

        Actions.runBlocking(traj_2);//drive to closest spike mark
        intake.setPower(1);
        Actions.runBlocking(traj_3);//drive to end of spike mark then shooting spot
        intake.setPower(0);
        //shoot 3 balls
        // Step #:  Shooting
        runtime.reset();
        //==================================================
        //Close shooting method
        //==================================================
        while (opModeIsActive() && (runtime.seconds() < 9)) {
            // false,false,true = FAR false,true,false = CLOSE
            // hood.setTarget(50) = FAR hood.setTarget(35) = CLOSE

            //==================================================
            //updates
            //==================================================
            turret.autoMode();
            turret.mode = Turret.turretMode.AUTO;
            hood.update();
            spin.sampleSensorsNow();
            spin.update();
            autoShoot.update();
            lifter.update();

            if (canSatisfyPattern(patternMgr, spin) && !autoShoot.isBusy()) {
                shooter.update(false, true, false);
                hood.setTarget(35);
                autoShoot.startCycle();
            } else {
                shooter.update(false, true, false);
                hood.setTarget(35);
                if (!autoShoot.forceShooting && !autoShoot.isBusy()) {
                    autoShoot.startForcedCycle();
                }

            }
        }
        Actions.runBlocking(traj_4);//drive to park
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