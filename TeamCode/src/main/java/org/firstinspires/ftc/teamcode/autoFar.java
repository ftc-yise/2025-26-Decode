package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.yise.Hood;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.ShotPatternManager;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.lifter;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.Parameters;


@Config
@Autonomous(name = "autoFar", group = "Linear Opmode")

public class autoFar extends LinearOpMode {
    // default alliance is red
    public Turret.turretAlliance alliance = Turret.turretAlliance.RED;
    // this will hold the trajectoryAction we select based on alliance color
    public Action trajectoryActionChosen;
    public DcMotor intake;
    public class ShootAction implements Action {

        private final ShooterExecutionClass autoShoot;
        private final ShooterClass shooter;
        private final Spindexer spin;
        private final Hood hood;
        private final lifter lifter;
        private final Turret turret;
        private final ShotPatternManager patternMgr;

        private final ElapsedTime runtime = new ElapsedTime();

        public ShootAction() {
            patternMgr = new ShotPatternManager();
            shooter = new ShooterClass(hardwareMap);
            spin = new Spindexer(hardwareMap);
            hood = new Hood(hardwareMap);
            lifter = new lifter(hardwareMap);
            autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap, lifter);
            turret = new Turret(hardwareMap, alliance, telemetry);

            autoShoot.setPatternManager(patternMgr);
            runtime.reset();
        }

        @Override
        public boolean run(@NonNull com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {

            turret.autoMode();
            turret.mode = Turret.turretMode.AUTO;

            hood.update();
            spin.sampleSensorsNow();
            spin.update();
            autoShoot.update();
            lifter.update();

            if (!autoShoot.isBusy()) {
                shooter.update(false, true, false);
                hood.setTarget(35);
                autoShoot.startCycle();
            }

            // run for 3 seconds instead of 9 (faster auto)
            return runtime.seconds() < 3;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // default alliance is red, only overwrite if blue was chosen in game values
        if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = Turret.turretAlliance.BLUE;
        }

        // instantiate drive class (MecanumDrive) at a particular pose.
        Pose2d initialPose = null;
        if (alliance == Turret.turretAlliance.RED) {
            initialPose = new Pose2d(63, -12, Math.toRadians(0));
        }else if (alliance == Turret.turretAlliance.BLUE) {
            initialPose = new Pose2d(63, -12, Math.toRadians(90));
        }
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        //Intake intake = new Intake(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");

        // we build our trajectories during initialization to avoid wasting time during auto
        // tab one is for red
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .stopAndAdd(new ShootAction())
                .strafeToLinearHeading(new Vector2d(33, 12), Math.toRadians(90))
                //turn on intake
                .strafeToLinearHeading(new Vector2d(33,50), Math.toRadians(90))
                //turn of intake
                .strafeToLinearHeading(new Vector2d(61,12), Math.toRadians(90))
                .stopAndAdd(new ShootAction())
                .strafeToLinearHeading(new Vector2d(11,12), Math.toRadians(90))
                //turn on intake
                .strafeToLinearHeading(new Vector2d(11,50), Math.toRadians(90))
                //turn of intake
                .strafeToLinearHeading(new Vector2d(61,12), Math.toRadians(90))
                .stopAndAdd(new ShootAction());
        // tab two is for blue
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                //.stopAndAdd(new ShootAction())
                .strafeToConstantHeading(new Vector2d(33, -12));
                //turn on intake
               /* .strafeToLinearHeading(new Vector2d(33,-50), Math.toRadians(270))
                //turn of intake
                .strafeToLinearHeading(new Vector2d(61,-12), Math.toRadians(270))
                .stopAndAdd(new ShootAction())
                .strafeToLinearHeading(new Vector2d(11,-12), Math.toRadians(270))
                //turn on intake
                .strafeToLinearHeading(new Vector2d(11, -50), Math.toRadians(270))
                //turn of intake
                .strafeToLinearHeading(new Vector2d(61,-12), Math.toRadians(270))
                .stopAndAdd(new ShootAction());
*/
        waitForStart();
        if (isStopRequested()) return;

        // set our trajectoryAction based on alliance color
        if (alliance == Turret.turretAlliance.RED) {
            trajectoryActionChosen = tab1.build();
        } else if (alliance == Turret.turretAlliance.BLUE) {
            trajectoryActionChosen = tab2.build();
        }

        intake.setPower(1);
        Actions.runBlocking(trajectoryActionChosen);
        intake.setPower(0);
    }
}