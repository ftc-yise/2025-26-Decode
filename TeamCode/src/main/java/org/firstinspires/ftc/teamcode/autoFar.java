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

import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import java.util.Objects;

@Config
@Autonomous(name = "autoFar", group = "Linear Opmode")

public class autoFar extends LinearOpMode {
    // default alliance is red
    public Turret.turretAlliance alliance = Turret.turretAlliance.RED;
    // this will hold the trajectoryAction we select based on alliance color
    public Action trajectoryActionChosen;
    public DcMotor intake;
    public Pose2d initialPose;
    public Action traj_1 = null;
    public Action traj_2 = null;
    public Action traj_3 = null;
    public Action traj_4 = null;
    public Action traj_5 = null;

    @Override
    public void runOpMode() throws InterruptedException {

        if (Parameters.allianceColor == Parameters.Color.RED) {
            alliance = Turret.turretAlliance.RED;
        } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = Turret.turretAlliance.BLUE;
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
                .strafeTo(new Vector2d(37,26));

        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(37,51))
                .strafeTo(new Vector2d(63,19));

        TrajectoryActionBuilder tab3 =tab2.endTrajectory().fresh()
                .strafeTo(new Vector2d(13,26));

        TrajectoryActionBuilder tab4 =tab3.endTrajectory().fresh()
                .strafeTo(new Vector2d(13,51))
                .strafeTo(new Vector2d(63,19));

        TrajectoryActionBuilder tab5 =tab4.endTrajectory().fresh()
                .strafeTo(new Vector2d(62,38));

        //blue side now
        TrajectoryActionBuilder tab6 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(26,-37));

        TrajectoryActionBuilder tab7 =tab6.endTrajectory().fresh()
                .strafeTo(new Vector2d(37,-51))
                .strafeTo(new Vector2d(63,-19));

        TrajectoryActionBuilder tab8= tab7.endTrajectory().fresh()
                .strafeTo(new Vector2d(13,-26));

        TrajectoryActionBuilder tab9 = tab8.endTrajectory().fresh()
                .strafeTo(new Vector2d(13,-51))
                .strafeTo(new Vector2d(63,-19));

        TrajectoryActionBuilder tab10 =tab9.endTrajectory().fresh()
                .strafeTo(new Vector2d(62,-38));


        waitForStart();
        if (isStopRequested()) return;

        // set our trajectoryAction based on alliance color
        if (alliance == Turret.turretAlliance.RED) {
            traj_1 = tab1.build();
            traj_2 = tab2.build();
            traj_3 = tab3.build();
            traj_4 = tab4.build();
            traj_5 = tab5.build();
        } else if (alliance == Turret.turretAlliance.BLUE) {
            traj_1 = tab6.build();
            traj_2 = tab7.build();
            traj_3 = tab8.build();
            traj_4 = tab9.build();
            traj_5 = tab10.build();
        }

        //shoot 3 balls
        Actions.runBlocking(traj_1);//drive to first spike mark
        //intake.setPower(1);
        Actions.runBlocking(traj_2);//drive to end of spike mark then shooting spot
        //intake.setPower(0)
        //shoot 3 balls
        Actions.runBlocking(traj_3);//drive to 2nd spike mark
        //intake.setPower(1);
        Actions.runBlocking(traj_4);//drive to end of spike mark then shooting spot
        //intake.setPower(0)
        //shoot 3 balls
        Actions.runBlocking(traj_5);//drive to park

    }
}