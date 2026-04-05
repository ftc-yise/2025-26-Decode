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

import java.util.Objects;

@Config
@Autonomous(name = "autoClose", group = "Linear Opmode")

public class autoClose extends LinearOpMode {
    // default alliance is red
    public String alliance = "RED";
    // this will hold the trajectoryAction we select based on alliance color
    public Action trajectoryActionChosen;
    public DcMotor intake;
    public Pose2d initialPose;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = null;
        if (Objects.equals(alliance, "RED")) {
            initialPose = new Pose2d(-64, 43, Math.toRadians(270));
        }else if (Objects.equals(alliance, "BLUE")) {
            initialPose = new Pose2d(-64, -43, Math.toRadians(180));
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        drive.localizer.setPose(initialPose);

        //positions 1-4 are for red side only
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-36,33))
                .turn(Math.toRadians(180));

        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(-17,35));

        TrajectoryActionBuilder tab3 =tab2.endTrajectory().fresh()
                .strafeTo(new Vector2d(-17,50))
                .strafeTo(new Vector2d(-36,33));

        TrajectoryActionBuilder tab4 =tab3.endTrajectory().fresh()
                .strafeTo(new Vector2d(-16,36));

        TrajectoryActionBuilder tab5 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-36,-33));

        waitForStart();
        if (isStopRequested()) return;

        // set our trajectoryAction based on alliance color
        if (Objects.equals(alliance, "RED")) {
            trajectoryActionChosen = tab1.build();
            trajectoryActionChosen = tab2.build();
            trajectoryActionChosen = tab3.build();
            trajectoryActionChosen = tab4.build();
        } else if (Objects.equals(alliance, "BLUE")) {
            trajectoryActionChosen = tab5.build();
        }

        Action traj_1 = tab1.build();
        Action traj_2 = tab2.build();
        Action traj_3 = tab3.build();
        Action traj_4 = tab4.build();
        Action traj_5 = tab5.build();

        //red side
        Actions.runBlocking(traj_1);//drive to shooting spot
        //shoot initial 3 balls
        Actions.runBlocking(traj_2);//drive to closest spike mark
        //intake.setPower(1);
        Actions.runBlocking(traj_3);//drive to end of spike mark then shooting spot
        //intake.setPower(0);
        //shoot 3 balls
        Actions.runBlocking(traj_4);//drive to park

        //blue side
        Actions.runBlocking(traj_5);//drive to shooting spot

    }
}