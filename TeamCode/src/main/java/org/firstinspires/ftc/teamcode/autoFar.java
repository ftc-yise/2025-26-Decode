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

@Config
@Autonomous(name = "autoFar", group = "Linear Opmode")

public class autoFar extends LinearOpMode {
    public DcMotor intake;
    public Pose2d initialPose;

    @Override
    public void runOpMode() throws InterruptedException {
        initialPose = new Pose2d(-60, 40, Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        intake = hardwareMap.get(DcMotor.class, "intake");

        //all positions are for red side only
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-50,37));

        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .turn(Math.toRadians(180))
                .strafeTo(new Vector2d(0,0))
                .turn(Math.toRadians(180));

        waitForStart();
        if (isStopRequested()) return;

        Action traj_1 = tab1.build();
        Action traj_2 = tab2.build();

        Actions.runBlocking(traj_1); //drive to shooting spot
        //shoot initial 3 balls
        //Actions.runBlocking(traj_2); //drive to closest spike mark
        intake.setPower(1);
        //drive to end of spike mark then shooting spot
        intake.setPower(0);
        //shoot 3 balls
        //drive to park

    }


}