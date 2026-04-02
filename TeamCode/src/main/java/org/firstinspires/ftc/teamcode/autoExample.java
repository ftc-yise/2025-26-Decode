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
@Autonomous(name = "autoExample", group = "Linear Opmode")

public class autoExample extends LinearOpMode {
    public DcMotor intake;
    public Pose2d initialPose;

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        intake = hardwareMap.get(DcMotor.class, "intake");

        initialPose = new Pose2d(0, 0, Math.toRadians(0));
        //TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
        //        .strafeTo(new Vector2d(18,18))
        //        .turn(Math.toRadians(180));

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(18,18));

        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .turn(Math.toRadians(180))
                .strafeTo(new Vector2d(0,0))
                .turn(Math.toRadians(180));

        waitForStart();
        if (isStopRequested()) return;

        Action traj_1 = tab1.build();
        Action traj_2 = tab2.build();

        intake.setPower(1);
        Actions.runBlocking(traj_1);
        intake.setPower(0);
        Actions.runBlocking(traj_2);
    }


}