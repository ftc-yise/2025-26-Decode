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

import org.firstinspires.ftc.teamcode.yise.Parameters;
import java.util.Objects;

@Config
@Autonomous(name = "autoFarPark", group = "Linear Opmode")

public class autoFarPark extends LinearOpMode {
    // default alliance is red
    public String alliance = "RED";
    // this will hold the trajectoryAction we select based on alliance color
    public Action trajectoryActionChosen;
    public DcMotor intake;
    public Pose2d initialPose;

    @Override
    public void runOpMode() throws InterruptedException {

        if (Parameters.allianceColor == Parameters.Color.RED) {
            alliance = "RED";
        } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = "BLUE";
        }

        Pose2d initialPose = null;

        if (Objects.equals(alliance, "RED")) {
            initialPose = new Pose2d(19, 65, Math.toRadians(90));
        }else if (Objects.equals(alliance, "BLUE")) {
            initialPose = new Pose2d(19, -65, Math.toRadians(270));
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        drive.localizer.setPose(initialPose);

        //red side
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(35,36));


        //blue side now
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(35,-36));



        waitForStart();
        if (isStopRequested()) return;

        // set our trajectoryAction based on alliance color
        if (Objects.equals(alliance, "RED")) {
            trajectoryActionChosen = tab1.build();

        } else if (Objects.equals(alliance, "BLUE")) {
            trajectoryActionChosen = tab2.build();

        }

        Action traj_1 = tab1.build();
        Action traj_2 = tab2.build();

        //red side
        //shoot 3 balls
        Actions.runBlocking(traj_1);//park


        //blue side
        //shoot 3 balls
        Actions.runBlocking(traj_2);//park

    }
}