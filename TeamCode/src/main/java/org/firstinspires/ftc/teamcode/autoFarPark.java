package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Objects;


@Config
@Autonomous(name = "autoFarPark", group = "Linear Opmode")

public class autoFarPark extends LinearOpMode {
    // default alliance is red
    public static String alliance = "RED";
    // this will hold the trajectoryAction we select based on alliance color
    public static Action trajectoryActionChosen;

    @Override
    public void runOpMode() throws InterruptedException {

        // default alliance is red, only overwrite if blue was chosen in game values
        if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = "BLUE";
        }

        // instantiate drive class (MecanumDrive) at a particular pose.
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // we build our trajectories during initialization to avoid wasting time during auto
        // tab one is for red
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .turn(Math.toRadians(180))
                .waitSeconds(3);
        // tab two is for blue
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .turn(Math.toRadians(-180))
                .waitSeconds(3);

        waitForStart();
        if (isStopRequested()) return;

        // set our trajectoryAction based on alliance color
        if (Objects.equals(alliance, "RED")) {
            trajectoryActionChosen = tab1.build();
        } else if (Objects.equals(alliance, "BLUE")) {
            trajectoryActionChosen = tab2.build();
        }

        // this is where we actually run the trajectoryAction
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen
                )
        );
    }
}