package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.yise.Parameters;

import java.util.Objects;


@Config
@Autonomous(name = "autoClosePark", group = "Linear Opmode")

public class autoClosePark extends LinearOpMode {
    // default alliance is red
    public String alliance = "RED";
    // this will hold the trajectoryAction we select based on alliance color
    public Action trajectoryActionChosen;
    public DcMotor intake;


    @Override
    public void runOpMode() throws InterruptedException {

        // default alliance is red, only overwrite if blue was chosen in game values
        if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = "BLUE";
        }

        // instantiate drive class (MecanumDrive) at a particular pose.
        Pose2d initialPose = null;
        if (Objects.equals(alliance, "RED")) {
            initialPose = new Pose2d(53, -50, Math.toRadians(0));
        }else if (Objects.equals(alliance, "BLUE")) {
            initialPose = new Pose2d(-53, 50, Math.toRadians(0));
        }
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        //Intake intake = new Intake(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");

        // we build our trajectories during initialization to avoid wasting time during auto
        // tab one is for red
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                //shoot 3 balls
                .strafeTo(new Vector2d(53, -40));
        // tab two is for blue
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                //shoot 3 balls
                .strafeTo(new Vector2d(-53, 40));

        waitForStart();
        if (isStopRequested()) return;

        // set our trajectoryAction based on alliance color
        if (Objects.equals(alliance, "RED")) {
            trajectoryActionChosen = tab1.build();
        } else if (Objects.equals(alliance, "BLUE")) {
            trajectoryActionChosen = tab2.build();
        }

        Actions.runBlocking(trajectoryActionChosen);
    }
}