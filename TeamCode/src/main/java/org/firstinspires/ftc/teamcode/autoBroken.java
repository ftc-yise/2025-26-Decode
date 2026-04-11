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
@Autonomous(name = "autoBroken", group = "Linear Opmode")

public class autoBroken extends LinearOpMode {
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
            initialPose = new Pose2d(-60, 40, Math.toRadians(0));
        }else if (Objects.equals(alliance, "BLUE")) {
            initialPose = new Pose2d(-60, -40, Math.toRadians(0));
        }
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        //Intake intake = new Intake(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");

        // we build our trajectories during initialization to avoid wasting time during auto
        // tab one is for red
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-50,37))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-33,25), Math.toRadians(180))
                //.setTangent(Math.toRadians(0))
                .waitSeconds(6)
                //the long wait is for shooting 3 balls
                .strafeToLinearHeading(new Vector2d(-10,22), Math.toRadians(90))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-10,49))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-10,22))
                .waitSeconds(6)
                //the long wait is for shooting 3 balls
                .strafeTo(new Vector2d(-58,14));
                //park
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-50,-37))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-33,-25), Math.toRadians(180))
                //.setTangent(Math.toRadians(0))
                .waitSeconds(6)
                //the long wait is for shooting 3 balls
                .strafeToLinearHeading(new Vector2d(-10,-22), Math.toRadians(270))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-10,-49))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-10,-22))
                .waitSeconds(6)
                //the long wait is for shooting 3 balls
                .strafeTo(new Vector2d(-58,-14));
                //park

        waitForStart();
        if (isStopRequested()) return;

        // set our trajectoryAction based on alliance color
        if (Objects.equals(alliance, "RED")) {
            trajectoryActionChosen = tab1.build();
        } else if (Objects.equals(alliance, "BLUE")) {
            trajectoryActionChosen = tab2.build();
        }

        intake.setPower(1);
        Actions.runBlocking(trajectoryActionChosen);
        intake.setPower(0);
    }
}