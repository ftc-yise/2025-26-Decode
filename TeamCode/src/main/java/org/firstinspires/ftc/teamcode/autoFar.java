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
                .strafeTo(new Vector2d(26,37));

        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(51,37))
                .strafeTo(new Vector2d(19,65));

        TrajectoryActionBuilder tab3 =tab2.endTrajectory().fresh()
                .strafeTo(new Vector2d(26,13));

        TrajectoryActionBuilder tab4 =tab3.endTrajectory().fresh()
                .strafeTo(new Vector2d(51,13))
                .strafeTo(new Vector2d(19,65));

        TrajectoryActionBuilder tab5 =tab4.endTrajectory().fresh()
                .strafeTo(new Vector2d(35,65));

        //blue side now
        TrajectoryActionBuilder tab6 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(26,-37));

        TrajectoryActionBuilder tab7 =tab6.endTrajectory().fresh()
                .strafeTo(new Vector2d(51,-37))
                .strafeTo(new Vector2d(19,-65));

        TrajectoryActionBuilder tab8= tab7.endTrajectory().fresh()
                .strafeTo(new Vector2d(26,-13));

        TrajectoryActionBuilder tab9 = tab8.endTrajectory().fresh()
                .strafeTo(new Vector2d(51,-13))
                .strafeTo(new Vector2d(19,-65));

        TrajectoryActionBuilder tab10 =tab9.endTrajectory().fresh()
                .strafeTo(new Vector2d(35,-65));


        waitForStart();
        if (isStopRequested()) return;

        // set our trajectoryAction based on alliance color
        if (alliance == Turret.turretAlliance.RED) {
            trajectoryActionChosen = tab1.build();
            trajectoryActionChosen = tab2.build();
            trajectoryActionChosen = tab3.build();
            trajectoryActionChosen = tab4.build();
            trajectoryActionChosen = tab5.build();
        } else if (Objects.equals(alliance, "BLUE")) {

            trajectoryActionChosen = tab6.build();
            trajectoryActionChosen = tab7.build();
            trajectoryActionChosen = tab8.build();
            trajectoryActionChosen = tab9.build();
            trajectoryActionChosen = tab10.build();
        }

        Action traj_1 = tab1.build();
        Action traj_2 = tab2.build();
        Action traj_3 = tab3.build();
        Action traj_4 = tab4.build();
        Action traj_5 = tab5.build();
        Action traj_6 = tab6.build();
        Action traj_7 = tab7.build();
        Action traj_8 = tab8.build();
        Action traj_9 = tab9.build();
        Action traj_10 =tab10.build();

        //red side
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


        //blue side
        //shoot 3 balls
        Actions.runBlocking(traj_6);//drive to first spike mark
        //intake.setPower(1);
        Actions.runBlocking(traj_7); //drive to end of spike mark then shooting spot
        //intake.setPower(0);
        //shoot 3 balls
        Actions.runBlocking(traj_8);//drive to 2nd spike mark
        //intake.setPower(1);
        Actions.runBlocking(traj_9);//drive to end of spike mark then shooting spot
        //intake.setPower(0);
        //shoot 3 balls
        Actions.runBlocking(traj_10);//drive to park

    }
}