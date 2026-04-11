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
@Autonomous(name = "autoFarLoadingZone", group = "Linear Opmode")

public class autoFarLoadingZone extends LinearOpMode {
    // default alliance is red
    public Turret.turretAlliance alliance = Turret.turretAlliance.RED;
    // this will hold the trajectoryAction we select based on alliance color
    public Action trajectoryActionChosen;
    public DcMotor intake;
    public Pose2d initialPose;
    public Action traj_1 = null;
    public Action traj_2 = null;

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
                .strafeTo(new Vector2d(62,58))
                .strafeTo(new Vector2d(62,19));

        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(62,38));

        //blue side now
        TrajectoryActionBuilder tab3 =drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(62,-58))
                .strafeTo(new Vector2d(62,-19));
        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()
                .strafeTo(new Vector2d(62,-38));

        waitForStart();
        if (isStopRequested()) return;

        // set our trajectoryAction based on alliance color
        if (alliance == Turret.turretAlliance.RED) {
            traj_1 = tab1.build();
            traj_2 = tab2.build();
        } else if (alliance == Turret.turretAlliance.BLUE) {
            traj_1 = tab3.build();
            traj_2 = tab4.build();
        }

        //shoot 3 balls
        //intake on
        Actions.runBlocking(traj_1);//drive to loading zone and back
        //intake off
        //shoot 3 balls
        Actions.runBlocking(traj_2);//park

    }
}