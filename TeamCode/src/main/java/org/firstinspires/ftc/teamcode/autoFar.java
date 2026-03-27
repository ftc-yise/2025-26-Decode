package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.Hood;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.ShotPatternManager;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.lifter;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Objects;

@Config
@Autonomous(name = "autoFar", group = "Linear Opmode")
public class autoFar extends LinearOpMode {

    public String alliance = "RED";
    public Action trajectoryActionChosen;
    public DcMotor intake;

    private ShooterClass shooter;
    private Spindexer spin;
    private Hood hood;
    private lifter lift;
    private ShooterExecutionClass autoShoot;
    private ShotPatternManager patternMgr;
    private Turret turret;

    private PrintWriter autoLogWriter = null;
    private final ElapsedTime runtime = new ElapsedTime();
    private String autoLogFilePath = null;

    private ShotPatternManager.ShotPattern patternFromTag(int tagId) {
        switch (tagId) {
            case 21: return ShotPatternManager.ShotPattern.GPP;
            case 22: return ShotPatternManager.ShotPattern.PGP;
            case 23: return ShotPatternManager.ShotPattern.PPG;
            default: return null;
        }
    }

    private void startAutoLog() {
        try {
            File dir = new File("/sdcard/FIRST/autoLogs");
            if (!dir.exists()) dir.mkdirs();

            autoLogFilePath = "/sdcard/FIRST/autoLogs/autoFar_" + System.currentTimeMillis() + ".csv";
            autoLogWriter = new PrintWriter(new FileWriter(autoLogFilePath));

            autoLogWriter.println(
                    "time_s,phase,alliance,tagId," +
                            "turretMode,turretId,pipeline," +
                            "autoState,shots,forceShooting,currentSiloIndex," +
                            "moveToSilo,spinWait,spinUp,fireUp,fireDown," +
                            "shooterMode,targetRPM,currentRPM,errorRPM," +
                            "spxMode,spxAngle,spxTarget,spxError,spxPower," +
                            "silo1,silo2,silo3"
            );
            autoLogWriter.flush();
        } catch (IOException e) {
            telemetry.addData("AUTO LOG ERROR", e.getMessage());
            telemetry.update();
        }
    }

    private void logAutoLine(String phase, int tagId) {
        if (autoLogWriter == null || turret == null || autoShoot == null || shooter == null || spin == null) return;

        ShooterClass.ShooterTelemetry s = shooter.getTelemetry();
        Spindexer.TelemetryPacket spx = spin.getTelemetry();

        autoLogWriter.printf(
                "%.3f,%s,%s,%d," +
                        "%s,%d,%d," +
                        "%s,%d,%b,%d," +
                        "%.3f,%.3f,%.3f,%.3f,%.3f," +
                        "%s,%.2f,%.2f,%.2f," +
                        "%s,%.1f,%.1f,%.1f,%.3f," +
                        "%s,%s,%s%n",
                runtime.seconds(), phase, alliance, tagId,
                turret.mode, turret.getID(), turret.limelight.getStatus().getPipelineIndex(),
                autoShoot.getStateName(), autoShoot.shots, autoShoot.forceShooting, autoShoot.getLastTargetSiloIndex(),
                autoShoot.getLastMoveToSiloSec(),
                autoShoot.getLastSpinWaitSec(),
                autoShoot.getLastSpinUpSec(),
                autoShoot.getLastFireLiftUpSec(),
                autoShoot.getLastFireLiftDownSec(),
                s.mode, s.targetRPM, s.currentRPM, s.errorRPM,
                spx.mode, spx.currentAngle, spx.targetAngle, spx.angleError, spx.appliedPower,
                spx.siloColors[0], spx.siloColors[1], spx.siloColors[2]
        );
        autoLogWriter.flush();
    }

    private void closeAutoLog() {
        if (autoLogWriter != null) {
            autoLogWriter.flush();
            autoLogWriter.close();
            autoLogWriter = null;
        }
    }

    private void serviceAutoSystems(boolean highGoal, String phase, int tagId) {
        // turret must stay in AUTO the whole shooting window
        turret.limelight.pipelineSwitch(2);
        turret.autoMode();
        turret.mode = Turret.turretMode.AUTO;

        // IMPORTANT: this is the command the shooter needs every loop.
        // updateTelemetry() alone does not drive the motor.
        if (highGoal) {
            shooter.update(false, false, true);
            hood.setTarget(75);
        } else {
            shooter.update(false, true, false);
            hood.setTarget(26);
        }

        // Keep subsystem updates alive every tick, like teleop does
        hood.update();
        spin.sampleSensorsNow();
        spin.update();
        lift.update();
        shooter.updateTelemetry();
        autoShoot.update();

        logAutoLine(phase, tagId);
    }

    private Action makeShootWindowAction(
            final String phase,
            final double durationSec,
            final boolean highGoal,
            final int tagId
    ) {
        return new Action() {
            private final ElapsedTime windowTimer = new ElapsedTime();
            private boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    windowTimer.reset();
                    intake.setPower(1);

                    turret.limelight.pipelineSwitch(2);
                    turret.autoMode();
                    turret.mode = Turret.turretMode.AUTO;

                    if (!autoShoot.isBusy()) {
                        autoShoot.startForcedCycle();
                    }
                }

                serviceAutoSystems(highGoal, phase, tagId);

                if (windowTimer.seconds() >= durationSec) {
                    autoShoot.stopForcedCycle();
                    intake.setPower(0);
                    return false;
                }
                return true;
            }
        };
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Parameters.autonomous = Parameters.AUTONOMOUS.YES;

        Pose2d initialPose;
        if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = "BLUE";
            initialPose = new Pose2d(-63, 12, Math.toRadians(90));
        } else {
            alliance = "RED";
            initialPose = new Pose2d(-63, -12, Math.toRadians(270));
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = new ShooterClass(hardwareMap);
        spin = new Spindexer(hardwareMap);
        hood = new Hood(hardwareMap);
        lift = new lifter(hardwareMap);
        autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap, lift);
        patternMgr = new ShotPatternManager();
        autoShoot.setPatternManager(patternMgr);

        Turret.turretAlliance turretAlliance = Objects.equals(alliance, "RED")
                ? Turret.turretAlliance.RED
                : Turret.turretAlliance.BLUE;
        turret = new Turret(hardwareMap, turretAlliance, telemetry);

        // Scan AprilTag / pattern during init
        turret.limelight.pipelineSwitch(2);
        sleep(300);
        int tagId = turret.getID();
        ShotPatternManager.ShotPattern p = patternFromTag(tagId);
        if (p != null) {
            patternMgr.clear();
            patternMgr.addPattern(p.sequence);
        }

        startAutoLog();
        logAutoLine("INIT", tagId);

        hood.stop();
        spin.initSilos();
        spin.goToSilo2();
        lift.setDown();

        Action shootStart = makeShootWindowAction("START", 12.0, true, tagId);
        Action shootMiddle = makeShootWindowAction("MIDDLE", Objects.equals(alliance, "RED") ? 10.0 : 12.0, true, tagId);
        Action shootEnd = makeShootWindowAction("END", Objects.equals(alliance, "RED") ? 10.0 : 12.0, true, tagId);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .afterTime(0, shootStart)
                .waitSeconds(12)
                .strafeTo(new Vector2d(-33, -12))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-33, -50))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-61, -12))
                .afterTime(0, shootMiddle)
                .waitSeconds(12)
                .strafeTo(new Vector2d(-11, -12))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-11, -50))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-61, -12))
                .afterTime(0, shootEnd)
                .waitSeconds(12);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .afterTime(0, shootStart)
                .waitSeconds(12)
                .strafeTo(new Vector2d(-33, 12))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-33, 50))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-61, 12))
                .afterTime(0, shootMiddle)
                .waitSeconds(Objects.equals(alliance, "RED") ? 10.0 : 12.0)
                .strafeTo(new Vector2d(-11, 12))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-11, 50))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-61, 12))
                .afterTime(0, shootEnd)
                .waitSeconds(Objects.equals(alliance, "RED") ? 10.0 : 12.0);

        waitForStart();
        if (isStopRequested()) {
            closeAutoLog();
            return;
        }

        trajectoryActionChosen = Objects.equals(alliance, "RED") ? tab1.build() : tab2.build();

        intake.setPower(1);
        logAutoLine("START", tagId);
        Actions.runBlocking(trajectoryActionChosen);
        intake.setPower(0);
        logAutoLine("END", tagId);
        closeAutoLog();
    }
}
