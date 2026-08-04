package org.firstinspires.ftc.teamcode.decode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.Endgame;
import org.firstinspires.ftc.teamcode.modules.Magazine;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Turret;

import static org.firstinspires.ftc.teamcode.decode.DecodeContext.blueApriltagPose;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.blueTargetPose;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.redApriltagPose;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.redTargetPose;

@Config
public class DecodeRobot extends Robot {

    public static DecodeRobot robot;

    public Pose targetPose;
    public Pose targetApriltagPose;
    public Pose cornerPose;

    public Drivetrain drivetrain;
    public Endgame endgame;
    public Shooter shooter;
    public Magazine magazine;
    public Turret turret;
    public RobotActions actions;

    public static WriteToggles writeToggles = new WriteToggles();
    public static ShooterTelemetry shooterTelemetry = new ShooterTelemetry();
    public static TurretTelemetry turretTelemetry = new TurretTelemetry();
    public static DrivetrainTelemetry drivetrainTelemetry = new DrivetrainTelemetry();
    public static EndgameTelemetry endgameTelemetry = new EndgameTelemetry();
    public static MagazineTelemetry magazineTelemetry = new MagazineTelemetry();
    public static AprilTagTelemetry aprilTagTelemetry = new AprilTagTelemetry();

    public DecodeRobot(EnhancedOpMode opMode) throws InterruptedException {
        super(opMode);
    }

    @Override
    protected void initializeGameModules() {
        // Must precede the module constructors: some modules (Magazine etc.) read DecodeRobot.robot.
        robot = this;
        setTargetPosesForAlliance();
        drivetrain = new Drivetrain(opMode.hardwareMap);
        shooter = new Shooter(opMode.hardwareMap);
        turret = new Turret(opMode.hardwareMap);
        magazine = new Magazine(opMode.hardwareMap);
        endgame = new Endgame(opMode.hardwareMap);
        actions = new RobotActions(this);
    }

    private void setTargetPosesForAlliance() {
        if (Context.allianceColor == null) {
            throw new IllegalStateException(
                    "Context.allianceColor is null; auto/teleop must set it before Robot init.");
        }
        switch (Context.allianceColor) {
            case RED:
                targetPose = redTargetPose;
                targetApriltagPose = redApriltagPose;
                cornerPose = new Pose(141.5, 141.5);
                break;
            case BLUE:
                targetPose = blueTargetPose;
                targetApriltagPose = blueApriltagPose;
                cornerPose = new Pose(0, 141.5);
                break;
            default:
                throw new IllegalStateException("Unhandled alliance: " + Context.allianceColor);
        }
    }

    public void updateWriteToggles() {
        boolean robotWriteEnabled = writeToggles.robotWrite;

        drivetrain.setWriteEnabled(robotWriteEnabled && writeToggles.drivetrainWrite);
        drivetrain.setTelemetryEnabled(drivetrainTelemetry.TOGGLE);

        endgame.setWriteEnabled(robotWriteEnabled && writeToggles.endgameWrite);
        endgame.setTelemetryEnabled(endgameTelemetry.TOGGLE);

        shooter.setWriteEnabled(robotWriteEnabled && writeToggles.shooterWrite);
        shooter.setTelemetryEnabled(shooterTelemetry.TOGGLE);

        magazine.setWriteEnabled(robotWriteEnabled && writeToggles.magazineWrite);
        magazine.setTelemetryEnabled(magazineTelemetry.TOGGLE);

        turret.setWriteEnabled(robotWriteEnabled && writeToggles.turretWrite);
        turret.setTelemetryEnabled(turretTelemetry.TOGGLE);
    }

    public static class WriteToggles {
        public boolean shooterWrite = true;
        public boolean magazineWrite = true;
        public boolean turretWrite = true;
        public boolean drivetrainWrite = true;
        public boolean endgameWrite = true;
        public boolean robotWrite = true;
    }

    public static class ShooterTelemetry {
        public boolean TOGGLE = true;
        public boolean flywheel = true;
        public boolean lut = true;
        public boolean hood = true;
        public boolean current = false;
    }

    public static class TurretTelemetry {
        public boolean TOGGLE = true;
        public boolean position = true;
        public boolean servos = false;
    }

    public static class DrivetrainTelemetry {
        public boolean TOGGLE = false;
        public boolean current = false;
    }

    public static class EndgameTelemetry {
        public boolean TOGGLE = true;
        public boolean current = false;
        public boolean initial = true;
        public boolean pto = false;
    }

    public static class MagazineTelemetry {
        public boolean TOGGLE = true;
        public boolean intake = false;
        public boolean vertical = false;
        public boolean servos = false;
        public boolean current = false;
        public boolean headlights = false;
        public boolean colorSensors = true;
    }

    public static class AprilTagTelemetry {
        public boolean TOGGLE = false;
        public boolean raw = false;
    }
}
