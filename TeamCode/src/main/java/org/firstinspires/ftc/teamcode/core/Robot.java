package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.auto.PedroSetup;
import org.firstinspires.ftc.teamcode.architecture.auto.RobotHardwareConfig;
import org.firstinspires.ftc.teamcode.architecture.auto.pathaction.PathActionScheduler;
import org.firstinspires.ftc.teamcode.architecture.diagnostics.OptimizationToggles;
import org.firstinspires.ftc.teamcode.architecture.telemetry.EnhancedTelemetry;

/**
 * Game-agnostic robot base. Holds the framework-level state (telemetry, follower, scheduler,
 * shared target poses) and exposes hooks for the game subclass to wire up its mechanisms.
 */
@Config
public abstract class Robot {
    public static Robot robot;

    public final EnhancedTelemetry telemetry;
    public final EnhancedOpMode opMode;
    public Follower follower;
    public PathActionScheduler pathActionScheduler;
    public TelemetryPacket packet;

    public Pose targetPose;
    public Pose targetApriltagPose;
    public Pose cornerPose;

    public static FrameworkTelemetry telemetryToggles = new FrameworkTelemetry();

    protected Robot(EnhancedOpMode opMode, boolean preservePosition, boolean initSRSHub)
            throws InterruptedException {
        this.opMode = opMode;

        Robot previousRobot = robot;
        telemetry = new EnhancedTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        setTargetPosesForAlliance();

        robot = this;

        pathActionScheduler = new PathActionScheduler();
        initializeFollower(preservePosition, previousRobot);
        initializeGameModules();
    }

    private void initializeFollower(boolean preservePosition, Robot previousRobot) {
        if (preservePosition && previousRobot != null && previousRobot.follower != null) {
            // Read live Pinpoint pose so a hot-reload / opmode swap doesn't snap us back to start.
            GoBildaPinpointDriver pinpoint = opMode.hardwareMap.get(
                    GoBildaPinpointDriver.class, RobotHardwareConfig.PINPOINT_NAME);
            pinpoint.update();
            Pose2D rawPose = pinpoint.getPosition();
            Pose livePose = new Pose(
                    rawPose.getX(DistanceUnit.INCH),
                    rawPose.getY(DistanceUnit.INCH),
                    rawPose.getHeading(AngleUnit.RADIANS));

            follower = PedroSetup.createFollower(opMode.hardwareMap);
            follower.setPose(livePose);
        } else {
            follower = PedroSetup.createFollower(opMode.hardwareMap);
            follower.setPose(new Pose(72, FieldConfig.fieldWidthInches - 10, Math.toRadians(90)));
        }
    }

    protected abstract void initializeGameModules();
    protected abstract void setTargetPosesForAlliance();
    public abstract void updateWriteToggles();

    public static class FrameworkTelemetry {
        public boolean dsTelemetry = true;
        public boolean dashboardTelemetry = true;
        public boolean dsDebug = false;
        public boolean voltage = true;
        public boolean current = false;
        public boolean loopProfile = OptimizationToggles.loopProfileTelemetryByDefault;
    }
}
