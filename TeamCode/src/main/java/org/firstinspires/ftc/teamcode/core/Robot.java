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
import org.firstinspires.ftc.teamcode.architecture.auto.scheduler.PathActionScheduler;
import org.firstinspires.ftc.teamcode.architecture.OptimizationToggles;
import org.firstinspires.ftc.teamcode.architecture.telemetry.DualTelemetry;

/** Game-agnostic robot base. Game subclasses wire up mechanisms in {@link #initializeGameModules()}. */
@Config
public abstract class Robot {
    public static Robot robot;

    public final DualTelemetry telemetry;
    public final EnhancedOpMode opMode;
    public Follower follower;
    public PathActionScheduler pathActionScheduler;
    public TelemetryPacket packet;

    public static TelemetryToggles telemetryToggles = new TelemetryToggles();

    protected Robot(EnhancedOpMode opMode, boolean preservePosition)
            throws InterruptedException {
        this.opMode = opMode;

        Robot previousRobot = robot;
        telemetry = new DualTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        // Available to module ctors below. If anything throws, wipe so the next init is fresh.
        robot = this;
        try {
            pathActionScheduler = new PathActionScheduler();
            initializeFollower(preservePosition, previousRobot);
            initializeGameModules();
        } catch (Throwable t) {
            robot = null;
            throw t;
        }
    }

    private void initializeFollower(boolean preservePosition, Robot previousRobot) {
        if (preservePosition && previousRobot != null && previousRobot.follower != null) {
            // Use live Pinpoint pose so a hot-reload / opmode swap doesn't snap back to start.
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

    public static class TelemetryToggles {
        public boolean dsTelemetry = true;
        public boolean dashboardTelemetry = true;
        public boolean voltage = true;
        public boolean current = false;
        public boolean loopProfile = OptimizationToggles.loopProfileTelemetryByDefault;
    }
}
