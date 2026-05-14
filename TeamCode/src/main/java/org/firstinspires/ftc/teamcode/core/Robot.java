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

/**
 * Game-agnostic base class for the robot. Holds purely framework state — telemetry, follower,
 * scheduler, target poses, opmode reference — and exposes hooks ({@link #initializeGameModules()},
 * {@link #setTargetPosesForAlliance()}, {@link #updateWriteToggles()}) that the game-specific
 * subclass implements. The constructor wires up framework state and then calls those hooks.
 *
 * <p>Subclasses provide the actual mechanism modules (drivetrain, shooter, etc.) and any
 * game-specific telemetry/write toggles.
 */
@Config
public abstract class Robot {
    /** Set in this class's constructor; read by framework helpers that still need a fallback singleton. */
    public static Robot robot;

    public final EnhancedTelemetry telemetry;
    public final EnhancedOpMode opMode;
    public Follower follower;
    public PathActionScheduler pathActionScheduler;
    public TelemetryPacket packet;

    public Pose targetPose;
    public Pose targetApriltagPose;
    public Pose cornerPose;

    /** Framework-only telemetry toggles (game subclasses keep their own). */
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

    /** Game subclass hook: instantiate game-specific mechanism modules. */
    protected abstract void initializeGameModules();

    /** Game subclass hook: assign {@link #targetPose}, {@link #targetApriltagPose}, {@link #cornerPose}. */
    protected abstract void setTargetPosesForAlliance();

    /** Push the latest dashboard {@code WriteToggles} state into module flags. */
    public abstract void updateWriteToggles();

    /** Framework-only telemetry toggles. Game subclasses keep their own static instances. */
    public static class FrameworkTelemetry {
        public boolean dsTelemetry = true;
        public boolean dashboardTelemetry = true;
        public boolean dsDebug = false;
        public boolean voltage = true;
        public boolean current = false;
        /** Render per-section loop-time breakdown in dashboard telemetry. */
        public boolean loopProfile = OptimizationToggles.loopProfileTelemetryByDefault;
    }
}
