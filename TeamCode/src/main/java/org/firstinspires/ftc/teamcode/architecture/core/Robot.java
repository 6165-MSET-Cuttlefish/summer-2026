package org.firstinspires.ftc.teamcode.architecture.core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldConfig;
import org.firstinspires.ftc.teamcode.architecture.auto.PedroSetup;
import org.firstinspires.ftc.teamcode.architecture.auto.scheduler.PathActionScheduler;
import org.firstinspires.ftc.teamcode.architecture.OptimizationToggles;
import org.firstinspires.ftc.teamcode.architecture.telemetry.DualTelemetry;

/** Game-agnostic robot base. Game subclasses wire up mechanisms in {@link #initializeGameModules()}. */
@Config
public abstract class Robot {

    public final DualTelemetry telemetry;
    public final EnhancedOpMode opMode;
    public Follower follower;
    public PathActionScheduler pathActionScheduler;
    public TelemetryPacket packet;

    public static TelemetryToggles telemetryToggles = new TelemetryToggles();

    protected Robot(EnhancedOpMode opMode) throws InterruptedException {
        this.opMode = opMode;
        telemetry = new DualTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        // Always start from the configured pose — no pose-carry across a Sloth reload / opmode swap.
        follower = PedroSetup.createFollower(opMode.hardwareMap);
        follower.setPose(new Pose(72, FieldConfig.fieldWidthInches - 10, Math.toRadians(90)));
        pathActionScheduler = new PathActionScheduler(follower);
        initializeGameModules();
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
