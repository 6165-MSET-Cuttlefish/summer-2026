package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.architecture.auto.scheduler.PathActionBuilder;
import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.architecture.action.Actions;

/**
 * End-to-end smoke test for the summer-2026 framework. One run exercises: the EnhancedOpMode
 * init/loop lifecycle, Robot + Module + State, the cooperative Action scheduler, the path-action
 * scheduler, the Pedro follower, and dual (DS + dashboard) telemetry.
 *
 * <p><strong>Drivetrain motion is OFF by default</strong> ({@code Tuning.enableDrive}). With it off
 * the auto actuates nothing by default — it steps the scheduler, flips a mock state, runs timed
 * actions, and reads odometry — so the architecture can be verified safely. To also prove a
 * hardware write, set {@code MockMechanism.pulseMotorName} on FtcDashboard to a motor name and it
 * pulses while the sequence runs. Turn {@code enableDrive} on only after confirming motor +
 * odometry directions, ideally wheels-off-ground first: the Pedro constants here are framework
 * defaults, not tuned for this robot.
 */
@Autonomous(name = "Mock Architecture Test", group = "test")
public class MockAuto extends EnhancedOpMode {

    @Config
    public static class Tuning {
        /** Commanded drivetrain motion. Leave false for a no-motion plumbing test. */
        public static boolean enableDrive = false;
        public static double driveInches = 24;
        public static int safetyTimeoutMs = 15000;
    }

    private String phase = "init";
    private MockRobot mock;

    @Override
    protected Robot createRobot() throws InterruptedException {
        Context.allianceColor = AllianceColor.RED;
        mock = new MockRobot(this);
        return mock;
    }

    @Override
    protected void initialize() {
        Pose start = robot.follower.getPose();
        double heading = start.getHeading();

        PathActionBuilder seq = new PathActionBuilder(robot.follower, () -> (long) getGameTimer().milliseconds())
                .setStartPose(start)
                .setState(MockMechanism.Status.ACTIVE)
                .run(() -> phase = "scheduler started, mech ACTIVE");

        if (Tuning.enableDrive) {
            Pose forward = new Pose(
                    start.getX() + Math.cos(heading) * Tuning.driveInches,
                    start.getY() + Math.sin(heading) * Tuning.driveInches,
                    heading);
            seq.run(() -> phase = "driving forward")
               .buildPath(p -> p.addLine(forward).setConstantHeading(heading))
               .action(Actions.builder()
                       .run(() -> phase = "blocking action (pause at far point)")
                       .delay(750)
                       .build())
               .run(() -> phase = "driving back")
               .buildPath(p -> p.addLine(start).setConstantHeading(heading));
        } else {
            seq.action(Actions.builder()
                    .run(() -> phase = "blocking action (no-motion mode)")
                    .delay(1500)
                    .build());
        }

        seq.setState(MockMechanism.Status.IDLE)
           .run(() -> phase = "sequence complete")
           .setTimeOverride(Tuning.safetyTimeoutMs, () -> phase = "SAFETY TIMEOUT — scheduler aborted");

        robot.pathActionScheduler = seq.build();
    }

    @Override
    protected void gameLoop() {
        robot.pathActionScheduler.update();
    }

    @Override
    protected void telemetry() {
        telemetry.addData("Mock Phase", phase);
        telemetry.addData("Drive Enabled", Tuning.enableDrive);
        telemetry.addData("Pulse Motor", mock.mech.boundMotorName()
                + (mock.mech.isMotorBound() ? " [bound]" : " [none — set MockMechanism.pulseMotorName]"));
        if (robot.pathActionScheduler != null) {
            telemetry.addData("Scheduler", robot.pathActionScheduler.getDebugInfo());
            telemetry.addData("Complete", robot.pathActionScheduler.isComplete());
        }
    }
}
