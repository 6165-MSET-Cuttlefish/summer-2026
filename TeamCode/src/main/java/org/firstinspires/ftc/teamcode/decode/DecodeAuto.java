package org.firstinspires.ftc.teamcode.decode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.architecture.auto.scheduler.SchedulerState;
import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.modules.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.Magazine;
import org.firstinspires.ftc.teamcode.modules.Turret;

public abstract class DecodeAuto extends DecodeOpMode {
    protected Pose startPose;
    protected PoseHistory poseHistory;
    protected Timer actionTimer;

    private PathChain lastRenderedPath = null;
    private boolean autonomousSequenceBuiltInInit = false;
    private int visualizationLoopCounter = 0;

    /**
     * DECODE-auto-only perf knobs. Kept local (not in framework {@code OptimizationToggles}, which the
     * summer refactor trimmed) so this port unit stays self-contained; names preserved from Decode.
     */
    @Config
    public static class AutoToggles {
        public static boolean optimizeBuildAutonomousSequenceOnceInInit = false;
        public static boolean optimizeRenderVisualizationCadence = true;
        public static int optimizeRenderVisualizationEveryNLoops = 2;
    }

    private void renderVisualization(boolean drawPlannedPaths) {
        if (drawPlannedPaths) {
            PathChain currentPath = robot.pathActionScheduler.getCurrentPath();
            if (currentPath != null) {
                lastRenderedPath = currentPath;
            }
            if (lastRenderedPath != null) {
                FieldVisualization.drawPath(robot.packet.fieldOverlay(), lastRenderedPath,
                        currentPath != null
                                ? FieldVisualization.COLOR_CURRENT_PATH
                                : FieldVisualization.COLOR_PATH);
            }
        }
    }

    @Override
    protected boolean shouldWriteDuringInit() {
        return true;
    }

    protected abstract Pose getSetupPose();
    protected abstract Pose getStartPose();

    protected abstract void buildAutonomousSequence();


    @Override
    protected void initialize() {
        Turret.isAuto = true;
        Drivetrain.DriveState.EXTERNAL.activate();

        autonomousSequenceBuiltInInit = false;
        visualizationLoopCounter = 0;

        startPose = getSetupPose();
        robot.follower.setPose(startPose);

        poseHistory = robot.follower.getPoseHistory();
        actionTimer = new Timer();

        if (Context.allianceColor.equals(AllianceColor.RED)) {
            Magazine.HeadlightFrontState.RED.activate();
        } else {
            Magazine.HeadlightFrontState.CYAN.activate();
        }

        robot.magazine.setStatusPrismSnake();
    }

    @Override
    protected void initializeLoop() {
        if (!AutoToggles.optimizeBuildAutonomousSequenceOnceInInit) {
            buildAutonomousSequence();
        } else if (!autonomousSequenceBuiltInInit) {
            buildAutonomousSequence();
            autonomousSequenceBuiltInInit = true;
        }

        if (shouldRenderVisualizationThisLoop()) {
            renderVisualization(true);
        }
        validateStartPose();
    }

    private void validateStartPose() {
        Pose current = robot.follower.getPose();
        Pose target = getStartPose();

        double dx = current.getX() - target.getX();
        double dy = current.getY() - target.getY();
        double distanceInches = Math.sqrt(dx * dx + dy * dy);

        double headingDiff = current.getHeading() - target.getHeading();
        headingDiff = Math.atan2(Math.sin(headingDiff), Math.cos(headingDiff));
        double headingDegrees = Math.abs(Math.toDegrees(headingDiff));

        boolean positionOk = distanceInches <= 2.0;
        boolean headingOk  = headingDegrees <= 10.0;

        if (!positionOk || !headingOk) {
            robot.telemetry.addData("START POSE WARNING",
                    "Dist: %.2f\" (max 2\"), Hdg: %.1f° (max 10°)",
                    distanceInches, headingDegrees);
            Magazine.HeadlightFrontState.RED_STROBE.activate();
        } else {
            robot.telemetry.addData("Start Pose OK",
                    "Dist: %.2f\", Hdg: %.1f°",
                    distanceInches, headingDegrees);
            Magazine.HeadlightFrontState.RED_GREEN_STROBE.activate();
        }
    }

    @Override
    protected void gameLoop() {
        if (robot.drivetrain.isBonk()
                && robot.pathActionScheduler.getCurrentState() == SchedulerState.PATH_RUNNING) {
            robot.pathActionScheduler.skipCurrentSegment();
            robot.drivetrain.bonkTimer.reset();
        }

        long stamp = getProfiler().enterSection();
        robot.pathActionScheduler.update();
        getProfiler().leaveSection("auto.scheduler.update", stamp);

        stamp = getProfiler().enterSection();
        if (shouldRenderVisualizationThisLoop()) {
            renderVisualization(true);
        }
        getProfiler().leaveSection("auto.renderVisualization", stamp);

        (robot.magazine.intakeIsFull
                ? Magazine.HeadlightFrontState.GREEN
                : Magazine.HeadlightFrontState.ORANGE).activate();
    }

    @Override
    protected void telemetry() {
        robot.telemetry.addDashboardData("Time Elapsed", "%.1f seconds", actionTimer.getElapsedTimeSeconds());
        robot.telemetry.addDashboardData("Scheduler", robot.pathActionScheduler.getDebugInfo());
        robot.telemetry.addDashboardData("Total Segments", robot.pathActionScheduler.getTotalSegments());
        robot.telemetry.addDashboardData("Obelisk", DecodeContext.motif.toString());

        Pose followerPose = robot.follower.getPose();
        robot.telemetry.addDashboardData("Follower Pose", "X:%.1f Y:%.1f H:%.1f°",
                followerPose.getX(), followerPose.getY(), Math.toDegrees(followerPose.getHeading()));
        robot.telemetry.addDashboardData("Follower Busy", robot.follower.isBusy());

        PathChain currentPath = robot.pathActionScheduler.getCurrentPath();
        if (currentPath != null) {
            Pose endPose = currentPath.endPose();
            robot.telemetry.addDashboardData("Path Target", "X:%.1f Y:%.1f H:%.1f°",
                    endPose.getX(), endPose.getY(), Math.toDegrees(endPose.getHeading()));
        }
    }

    private boolean shouldRenderVisualizationThisLoop() {
        if (!AutoToggles.optimizeRenderVisualizationCadence) {
            return true;
        }

        int every = Math.max(1, AutoToggles.optimizeRenderVisualizationEveryNLoops);
        return (visualizationLoopCounter++ % every) == 0;
    }
}
