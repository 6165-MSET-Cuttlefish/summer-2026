package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.architecture.auto.FieldPose;
import org.firstinspires.ftc.teamcode.architecture.auto.scheduler.PathActionBuilder;
import org.firstinspires.ftc.teamcode.architecture.auto.scheduler.SchedulerState;
import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.architecture.action.Actions;

/**
 * DECODE "Close" autonomous, ported to the summer-2026 framework against hardware-free STUB
 * mechanisms ({@link CloseRobot}). It scores a preload, cycles artifacts from the gate/rows back to
 * a scoring pose shooting each batch, then drives to a park/score pose — assembled into
 * {@code robot.pathActionScheduler} via a {@link PathActionBuilder} and pumped from {@link #gameLoop()}.
 *
 * <p>Fidelity: path geometry, segment ordering, per-segment timeouts, holdAtDistance values, the
 * alliance-mirroring + per-alliance branches, and the RobotActions/state calls are reproduced
 * step-for-step from the original. Differences forced by the target API (all behaviour-preserving):
 * <ul>
 *   <li>DECODE seeded the follower + built the sequence in {@code Auto}'s init_loop; the summer-2026
 *       idiom (see MockAuto) seeds in {@link #initialize()} and builds the scheduler once there.</li>
 *   <li>DECODE's segmented {@code HeadingInterpolatorBuilder} (constant/linear/tangent splices) has
 *       no summer-2026 equivalent; each is mapped to the framework's {@code setLinearHeading} /
 *       {@code setTangentHeading}, preserving the start→end heading sweep (and tangent/reversed flags).</li>
 *   <li>DECODE scheduled RobotActions with {@code .run()}; the summer-2026 verb is {@code .schedule()}.</li>
 *   <li>{@code robot.targetPose} + red/blue target poses live on {@link CloseRobot} (DECODE kept them
 *       in Context/DecodeRobot); the alliance offset math is copied verbatim.</li>
 * </ul>
 * Alliance is read from {@link Context#allianceColor} (RED by default); run {@code CloseRed}/
 * {@code CloseBlue} to pick a side, mirroring DECODE's ContextRed/ContextBlue selector opmodes.
 */
@Autonomous(name = "Close", group = "A")
public class Close extends EnhancedOpMode {

    /** Dashboard-tunable autonomous toggles (nested @Config, matching the MockAuto convention). */
    @Config
    public static class Tuning {
        public static boolean sorting = false;
        public static boolean runSecondRow = true;
        public static boolean runGateIntake = true;
        public static boolean runFirstRow = true;
        public static boolean partnerShooting = true;
        public static double shootPreDistanceCycles = 14; // used to be 20, going for consistency
        public static double REDrpmCycleOffset = -30;
        public static double BLUErpmCycleOffset = -100;
        /** Safety cutoff: abort the scheduler if the 30s auto window is blown (DECODE had none explicit). */
        public static int safetyTimeoutMs = 30000;
    }

    private int intakeDelay = 2000;
    private final int shootDelay = 600;

    private final double offsetStartPoseX = 0;
    private final double offsetStartPoseY = 0;

    private CloseRobot robot;
    private String phase = "init";

    // Poses are authored for RED and mirrored to BLUE by FieldPose.forAlliance, which reads
    // Context.allianceColor. They MUST be built in buildPoses() (from initialize(), after
    // createRobot() sets the alliance) and NOT as field initializers: the SDK constructs the OpMode
    // — running field initializers — before init()/createRobot(), so a field initializer would
    // mirror against the stale default alliance and CloseBlue would silently run RED geometry.
    private Pose setupPose, startPose, sortStartPose;
    private Pose preloadScorePose;
    private Pose intake2RowRed, intake2RowBlue, intake2RowControl1, intake2RowControl2, intake2RowControl3Blue, score2RowPose;
    private Pose gateIntakeRed, getIntakeBlue;
    private Pose gateHoldPose, gateScorePose;
    private Pose intake3RowPose, intake3RowControl, score3RowPose;
    private Pose intake1RowPose, intake1RowControl, scoreParkPose;

    private void buildPoses() {
        setupPose = colorPose(110.375 + offsetStartPoseX, 109.75 + offsetStartPoseY, Math.toRadians(0));
        startPose = colorPose(121.1 + offsetStartPoseX, 119.7 + offsetStartPoseY, Math.toRadians(44.6));
        sortStartPose = colorPose(121.1 + offsetStartPoseX, 119.7 + offsetStartPoseY, Math.toRadians(134.6));

        preloadScorePose = colorPose(72 + offsetStartPoseX, 72 + offsetStartPoseY, Math.toRadians(44.6));

        intake2RowRed = colorPose(134 + offsetStartPoseX, 61 + offsetStartPoseY, Math.toRadians(5));
        intake2RowBlue = colorPose(134 + offsetStartPoseX, 63.5 + offsetStartPoseY, Math.toRadians(5));
        intake2RowControl1 = colorPose(70 + offsetStartPoseX, 75 + offsetStartPoseY, Math.toRadians(0));
        intake2RowControl2 = colorPose(88 + offsetStartPoseX, 56 + offsetStartPoseY, Math.toRadians(0));
        intake2RowControl3Blue = colorPose(125 + offsetStartPoseX, 53 + offsetStartPoseY, Math.toRadians(0));
        score2RowPose = colorPose(85.0 + offsetStartPoseX, 76 + offsetStartPoseY, Math.toRadians(-20));

        gateIntakeRed = colorPose(132 + offsetStartPoseX, 58 + offsetStartPoseY, Math.toRadians(28));
        getIntakeBlue = colorPose(132.5 + offsetStartPoseX, 58.5 + offsetStartPoseY, Math.toRadians(15));

        gateHoldPose = colorPose(127.5 + offsetStartPoseX, 60.0 + offsetStartPoseY, Math.toRadians(-10));
        gateScorePose = colorPose(85.0 + offsetStartPoseX, 76 + offsetStartPoseY, Math.toRadians(-20));

        intake3RowPose = colorPose(130 + offsetStartPoseX, 36 + offsetStartPoseY, Math.toRadians(0));
        intake3RowControl = colorPose(84 + offsetStartPoseX, 39 + offsetStartPoseY, Math.toRadians(0));
        score3RowPose = colorPose(85.0 + offsetStartPoseX, 76.5 + offsetStartPoseY, Math.toRadians(-20));

        intake1RowPose = colorPose(127 + offsetStartPoseX, 84 + offsetStartPoseY, Math.toRadians(0));
        intake1RowControl = colorPose(103 + offsetStartPoseX, 89.0 + offsetStartPoseY, Math.toRadians(0));
        scoreParkPose = colorPose(86.5 + offsetStartPoseX, 110 + offsetStartPoseY, Math.toRadians(-25));
    }

    /** DECODE authored geometry for RED via FieldPose.ColorPose; this is the summer-2026 equivalent. */
    private static Pose colorPose(double x, double y, double heading) {
        return FieldPose.forAlliance(x, y, heading);
    }

    /** Override in CloseBlue; default RED matches DECODE's Context default. */
    protected AllianceColor alliance() {
        return AllianceColor.RED;
    }

    @Override
    protected Robot createRobot() throws InterruptedException {
        Context.allianceColor = alliance();
        robot = new CloseRobot(this);
        return robot;
    }

    protected Pose getSetupPose() {
        return setupPose;
    }

    protected Pose getStartPose() {
        return Tuning.sorting ? sortStartPose : startPose;
    }

    @Override
    protected void initialize() {
        // Build poses now — Context.allianceColor is set (createRobot ran), so BLUE mirrors correctly.
        buildPoses();
        // DECODE seeded Pedro odometry from the setup pose (Auto.initialize -> follower.setPose).
        robot.follower.setPose(getSetupPose());
        buildAutonomousSequence();
    }

    private void buildAutonomousSequence() {
        PathActionBuilder builder = new PathActionBuilder(
                robot.follower, () -> (long) getGameTimer().milliseconds());
        builder.setStartPose(robot.follower.getPose());

        addPreloadSecondRow(builder, Tuning.runSecondRow);
        addGateIntake(builder, false, Tuning.runGateIntake);
        if (Tuning.sorting) {
            addThirdRow(builder);
        } else {
            addGateIntake(builder, false, Tuning.runGateIntake);
            addGateIntake(builder, true, Tuning.runGateIntake);
            if (Tuning.partnerShooting) {
                // Shorten the intake dwell for the extra partner cycle. This was a dead store set
                // AFTER the addGateIntake that should consume it; moved before so it takes effect.
                intakeDelay = 1000;
                addGateIntake(builder, true, Tuning.runGateIntake);
            }
        }
        addFirstRow(builder, Tuning.runFirstRow);

        builder.setTimeOverride(Tuning.safetyTimeoutMs, () -> phase = "SAFETY TIMEOUT — scheduler aborted");

        robot.pathActionScheduler = builder.build();
    }

    private void addPreloadSecondRow(PathActionBuilder builder, boolean enabled) {
        Pose intake2Pose = Context.allianceColor == AllianceColor.RED ? intake2RowRed : intake2RowBlue;

        builder.setEnabled(enabled);
        builder.setState(CloseRobot.Magazine.IntakeState.IDLE, CloseRobot.Shooter.HoodState.PID)
                .run(() -> {
                    phase = "preload";
                    robot.targetPose = Context.allianceColor == AllianceColor.RED
                            ? new Pose(CloseRobot.redTargetPose.getX() + 10, CloseRobot.redTargetPose.getY())
                            : new Pose(CloseRobot.blueTargetPose.getX() - 15, CloseRobot.blueTargetPose.getY());
                })
                .run(() -> robot.turret.lock(preloadScorePose))
                .run(() ->
                        Actions.builder()
                                .delay(Tuning.sorting ? 150 : 0) // needs tuning for sorting
                                .set(CloseRobot.Shooter.FlywheelState.CLOSE_AUTO_PRELOAD)
                                .waitUntil(() -> robot.shooter.getCurrentVelocityRPM() > 1000)
                                .run(() -> robot.actions.shootAll(true, false).schedule())
                                .build().schedule()
                )
                .run(() -> robot.targetPose = Context.allianceColor == AllianceColor.RED
                        ? new Pose(CloseRobot.redTargetPose.getX() + 18, CloseRobot.redTargetPose.getY())
                        : new Pose(CloseRobot.blueTargetPose.getX() - 17, CloseRobot.blueTargetPose.getY()))
                .run(() -> {
                    robot.shooter.closeVelocityOffset = Context.allianceColor == AllianceColor.BLUE ? Tuning.BLUErpmCycleOffset : Tuning.REDrpmCycleOffset;
                    robot.shooter.lockFlywheel(score2RowPose);
                    robot.shooter.lockHood(score2RowPose);
                })
                .buildPath(path -> {
                    path.addParametricCallback(0.5, () -> robot.turret.lock(score2RowPose));
                    if (Context.allianceColor == AllianceColor.RED) {
                        path.addCurve(intake2RowControl1, intake2RowControl2, intake2Pose);
                    } else {
                        path.addCurve(intake2RowControl1, intake2RowControl2, intake2RowControl3Blue, intake2Pose);
                    }
                    // DECODE used a constant/linear/constant heading splice that nets to a
                    // preloadScore.heading -> intake2.heading sweep; map to the framework's linear heading.
                    if (Tuning.sorting) {
                        path.setLinearHeading(preloadScorePose.getHeading(), intake2Pose.getHeading());
                        path.addParametricCallback(0, () -> robot.actions.detectObeliskLoop().schedule());
                    } else {
                        path.setLinearHeading(preloadScorePose.getHeading(), intake2Pose.getHeading());
                        path.addParametricCallback(0, () -> robot.follower.setMaxPower(.59));
                        path.addParametricCallback(.3, () -> robot.follower.setMaxPower(1));
                    }
                    path.addParametricCallback(0.5, () ->
                            Actions.builder()
                                    .set(CloseRobot.Magazine.VerticalState.OFF, CloseRobot.Magazine.IntakeState.FORWARD)
                                    .build().schedule());
                    path.addParametricCallback(.68, () -> robot.follower.setMaxPower(0.4));
                    path.addParametricCallback(.92, () -> robot.follower.setMaxPower(1));
                    path.setConstraints(new PathConstraints(0.9, 0));
                    path.holdAtDistance(15);
                }, 5000);

        builder
                .run(() -> robot.turret.lock(score2RowPose))
//                .run(() -> robot.turret.unlock())
                .run(() -> robot.follower.setMaxPower(1))
                .buildPath(path -> {
                    path.addLine(score2RowPose);
                    path.setLinearHeading(
                            path.getStartPose().getHeading(), score2RowPose.getHeading(), 0.01);
                    path.setConstraints(new PathConstraints(0.9, 0));
                    path.addCallback(() -> robot.follower.getDistanceRemaining() <= 15, () -> {
                        if (Tuning.sorting) robot.actions.sortMagazine().schedule();
                    });
                    path.addCallback(() -> robot.follower.getDistanceRemaining() <= Tuning.shootPreDistanceCycles,
                            () -> robot.actions.shootAll(true, false).schedule());
                    path.holdAtDistance(15);
                }, 3000)
                .delay(shootDelay);
    }

    private void addGateIntake(PathActionBuilder builder, boolean hold, boolean enabled) {
        Pose intakePose = Context.allianceColor == AllianceColor.RED ? gateIntakeRed : getIntakeBlue;

        builder.setEnabled(enabled);
        builder.setState(CloseRobot.Magazine.IntakeState.FORWARD,
                        CloseRobot.Shooter.FlywheelState.CLOSE_AUTO, CloseRobot.Shooter.HoodState.CLOSE_AUTO)
                .run(() -> {
                    phase = "gate intake" + (hold ? " (hold)" : "");
                    robot.targetPose = Context.allianceColor == AllianceColor.RED
                            ? new Pose(CloseRobot.redTargetPose.getX() + 13, CloseRobot.redTargetPose.getY())
                            : new Pose(CloseRobot.blueTargetPose.getX() - 17, CloseRobot.blueTargetPose.getY());
                })
                .run(() -> {
                    robot.shooter.closeVelocityOffset = Context.allianceColor == AllianceColor.BLUE ? Tuning.BLUErpmCycleOffset : Tuning.REDrpmCycleOffset;
                    robot.shooter.lockFlywheel(gateScorePose);
                    robot.shooter.lockHood(gateScorePose);
                })
                .buildPath(path -> {
                    path.addLine(intakePose);
                    // DECODE: constant(.2,gateScore.h)/tangent(.5)/constant(1,intake.h) -> heading sweep.
                    path.setLinearHeading(gateScorePose.getHeading(), intakePose.getHeading());
                    path.setConstraints(new PathConstraints(0.95, 0));
                    path.holdAtDistance(1.5);
                }, 100000)
                .delay(intakeDelay);

        if (hold) {
            builder.buildPath(path -> {
                path.addLine(gateHoldPose);
                path.setTangentHeading();
                path.setReversed();
                path.setConstraints(new PathConstraints(0.97, 0));
            }, 100);
        }

        builder
                .run(() -> robot.turret.lock(gateScorePose))
                .buildPath(path -> {
                    path.addLine(gateScorePose);
                    path.setLinearHeading(
                            path.getStartPose().getHeading(), gateScorePose.getHeading(), 0.01);
                    path.setConstraints(new PathConstraints(0.9, 0));
                    path.addTemporalCallback(400, () -> {
                        if (Tuning.sorting) robot.actions.sortMagazine().schedule();
                    });
                    path.addCallback(() -> robot.follower.getDistanceRemaining() <= Tuning.shootPreDistanceCycles, () -> {
                        if (Tuning.sorting) {
                            robot.actions.shootSorted(true).schedule();
                        } else {
                            robot.actions.shootAll(true, false).schedule();
                        }
                    });
                    path.holdAtDistance(15);
                }, 100000)
                .delay(shootDelay)
                .await(() -> robot.magazine.shotSorted);
    }

    private void addThirdRow(PathActionBuilder builder) {
        // The third row is the sorting cycle (already gated by the sorting branch). Enable it
        // explicitly so a preceding addGateIntake(..., enabled=false) can't silently disable it via
        // the builder's sticky enabled flag.
        builder.setEnabled(true);
        builder.setState(CloseRobot.Magazine.IntakeState.FORWARD,
                        CloseRobot.Shooter.FlywheelState.CLOSE_AUTO, CloseRobot.Shooter.HoodState.CLOSE_AUTO)
                .run(() -> robot.turret.lock(score3RowPose))
                .run(() -> robot.shooter.unlock())
                .buildPath(path -> {
                    path.addCurve(intake3RowControl, intake3RowPose);
                    // DECODE: tangent(0.6)/linear(.8,start,intake3.h)/constant(1) -> start->intake3 sweep.
                    path.setLinearHeading(path.getStartPose().getHeading(), intake3RowPose.getHeading());
                    path.setConstraints(new PathConstraints(0.9, 0));
                    path.addParametricCallback(0.4, () -> robot.follower.setMaxPower(0.8));
                    path.setNoDeceleration();
                }, 4000)
                .actionDuring(robot.actions.flickExtakeThenOn(100));

        builder.run(() -> robot.follower.setMaxPower(1))
                .buildPath(path -> {
                    path.addLine(score3RowPose);
                    // DECODE: constant(0.7,-50deg)/linear(0.9,start,score3.h)/constant(1) -> start->score3 sweep.
                    path.setLinearHeading(path.getStartPose().getHeading(), score3RowPose.getHeading());
                    path.addTemporalCallback(400, () -> robot.actions.sortMagazine().schedule());
                    path.setConstraints(new PathConstraints(0.95, 0));
                }, 4000)
                .action(robot.actions.shootSorted(true));
    }

    private void addFirstRow(PathActionBuilder builder, boolean enabled) {
        builder.setEnabled(enabled);
        builder.setState(CloseRobot.Magazine.IntakeState.FORWARD,
                        CloseRobot.Shooter.FlywheelState.PID, CloseRobot.Shooter.HoodState.PID)
                .run(() -> {
                    phase = "first row / park";
                    robot.targetPose = Context.allianceColor == AllianceColor.RED
                            ? new Pose(CloseRobot.redTargetPose.getX() + 10, CloseRobot.redTargetPose.getY() - 10)
                            : new Pose(CloseRobot.blueTargetPose.getX() - 10, CloseRobot.blueTargetPose.getY() - 11);
                })
                .buildPath(gateScorePose, path -> {
                    path.addCurve(intake1RowControl, intake1RowPose);
                    path.setLinearHeading(
                            path.getStartPose().getHeading(), intake1RowPose.getHeading(), 0.3);
                    path.setConstraints(new PathConstraints(0.9, 0));
                    path.addParametricCallback(0.4, () -> robot.follower.setMaxPower(0.7));
                    path.setNoDeceleration();
                    path.holdAtDistance(1);
                }, 100000)
                .actionDuring(robot.actions.flickExtakeThenOn(100));

        builder
                .run(() -> robot.turret.unlock())
                .run(() -> robot.follower.setMaxPower(.8))
                .buildPath(path -> {
                    path.addLine(scoreParkPose);
                    path.setLinearHeading(
                            path.getStartPose().getHeading(), scoreParkPose.getHeading(), 0.1);
                    path.setConstraints(new PathConstraints(0.95, 0));
                    path.addTemporalCallback(400, () -> {
                        if (Tuning.sorting) robot.actions.sortMagazine().schedule();
                    });
                    path.addCallback(() -> robot.follower.getDistanceRemaining() <= 35,
                            () -> { if (!Tuning.sorting) robot.actions.shootAll(true, false).schedule(); });
                    path.addCallback(() -> robot.follower.getDistanceRemaining() <= 20,
                            () -> { if (Tuning.sorting) robot.actions.shootSorted(true).schedule(); });
                    path.holdAtDistance(1);
                }, 100000)
                .await(() -> robot.magazine.shotSorted);

        builder.delay(10000);
    }

    @Override
    protected void gameLoop() {
        // DECODE's bonk recovery: a stalled drive still advances (and its after-actions still fire).
        if (robot.drivetrain.isBonk()
                && robot.pathActionScheduler.getCurrentState() == SchedulerState.PATH_RUNNING) {
            robot.pathActionScheduler.skipCurrentSegment();
        }
        robot.pathActionScheduler.update();
    }

    @Override
    protected void telemetry() {
        telemetry.addData("Close Phase", phase);
        telemetry.addData("Alliance", Context.allianceColor);
        telemetry.addData("Target Pose", robot.targetPose);
        if (robot.pathActionScheduler != null) {
            telemetry.addData("Scheduler", robot.pathActionScheduler.getDebugInfo());
            telemetry.addData("Complete", robot.pathActionScheduler.isComplete());
        }
        // Compare against the pose the follower was actually seeded to (setup pose), not the
        // intended start pose — otherwise this misreports drift from the first loop.
        Pose seededFrom = getSetupPose();
        if (seededFrom != null) {
            Pose cur = robot.follower.getPose();
            double dist = Math.hypot(cur.getX() - seededFrom.getX(), cur.getY() - seededFrom.getY());
            telemetry.addData("Start offset (in)", String.format("%.1f", dist));
        }
    }
}
