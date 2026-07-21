package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.architecture.auto.FieldPose.forAlliance;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.blueTargetPose;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.redTargetPose;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.architecture.action.Actions;
import org.firstinspires.ftc.teamcode.architecture.auto.scheduler.HeadingInterpolatorBuilder;
import org.firstinspires.ftc.teamcode.architecture.auto.scheduler.PathActionBuilder;
import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.decode.DecodeAuto;
import org.firstinspires.ftc.teamcode.modules.Magazine;
import org.firstinspires.ftc.teamcode.modules.Shooter;

// Deliberately not @Autonomous: CloseRed/CloseBlue are the entry points, and annotating this base
// would register a duplicate RED auto that looks alliance-neutral to a BLUE driver.
@Config
public class Close extends DecodeAuto {
    public static boolean sorting = false;
    public static boolean runSecondRow = true;
    public static boolean runGateIntake = true;
    public static boolean runFirstRow = true;

    public static boolean partnerShooting = true;


    private int intakeDelay = 2000;
    private int shootDelay = 600;
    public static double shootPreDistanceCycles = 14;

    private final double offsetStartPoseX = 0;
    private final double offsetStartPoseY = 0;

    public static double REDrpmCycleOffset = -30;
    public static double BLUErpmCycleOffset = -100;

    /** Alliance pinned before the mirrored poses and {@code createRobot()} build; overridden by {@link CloseRed} / {@link CloseBlue}. */
    protected AllianceColor alliance() {
        return AllianceColor.RED;
    }

    // Must run before the mirrored Pose fields below, so forAlliance() and the later
    // createRobot()/setTargetPosesForAlliance() all observe the same alliance.
    {
        Context.allianceColor = alliance();
    }

    private final Pose setupPose = forAlliance(110.375 + offsetStartPoseX, 109.75 + offsetStartPoseY, Math.toRadians(0));
    private final Pose startPose = forAlliance(121.1 + offsetStartPoseX, 119.7 + offsetStartPoseY, Math.toRadians(44.6));
    private final Pose sortStartPose = forAlliance(121.1 + offsetStartPoseX, 119.7 + offsetStartPoseY, Math.toRadians(134.6));

    private final Pose preloadScorePose = forAlliance(72 + offsetStartPoseX, 72 + offsetStartPoseY, Math.toRadians(44.6));

    private final Pose intake2RowRed = forAlliance(134 + offsetStartPoseX, 61 + offsetStartPoseY, Math.toRadians(5));
    private final Pose intake2RowBlue = forAlliance(134 + offsetStartPoseX, 63.5 + offsetStartPoseY, Math.toRadians(5));
    private final Pose intake2RowControl1 = forAlliance(70 + offsetStartPoseX, 75 + offsetStartPoseY, Math.toRadians(0));
    private final Pose intake2RowControl2 = forAlliance(88 + offsetStartPoseX, 56 + offsetStartPoseY, Math.toRadians(0));
    private final Pose intake2RowControl3Blue = forAlliance(125 + offsetStartPoseX, 53 + offsetStartPoseY, Math.toRadians(0));
    private final Pose score2RowPose = forAlliance(85.0 + offsetStartPoseX, 76 + offsetStartPoseY, Math.toRadians(-20));

    private final Pose gateIntakeRed = forAlliance(132 + offsetStartPoseX, 58 + offsetStartPoseY, Math.toRadians(28));
    private final Pose getIntakeBlue = forAlliance(132.5 + offsetStartPoseX, 58.5 + offsetStartPoseY, Math.toRadians(15));

    private final Pose gateHoldPose = forAlliance(127.5 + offsetStartPoseX, 60.0 + offsetStartPoseY, Math.toRadians(-10));
    private final Pose gateScorePose = forAlliance(85.0 + offsetStartPoseX, 76 + offsetStartPoseY, Math.toRadians(-20));

    private final Pose intake3RowPose = forAlliance(130 + offsetStartPoseX, 36 + offsetStartPoseY, Math.toRadians(0));
    private final Pose intake3RowControl = forAlliance(84 + offsetStartPoseX, 39 + offsetStartPoseY, Math.toRadians(0));
    private final Pose score3RowPose = forAlliance(85.0 + offsetStartPoseX, 76.5 + offsetStartPoseY, Math.toRadians(-20));

    private final Pose intake1RowPose = forAlliance(127 + offsetStartPoseX, 84 + offsetStartPoseY, Math.toRadians(0));
    private final Pose intake1RowControl = forAlliance(103 + offsetStartPoseX, 89.0 + offsetStartPoseY, Math.toRadians(0));
    private final Pose scoreParkPose = forAlliance(86.5 + offsetStartPoseX, 110 + offsetStartPoseY, Math.toRadians(-25));

    @Override
    protected Pose getSetupPose() {
        return setupPose;
    }

    @Override
    protected Pose getStartPose() {
        return sorting? sortStartPose : startPose;
    }

    @Override
    protected void buildAutonomousSequence() {
        PathActionBuilder builder = new PathActionBuilder(robot.follower, () -> (long) getGameTimer().milliseconds());
        builder.setStartPose(robot.follower.getPose());

        addPreloadSecondRow(builder, runSecondRow);
        addGateIntake(builder, false, runGateIntake);
        if (sorting) {
            addThirdRow(builder);
        } else {
            addGateIntake(builder, false, runGateIntake);
            addGateIntake(builder, true, runGateIntake);
            if (partnerShooting) {
                addGateIntake(builder, true, runGateIntake);
                intakeDelay = 1000;
            }
        }
        addFirstRow(builder, runFirstRow);


        robot.pathActionScheduler = builder.build();
    }

    private void addPreloadSecondRow(PathActionBuilder builder, boolean enabled) {
        Pose intake2Pose = Context.allianceColor == AllianceColor.RED ? intake2RowRed : intake2RowBlue;

        builder.setEnabled(enabled);
        builder.setState(Magazine.IntakeState.IDLE, Shooter.HoodState.PID)
                .run(() -> robot.targetPose = Context.allianceColor == AllianceColor.RED
                        ? new Pose(redTargetPose.getX() + 10, redTargetPose.getY())
                        : new Pose(blueTargetPose.getX() - 15, blueTargetPose.getY()))
                .run(() -> robot.turret.lock(preloadScorePose))
                .run(() ->
                        Actions.builder()
                                .delay(sorting? 150 : 0)
                                .set(Shooter.FlywheelState.CLOSE_AUTO_PRELOAD)
                                .waitUntil(() -> robot.shooter.getCurrentVelocityRPM() > 1000)
                                .run(() -> robot.actions.shootAll(true, false).schedule())
                                .build().schedule()
                )
                .run(() -> robot.targetPose = Context.allianceColor == AllianceColor.RED
                        ? new Pose(redTargetPose.getX() + 18, redTargetPose.getY())
                        : new Pose(blueTargetPose.getX() - 17, blueTargetPose.getY()))
                .run(() -> {
                    robot.shooter.closeVelocityOffset = Context.allianceColor.equals(AllianceColor.BLUE)? BLUErpmCycleOffset : REDrpmCycleOffset;
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
                    if (sorting) {
                        path.setHeadingInterpolation(new HeadingInterpolatorBuilder()
                                .constant(.3, path.getStartPose().getHeading())
                                .linear(.4, preloadScorePose.getHeading(), intake2Pose.getHeading())
                                .constant(1, intake2Pose.getHeading())
                                .build());
                        path.addParametricCallback(0, () -> robot.actions.detectObeliskLoop().schedule());
                    } else {
                        if (Context.allianceColor == AllianceColor.RED) {
                            path.setHeadingInterpolation(new HeadingInterpolatorBuilder()
                                    .constant(.4, path.getStartPose().getHeading())
                                    .linear(.5, preloadScorePose.getHeading(), intake2Pose.getHeading())
                                    .constant(1, intake2Pose.getHeading())
                                    .build());
                        } else {
                            path.setHeadingInterpolation(new HeadingInterpolatorBuilder()
                                    .constant(.35, path.getStartPose().getHeading())
                                    .linear(.45, preloadScorePose.getHeading(), intake2Pose.getHeading())
                                    .constant(1, intake2Pose.getHeading())
                                    .build());
                        }
                        path.addParametricCallback(0, () -> robot.follower.setMaxPower(.59));
                        path.addParametricCallback(.3, () -> robot.follower.setMaxPower(1));
                    }
                    path.addParametricCallback(0.5, () ->
                            Actions.builder()
                                    .set(Magazine.VerticalState.OFF, Magazine.IntakeState.FORWARD)
                                    .build().schedule());
                    path.addParametricCallback(.68, () -> robot.follower.setMaxPower(0.4));
                    path.addParametricCallback(.92, () -> robot.follower.setMaxPower(1));
                    path.holdAtDistance(15);
                }, 5000);

        builder
                .run(() -> robot.turret.lock(score2RowPose))
                .run(() -> robot.follower.setMaxPower(1))
                .buildPath(path -> {
                    path.addLine(score2RowPose);
                    path.setLinearHeading(
                            path.getStartPose().getHeading(), score2RowPose.getHeading(), 0.01);
                    path.addCallback(() -> robot.follower.getDistanceRemaining() <= 15, () -> {
                        if (sorting) robot.actions.sortMagazine().schedule();
                    });
                    path.addCallback(() -> robot.follower.getDistanceRemaining() <= shootPreDistanceCycles, () -> robot.actions.shootAll(true, false).schedule());
                    path.holdAtDistance(15);
                }, 3000)
                .delay(shootDelay);
    }

    private void addGateIntake(PathActionBuilder builder, boolean hold, boolean enabled) {
        Pose intakePose = Context.allianceColor == AllianceColor.RED ? gateIntakeRed : getIntakeBlue;

        builder.setEnabled(enabled);
        builder.setState(Magazine.IntakeState.FORWARD, Shooter.FlywheelState.CLOSE_AUTO, Shooter.HoodState.CLOSE_AUTO)
                .run(() -> robot.targetPose = Context.allianceColor == AllianceColor.RED
                        ? new Pose(redTargetPose.getX() + 13, redTargetPose.getY())
                        : new Pose(blueTargetPose.getX() - 17, blueTargetPose.getY()))
                .run(() -> {
                    robot.shooter.closeVelocityOffset = Context.allianceColor.equals(AllianceColor.BLUE)? BLUErpmCycleOffset : REDrpmCycleOffset;
                    robot.shooter.lockFlywheel(gateScorePose);
                    robot.shooter.lockHood(gateScorePose);
                })

                .buildPath(path -> {
                    path.addLine(intakePose);
                    path.setHeadingInterpolation(new HeadingInterpolatorBuilder()
                            .constant(.2, gateScorePose.getHeading())
                            .tangent(.5)
                            .constant(1, intakePose.getHeading())
                            .build());
                    path.holdAtDistance(1.5);
                }, 100000)
                .delay(intakeDelay);

        if (hold) {
            builder.buildPath(path -> {
                path.addLine(gateHoldPose);
                path.setTangentHeading();
                path.setReversed();
            }, 100);
        }

        builder
                .run(() -> robot.turret.lock(gateScorePose))
                .buildPath(path -> {
                    path.addLine(gateScorePose);
                    path.setLinearHeading(
                            path.getStartPose().getHeading(), gateScorePose.getHeading(), 0.01);
                    path.addTemporalCallback(400, () -> {
                        if (sorting) robot.actions.sortMagazine().schedule();
                    });
                    path.addCallback(() -> robot.follower.getDistanceRemaining() <= shootPreDistanceCycles, () -> {
                        if (sorting) {
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
        builder.setState(Magazine.IntakeState.FORWARD, Shooter.FlywheelState.CLOSE_AUTO, Shooter.HoodState.CLOSE_AUTO)
                .run(() -> robot.turret.lock(score3RowPose))
                .run(() -> robot.shooter.unlock())

                .buildPath(path -> {
                    path.addCurve(intake3RowControl, intake3RowPose);
                    path.setHeadingInterpolation(new HeadingInterpolatorBuilder()
                            .tangent(0.6)
                            .linear(.8, path.getStartPose().getHeading(), intake3RowPose.getHeading())
                            .constant(1, intake3RowPose.getHeading())
                            .build());
                    path.addParametricCallback(0.4, () -> robot.follower.setMaxPower(0.8));
                    path.setNoDeceleration();
                }, 4000)
                .actionDuring(robot.actions.flickExtakeThenOn(100));

        builder.run(() -> robot.follower.setMaxPower(1))
                .buildPath(path -> {
                    path.addLine(score3RowPose);
                    path.setHeadingInterpolation(new HeadingInterpolatorBuilder()
                            .constant(0.7, Math.toRadians(-50))
                            .linear(0.9, path.getStartPose().getHeading(), score3RowPose.getHeading())
                            .constant(1, score3RowPose.getHeading())
                            .build());
                    path.addTemporalCallback(400, () -> robot.actions.sortMagazine().schedule());
                }, 4000)
                .action(robot.actions.shootSorted(true));
    }

    private void addFirstRow(PathActionBuilder builder, boolean enabled) {
        builder.setEnabled(enabled);
        builder.setState(Magazine.IntakeState.FORWARD, Shooter.FlywheelState.PID, Shooter.HoodState.PID)
                .run(() -> robot.targetPose = Context.allianceColor == AllianceColor.RED
                        ? new Pose(redTargetPose.getX() + 10, redTargetPose.getY() - 10)
                        : new Pose(blueTargetPose.getX() - 10, blueTargetPose.getY() - 11))
                .buildPath(gateScorePose, path -> {
                    path.addCurve(intake1RowControl, intake1RowPose);
                    path.setLinearHeading(
                            path.getStartPose().getHeading(), intake1RowPose.getHeading(), 0.3);
                    path.addParametricCallback(0.4, () -> robot.follower.setMaxPower(0.7));
                    path.setNoDeceleration();
                    path.holdAtDistance(1);
                }, 100000)
                .actionDuring(robot.actions.flickExtakeThenOn(100));
        ;

        builder
                .run(() -> robot.turret.unlock())
                .run(() -> robot.follower.setMaxPower(.8))
                .buildPath(path -> {
                    path.addLine(scoreParkPose);
                    path.setLinearHeading(
                            path.getStartPose().getHeading(), scoreParkPose.getHeading(), 0.1);
                    path.addTemporalCallback(400, () -> {
                        if (sorting) robot.actions.sortMagazine().schedule();
                    });
                    path.addCallback(() -> robot.follower.getDistanceRemaining() <= 35, () -> {if (!sorting) robot.actions.shootAll(true, false).schedule();});
                    path.addCallback(() -> robot.follower.getDistanceRemaining() <= 20, () -> {if (sorting) robot.actions.shootSorted(true).schedule();});
                    path.holdAtDistance(1);
                }, 100000)
                .await(() -> robot.magazine.shotSorted);

        builder.delay(10000);
    }
}
