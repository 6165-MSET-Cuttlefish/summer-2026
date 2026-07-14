package org.firstinspires.ftc.teamcode.decode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.architecture.action.Action;
import org.firstinspires.ftc.teamcode.architecture.action.Actions;
import org.firstinspires.ftc.teamcode.modules.Endgame;
import org.firstinspires.ftc.teamcode.modules.Magazine;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Turret;

public class RobotActions {
    private final DecodeRobot robot;

    private Action endgameAction;

    public RobotActions(DecodeRobot robot) {
        this.robot = robot;
    }

    private Magazine.HorizontalBackState backShootState() {
        Magazine.HorizontalBackState state = robot.magazine.getState(Magazine.HorizontalBackState.class);
        if (state == Magazine.HorizontalBackState.OPEN) return Magazine.HorizontalBackState.OPEN_SHOOT;
        if (state == Magazine.HorizontalBackState.STORED) return Magazine.HorizontalBackState.STORED_SHOOT;
        return state;
    }

    private Magazine.HorizontalBackState backResetState() {
        Magazine.HorizontalBackState state = robot.magazine.getState(Magazine.HorizontalBackState.class);
        if (state == Magazine.HorizontalBackState.OPEN_SHOOT) return Magazine.HorizontalBackState.OPEN;
        if (state == Magazine.HorizontalBackState.STORED_SHOOT) return Magazine.HorizontalBackState.STORED;
        return state;
    }

    public Action prepareShooterFar() {
        return Actions.set(Shooter.FlywheelState.FAR_AUTO, Shooter.HoodState.FAR_AUTO);
    }

    public Action prepareShooterClose() {
        return Actions.set(Shooter.FlywheelState.PID);
    }

    public Action intakeUntilFull() {
        return Actions.builder()
                .set(Magazine.IntakeState.FORWARD, Magazine.VerticalState.HALF_DOWN)
                .waitUntil(() -> robot.magazine.intakeIsFull)
                .set(Magazine.IntakeState.IDLE)
                .build();
    }

    public Action intakeOn() {
        return Actions.builder()
                .set(Magazine.IntakeState.FORWARD)
                .build();
    }

    public Action shootWhenReady() {
        return Actions.builder()
                .waitUntil(() -> robot.shooter.isAtTargetVelocity())
                .set(Magazine.VerticalState.ON)
                .build();
    }

    public Action shootAll(boolean resetAfterShoot, boolean firstBallDelay) {
        return Actions.builder()
//                .set(Turret.TurretState.AUTOAIM)
                .set(Magazine.IntakeState.OFF, Magazine.VerticalState.ON)

                .setLazy(this::backShootState)
                .delay(100)
                .delay(firstBallDelay ? 400 : 0)
                .set(Magazine.IntakeState.SHOOTING)
                .delay(firstBallDelay ? 1100 : 1000)
                .run(() -> {
                    if (resetAfterShoot) {
                        Magazine.IntakeState.FORWARD.activate();
                        Magazine.VerticalState.HALF_DOWN.activate();
                        backResetState().activate();
                    }
                })
                .build();
    }

    public Action shootSorted(boolean resetAfterShoot) {
        return Actions.builder()
                .ifElse(
                        () -> robot.magazine.getState(Magazine.HorizontalFrontState.class) == Magazine.HorizontalFrontState.OPEN
                                && robot.magazine.getState(Magazine.HorizontalBackState.class) == Magazine.HorizontalBackState.OPEN,
                        shootAll(resetAfterShoot, true),
                        Actions.builder()
                                .run(() -> robot.magazine.shotSorted = false)
                                .set(Magazine.IntakeState.OFF, Magazine.VerticalState.ON)
                                .setLazy(this::backShootState)
                                .delay(100)
                                .set(Magazine.IntakeState.SHOOTING)
                                .delay(300)
                                .set(Magazine.HorizontalFrontState.OPEN, Magazine.HorizontalBackState.OPEN_SHOOT, Magazine.IntakeState.OFF)
                                .delay(100)
                                .set(Magazine.IntakeState.SHOOTING)
                                .delay(800)
                                .run(() -> {
                                    if (resetAfterShoot) {
                                        Magazine.IntakeState.IDLE.activate();
                                        Magazine.VerticalState.HALF_DOWN.activate();
                                        Magazine.HorizontalBackState.OPEN.activate();
                                    }
                                })
                                .run(() -> robot.magazine.shotSorted = true)
                                .build()
                )
                .build();
    }

    public Action intakeOffThenOn(long delay) {
        return Actions.builder()
                .set(Magazine.IntakeState.OFF)
                .delay(delay)
                .set(Magazine.IntakeState.FORWARD)
                .build();
    }


    public Action flickExtakeThenOn(long delay) {
        return Actions.builder()
                .set(Magazine.IntakeState.AUTO_EXTAKE)
                .delay(delay)
                .set(Magazine.IntakeState.FORWARD)
                .build();
    }

    public Action flickExtakeThenOffThenOn(long delay1, long delay2) {
        return Actions.builder()
                .delay(50)
                .set(Magazine.IntakeState.AUTO_EXTAKE)
                .delay(delay1)
                .set(Magazine.IntakeState.OFF)
                .delay(delay2)
                .set(Magazine.IntakeState.FORWARD)
                .build();
    }

    public Action shootAllPreloadMoving() {
        return Actions.builder()
                .waitUntil(() -> robot.shooter.isWithinLUTRange())
                .set(Magazine.IntakeState.IDLE, Magazine.VerticalState.ON, Magazine.HorizontalBackState.OPEN_SHOOT)
                .delay(600)
                .set(Magazine.HorizontalFrontState.OPEN_SHOOT)
                .delay(300)
                .set(Magazine.IntakeState.FORWARD, Magazine.VerticalState.OFF, Magazine.HorizontalBackState.OPEN, Magazine.HorizontalFrontState.OPEN)
                .build();
    }

    public Action sortMagazine() {
        final boolean[] sorted = {false};
        return Actions.builder()
                .set(Magazine.IntakeState.IDLE)
                .run(() -> {
                    Magazine.StorageDecision decision =
                            robot.magazine.checkAndStoreBalls(DecodeContext.motif);

                    sorted[0] = decision.storeInBack || decision.storeInFront;
                    if (!sorted[0]) return;

                    (decision.storeInBack
                            ? Magazine.HorizontalBackState.STORED
                            : Magazine.HorizontalBackState.OPEN).activate();
                    (decision.storeInFront
                            ? Magazine.HorizontalFrontState.STORED
                            : Magazine.HorizontalFrontState.OPEN).activate();
                })
                .stopIf(() -> !sorted[0])
                .delay(400)
                .set(Magazine.IntakeState.FORWARD)
                .build();
    }

    public Action endgameSequence() {
        endgameAction = Actions.builder()
                .set(Endgame.InitialState.LIFT)
                .waitUntil(() -> robot.endgame.initialLiftComplete()) // tune threshold
                .run(() -> {
                    robot.drivetrain.getFl().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.drivetrain.getFr().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.drivetrain.getFl().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.drivetrain.getFr().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                })
                .set(Endgame.LeftPtoState.DOWN, Endgame.RightPtoState.DOWN)
                .delay(300)
                .set(Endgame.InitialState.DISABLED)
                .set(Endgame.FullLiftState.FULL_LIFT)
                .build();
        return endgameAction;
    }

    public Action cancelEndgameSequence() {
        return Actions.builder()
                .run(() -> {
                    if (endgameAction != null) endgameAction.cancel();
                })
                .waitUntil(() -> endgameAction == null || endgameAction.isCancelled())
                .set(Endgame.InitialState.DISABLED, Endgame.FullLiftState.OFF)
                .run(() -> robot.drivetrain.setRawTargets(0, 0, 0, 0))
                .build();
    }

    public Action wiggleFrontHzPusher() {
        return Actions.builder()
                .set(Magazine.HorizontalFrontState.OPEN)
                .delay(125)
                .set(Magazine.HorizontalFrontState.OPEN_SHOOT)
                .delay(125)
                .set(Magazine.HorizontalFrontState.OPEN)
                .build();
    }

    private Action onePeriodShootAll() {
        return Actions.builder()
                .delay(150)
                .set(Magazine.HorizontalBackState.OPEN)
                .delay(150)
                .set(Magazine.HorizontalBackState.OPEN_SHOOT)
                .build();
    }

    public Action wiggleBackHzPusher() {
        return Actions.builder()
                .set(Magazine.HorizontalBackState.OPEN)
                .delay(125)
                .set(Magazine.HorizontalBackState.OPEN_SHOOT)
                .build();
    }


    public Action detectObeliskLoop() {
        return Actions.builder()
                .run(() -> robot.turret.detectingObelisk = true)
                .loop(
                        Actions.builder()
                                .run(() -> robot.turret.detectObelisk())
                                .build(),
                        () -> robot.turret.detectingObelisk
                )
                .build();
    }

    public Action detectObelisk() {
        return Actions.builder()
                .run(() -> robot.turret.detectObelisk())
                .build();
    }

    public Action autoLockSOTMDecel(Pose target, double velocity) {
        return Actions.builder()
                .loop(
                        Actions.builder().run(() -> {
                            double heading = target.getHeading();
                            double distRemaining = robot.follower.getDistanceRemaining();

                            // Linear decel: robot was at `velocity` at 10in out, stops at 0in
                            double currentVelocity = velocity * Math.min(1.0, distRemaining / 8.0);

                            // Predict where robot center will be when shot lands
                            double predictedX = target.getX() + Math.cos(heading) * currentVelocity * robot.turret.flightTime;
                            double predictedY = target.getY() + Math.sin(heading) * currentVelocity * robot.turret.flightTime;

                            robot.turret.lock(new Pose(predictedX, predictedY, heading));
                        }).build(),
                        () -> robot.follower.getDistanceRemaining() > 0
                )
                .build();
    }

    public Action autoLockSOTMAccel(Pose target, double maxVelocity) {
        return Actions.builder()
                .loop(
                        Actions.builder().run(() -> {
                            double heading = target.getHeading();
                            double distTraveled = robot.follower.getDistanceTraveledOnPath();


                            // Linear accel: robot starts at 0, reaches `maxVelocity` at 10in
                            double currentVelocity = maxVelocity * Math.min(1.0, distTraveled / 10.0);

                            // Predict where robot center will be when shot lands
                            double predictedX = target.getX() + Math.cos(heading) * currentVelocity * robot.turret.flightTime;
                            double predictedY = target.getY() + Math.sin(heading) * currentVelocity * robot.turret.flightTime;

                            robot.turret.lock(new Pose(predictedX, predictedY, heading));
                        }).build(),
                        () -> robot.follower.getDistanceTraveledOnPath() < 10
                )
                .build();
    }
}
