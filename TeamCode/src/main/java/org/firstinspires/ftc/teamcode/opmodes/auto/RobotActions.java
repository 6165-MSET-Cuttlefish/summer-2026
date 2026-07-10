package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.architecture.action.Action;
import org.firstinspires.ftc.teamcode.architecture.action.Actions;

import org.firstinspires.ftc.teamcode.opmodes.auto.CloseRobot.Magazine;

/**
 * Hardware-free stand-in for DECODE's {@code RobotActions} factory, scoped to what the ported
 * "Close" auto invokes. Each factory returns a summer-2026 {@link Action} built from
 * {@link Actions#builder()}; the steps drive the stub {@link Magazine}'s State machine and preserve
 * the original DECODE delays/waits, but command no hardware.
 *
 * <p>Call-site note: DECODE scheduled these with {@code action.run()}. The summer-2026 cooperative
 * scheduler's verb is {@link Action#schedule()} — the ported {@code Close} uses {@code .schedule()}
 * at every site that DECODE used {@code .run()}. Behaviour is the same (fire-and-forget; conflicting
 * actions sharing a target Module are cancelled).
 */
public class RobotActions {
    private final CloseRobot robot;

    public RobotActions(CloseRobot robot) {
        this.robot = robot;
    }

    /**
     * Full burst shot. DECODE: intake OFF + vertical ON, lazy back-pusher SHOOT, delay 100, optional
     * +400 first-ball delay, intake SHOOTING, delay ~1000 to feed, then (if reset) back to intaking.
     */
    public Action shootAll(boolean resetAfterShoot, boolean firstBallDelay) {
        return Actions.builder()
                // Block the between-cycle await(shotSorted) until this shot completes. Without this,
                // shotSorted stays true (only shootSorted() ever cleared it) so the next segment
                // races the in-flight shot in the default (non-sorting) path.
                .run(() -> robot.magazine.shotSorted = false)
                .set(Magazine.IntakeState.OFF, Magazine.VerticalState.ON)
                .set(Magazine.HorizontalBackState.OPEN_SHOOT)
                .delay(100)
                .ifThen(() -> firstBallDelay, Actions.delay(400))
                .set(Magazine.IntakeState.SHOOTING)
                .delay(1000)
                .ifThen(() -> resetAfterShoot, Actions.builder()
                        .set(Magazine.IntakeState.FORWARD,
                                Magazine.VerticalState.HALF_DOWN,
                                Magazine.HorizontalBackState.OPEN)
                        .build())
                .run(() -> robot.magazine.shotSorted = true)
                .withName("shootAll")
                .build();
    }

    /**
     * DECODE: if nothing is staged, delegates to shootAll; else shoots the staged/sorted balls in
     * two waves. Mock preserves the two-wave delays and clears/sets {@code shotSorted}.
     */
    public Action shootSorted(boolean resetAfterShoot) {
        return Actions.builder()
                .run(() -> robot.magazine.shotSorted = false)
                .set(Magazine.IntakeState.OFF, Magazine.VerticalState.ON)
                .set(Magazine.HorizontalBackState.OPEN_SHOOT)
                .delay(100)
                .set(Magazine.IntakeState.SHOOTING)
                .delay(300)
                .set(Magazine.HorizontalFrontState.OPEN,
                        Magazine.HorizontalBackState.OPEN_SHOOT,
                        Magazine.IntakeState.OFF)
                .delay(100)
                .set(Magazine.IntakeState.SHOOTING)
                .delay(800)
                .ifThen(() -> resetAfterShoot, Actions.builder()
                        .set(Magazine.IntakeState.IDLE,
                                Magazine.VerticalState.HALF_DOWN,
                                Magazine.HorizontalBackState.OPEN)
                        .build())
                .run(() -> robot.magazine.shotSorted = true)
                .withName("shootSorted")
                .build();
    }

    /** DECODE: stage balls into back/front lanes per the motif, then resume intaking. */
    public Action sortMagazine() {
        return Actions.builder()
                .set(Magazine.IntakeState.IDLE)
                .delay(400)
                .set(Magazine.IntakeState.FORWARD)
                .withName("sortMagazine")
                .build();
    }

    /** DECODE: reverse-flick the intake to reseat a ball, then resume forward. */
    public Action flickExtakeThenOn(long delay) {
        return Actions.builder()
                .set(Magazine.IntakeState.AUTO_EXTAKE)
                .delay(delay)
                .set(Magazine.IntakeState.FORWARD)
                .withName("flickExtakeThenOn")
                .build();
    }

    /**
     * DECODE: continuously polls the Limelight for the obelisk/motif. Vision is Decode-specific and
     * absent here, so the mock is a no-op placeholder kept for sequence fidelity (sorting branch).
     */
    public Action detectObeliskLoop() {
        return Actions.builder()
                .run(() -> { /* no Limelight in the pre-season scaffold */ })
                .withName("detectObeliskLoop")
                .build();
    }
}
