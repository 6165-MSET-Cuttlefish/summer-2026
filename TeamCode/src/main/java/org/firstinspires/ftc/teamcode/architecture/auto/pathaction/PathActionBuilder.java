package org.firstinspires.ftc.teamcode.architecture.auto.pathaction;

import androidx.annotation.CheckResult;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.Callable;
import java.util.function.Consumer;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.core.Action;
import org.firstinspires.ftc.teamcode.core.Module;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.State;

/**
 * Fluent builder for autonomous path-and-action sequences.
 *
 * <p>Typical usage:
 * <pre>{@code
 * builder.setStartPose(robot.follower.getPose())
 *        .setState(Magazine.IntakeState.IDLE)
 *        .buildPath(p -> p.addLine(scorePose))
 *        .action(robot.actions.shootAll());
 * }</pre>
 *
 * <p>Pending {@code setState}, {@code run}, and {@code actionAsync} calls are accumulated and
 * flushed into the prelude/during of the next real segment, so they cost zero extra ticks.
 */
public class PathActionBuilder {

    // ─── pending state, flushed to next segment ──────────────────────────────

    private final List<PathActionSegment> segments = new ArrayList<>();
    /** Keyed by state class so multiple state enums on the same module don't overwrite each other. */
    private final Map<Class<? extends State>, State> pendingStates = new LinkedHashMap<>();
    private final List<Runnable> pendingRunnables = new ArrayList<>();
    private final List<Action> pendingDuringActions = new ArrayList<>();

    private Pose lastPathEndPose = null;
    private boolean enabled = true;
    private boolean lastSegmentDisabled = false;

    private Callable<Boolean> overrideCondition = null;
    private Runnable overrideHandler = null;

    private final Follower follower;
    private final LongSupplier clock;

    public PathActionBuilder() {
        this(null, null);
    }

    public PathActionBuilder(Follower follower, LongSupplier clock) {
        this.follower = follower; // null = late-bound to Robot.robot.follower at use time
        this.clock = clock;
    }

    private Follower follower() {
        return follower != null ? follower : Robot.robot.follower;
    }

    // ─── config ──────────────────────────────────────────────────────────────

    public PathActionBuilder setStartPose(Pose startPose) {
        lastPathEndPose = startPose;
        return this;
    }

    /** Subsequent segments are skipped when {@code false}. Use to gate optional routines. */
    public PathActionBuilder setEnabled(boolean enabled) {
        if (!enabled) lastSegmentDisabled = true;
        this.enabled = enabled;
        return this;
    }

    /**
     * Queue states to be applied at the start of the next segment.
     * @throws IllegalStateException if a state has no registered module
     */
    public PathActionBuilder setState(State... states) {
        for (State state : states) {
            Module module = state.getModule();
            if (module == null) {
                throw new IllegalStateException(
                        "State " + state + " has no registered module. "
                        + "Register it via Module.setStates() before calling setState().");
            }
            pendingStates.put(state.getClass(), state);
        }
        return this;
    }

    // ─── paths ───────────────────────────────────────────────────────────────

    /** Linear transit to a target pose with the default 4-second path timeout. */
    public PathActionBuilder transitTo(Pose targetPose) {
        return transitTo(targetPose, 4000);
    }

    public PathActionBuilder transitTo(Pose targetPose, int pathTimeoutMs) {
        segments.add(new PathActionSegment.Builder()
                .transitTarget(targetPose)
                .pathTimeoutMs(pathTimeoutMs)
                .preludeRunnables(drainPendingRunnables())
                .duringActions(drainPendingDuringActions())
                .moduleStates(drainPendingStates())
                .enabled(enabled)
                .build());
        lastPathEndPose = copyOf(targetPose);
        return this;
    }

    /** Build a path starting from the last segment's end pose. */
    public PathActionBuilder buildPath(Consumer<IntegratedPathBuilder> body) {
        return buildPath(lastPathEndPose, body, null);
    }

    public PathActionBuilder buildPath(Consumer<IntegratedPathBuilder> body, int pathTimeoutMs) {
        return buildPath(lastPathEndPose, body, pathTimeoutMs);
    }

    public PathActionBuilder buildPath(Pose startPose, Consumer<IntegratedPathBuilder> body) {
        return buildPath(startPose, body, null);
    }

    public PathActionBuilder buildPath(Pose startPose, Consumer<IntegratedPathBuilder> body, int pathTimeoutMs) {
        return buildPath(startPose, body, Integer.valueOf(pathTimeoutMs));
    }

    private PathActionBuilder buildPath(Pose startPose, Consumer<IntegratedPathBuilder> body, Integer pathTimeoutMs) {
        Pose safeStart = Objects.requireNonNull(
                startPose, "startPose is required; call setStartPose() before buildPath");

        // If we skipped a disabled segment that moved elsewhere, transit back first.
        if (enabled && lastSegmentDisabled
                && lastPathEndPose != null
                && !posesApproxEqual(safeStart, lastPathEndPose)) {
            transitTo(safeStart);
            lastSegmentDisabled = false;
        }

        IntegratedPathBuilder pathBuilder = new IntegratedPathBuilder(safeStart, follower().pathBuilder());
        body.accept(pathBuilder);
        segments.add(buildPathSegment(
                pathBuilder.buildPath(),
                pathBuilder.getDuringActions(),
                pathBuilder.isHoldEnd(),
                pathTimeoutMs,
                pathBuilder.getHoldAtDistance()));
        return this;
    }

    /**
     * Schedule a path whose geometry is resolved at execution time. The body runs when
     * the segment starts, so it can read live sensor data to target a pose.
     *
     * @param declaredEndPose best-guess end pose, used only to seed the next segment's
     *                        default start pose; the actual path uses the live robot pose.
     */
    public PathActionBuilder buildPathDeferred(Pose declaredEndPose, Consumer<IntegratedPathBuilder> body) {
        if (enabled) lastPathEndPose = copyOf(declaredEndPose);

        Supplier<PathActionSegment.Resolved> resolver = () -> {
            IntegratedPathBuilder pb = new IntegratedPathBuilder(follower().getPose(), follower().pathBuilder());
            body.accept(pb);
            return new PathActionSegment.Resolved(pb.buildPath(), pb.getDuringActions(), pb.isHoldEnd());
        };

        segments.add(new PathActionSegment.Builder()
                .resolver(resolver)
                .preludeRunnables(drainPendingRunnables())
                .duringActions(drainPendingDuringActions())
                .moduleStates(drainPendingStates())
                .enabled(enabled)
                .build());
        return this;
    }

    // ─── actions ─────────────────────────────────────────────────────────────

    /** Run a blocking action. The scheduler waits for it to complete before advancing. */
    public PathActionBuilder action(Action action) {
        segments.add(buildActionSegment(Collections.singletonList(action)));
        return this;
    }

    /**
     * Queue a fire-and-forget action that launches at the start of the next real segment
     * (path, await, delay, or blocking action) with zero loop overhead. Despite the name,
     * the action is <em>not</em> dispatched immediately — it's stored and emitted with
     * the next segment's prelude.
     */
    public PathActionBuilder actionAsync(Action action) {
        pendingDuringActions.add(action);
        return this;
    }

    /**
     * Queue synchronous code to run at the start of the next real segment, alongside any
     * pending {@code setState()} and {@code actionAsync()} calls.
     */
    public PathActionBuilder run(Runnable code) {
        pendingRunnables.add(code);
        return this;
    }

    // ─── waiting ─────────────────────────────────────────────────────────────

    /** Wait up to {@code timeoutMs} for {@code condition} to return true. */
    public PathActionBuilder await(Callable<Boolean> condition, int timeoutMs) {
        segments.add(new PathActionSegment.Builder()
                .continueConditions(Collections.singletonList(condition))
                .continueMode(ConditionMode.OR)
                .timeoutMs(timeoutMs)
                .preludeRunnables(drainPendingRunnables())
                .duringActions(drainPendingDuringActions())
                .moduleStates(drainPendingStates())
                .enabled(enabled)
                .build());
        return this;
    }

    /** Wait indefinitely for {@code condition} to return true. */
    public PathActionBuilder await(Callable<Boolean> condition) {
        return await(condition, Integer.MAX_VALUE);
    }

    /** Pause for {@code delayMs} before advancing. */
    public PathActionBuilder delay(int delayMs) {
        segments.add(new PathActionSegment.Builder()
                .preludeRunnables(drainPendingRunnables())
                .duringActions(drainPendingDuringActions())
                .moduleStates(drainPendingStates())
                .timeoutMs(delayMs)
                .enabled(enabled)
                .build());
        return this;
    }

    // ─── overrides ───────────────────────────────────────────────────────────

    /** Register a condition that, when true, cancels the sequence and runs {@code handler}. */
    public PathActionBuilder setOverride(Callable<Boolean> condition, Runnable handler) {
        this.overrideCondition = condition;
        this.overrideHandler = handler;
        return this;
    }

    /** Shorthand for a time-based override using the OpMode game timer. */
    public PathActionBuilder setTimeOverride(int timeMs, Runnable handler) {
        return setOverride(() -> Robot.robot.opMode.getGameTimer().milliseconds() >= timeMs, handler);
    }

    // ─── build ───────────────────────────────────────────────────────────────

    @CheckResult
    public PathActionScheduler build() {
        // Trailing flush: anything still pending becomes a final no-op segment.
        if (!pendingRunnables.isEmpty() || !pendingDuringActions.isEmpty() || !pendingStates.isEmpty()) {
            segments.add(new PathActionSegment.Builder()
                    .preludeRunnables(drainPendingRunnables())
                    .duringActions(drainPendingDuringActions())
                    .moduleStates(drainPendingStates())
                    .enabled(enabled)
                    .build());
        }
        PathActionScheduler scheduler = new PathActionScheduler(follower, clock);
        for (int i = 0; i < segments.size(); i++) scheduler.addSegment(segments.get(i));
        if (overrideCondition != null) scheduler.setOverride(overrideCondition, overrideHandler);
        return scheduler;
    }

    // ─── helpers ─────────────────────────────────────────────────────────────

    private List<State> drainPendingStates() {
        List<State> out = new ArrayList<>(pendingStates.values());
        pendingStates.clear();
        return out;
    }

    private List<Runnable> drainPendingRunnables() {
        List<Runnable> out = new ArrayList<>(pendingRunnables);
        pendingRunnables.clear();
        return out;
    }

    private List<Action> drainPendingDuringActions() {
        List<Action> out = new ArrayList<>(pendingDuringActions);
        pendingDuringActions.clear();
        return out;
    }

    private PathActionSegment buildPathSegment(
            PathChain path, List<Action> duringActions, boolean holdEnd,
            Integer pathTimeoutMs, Double holdAtDistance) {
        if (enabled) lastPathEndPose = copyOf(path.endPose());
        List<Action> allDuring = new ArrayList<>(duringActions);
        allDuring.addAll(drainPendingDuringActions());
        PathActionSegment.Builder b = new PathActionSegment.Builder()
                .path(path)
                .preludeRunnables(drainPendingRunnables())
                .duringActions(allDuring)
                .moduleStates(drainPendingStates())
                .holdEnd(holdEnd)
                .enabled(enabled);
        if (pathTimeoutMs != null) b.pathTimeoutMs(pathTimeoutMs);
        if (holdAtDistance != null) b.holdAtDistance(holdAtDistance);
        return b.build();
    }

    private PathActionSegment buildActionSegment(List<Action> afterActions) {
        return new PathActionSegment.Builder()
                .preludeRunnables(drainPendingRunnables())
                .duringActions(drainPendingDuringActions())
                .afterActions(afterActions)
                .moduleStates(drainPendingStates())
                .enabled(enabled)
                .build();
    }

    private static Pose copyOf(Pose p) {
        return new Pose(p.getX(), p.getY(), p.getHeading());
    }

    private static boolean posesApproxEqual(Pose a, Pose b) {
        return Math.abs(a.getX() - b.getX()) < 0.1
                && Math.abs(a.getY() - b.getY()) < 0.1
                && Math.abs(a.getHeading() - b.getHeading()) < 0.001;
    }
}
