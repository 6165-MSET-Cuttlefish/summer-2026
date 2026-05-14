package org.firstinspires.ftc.teamcode.architecture.auto.scheduler;

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
import org.firstinspires.ftc.teamcode.core.action.Action;
import org.firstinspires.ftc.teamcode.core.Module;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.State;

/**
 * Builds an autonomous sequence of paths, actions, and waits. Queued setState/run/actionDuring
 * calls accumulate into the next real segment's prelude — zero extra scheduler ticks.
 */
public class PathActionBuilder {

    private final List<PathActionSegment> segments = new ArrayList<>();
    // Keyed by state class so two enums on the same module don't clobber each other.
    private final Map<Class<? extends State>, State> queuedStates = new LinkedHashMap<>();
    private final List<Runnable> queuedRunnables = new ArrayList<>();
    private final List<Action> queuedDuringActions = new ArrayList<>();

    private Pose nextSegmentStartPose = null;
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
        // null follower → late-bind to Robot.robot.follower at use time.
        this.follower = follower;
        this.clock = clock;
    }

    private Follower follower() {
        return follower != null ? follower : Robot.robot.follower;
    }

    public PathActionBuilder setStartPose(Pose startPose) {
        nextSegmentStartPose = startPose;
        return this;
    }

    /** Gate subsequent segments without restructuring the chain. */
    public PathActionBuilder setEnabled(boolean enabled) {
        if (!enabled) lastSegmentDisabled = true;
        this.enabled = enabled;
        return this;
    }

    public PathActionBuilder setState(State... states) {
        for (State state : states) {
            Module module = state.getModule();
            if (module == null) {
                throw new IllegalStateException(
                        "State " + state + " has no registered module. "
                        + "Register it via Module.setStates() before calling setState().");
            }
            queuedStates.put(state.getClass(), state);
        }
        return this;
    }

    public PathActionBuilder driveTo(Pose targetPose) {
        return driveTo(targetPose, 4000);
    }

    public PathActionBuilder driveTo(Pose targetPose, int pathTimeoutMs) {
        segments.add(new PathActionSegment.Builder()
                .transitTarget(targetPose)
                .pathTimeoutMs(pathTimeoutMs)
                .preludeRunnables(drainQueuedRunnables())
                .duringActions(drainQueuedDuringActions())
                .moduleStates(drainQueuedStates())
                .enabled(enabled)
                .build());
        nextSegmentStartPose = targetPose;
        return this;
    }

    public PathActionBuilder buildPath(Consumer<TrackingPathBuilder> body) {
        return buildPath(nextSegmentStartPose, body, null);
    }

    public PathActionBuilder buildPath(Consumer<TrackingPathBuilder> body, int pathTimeoutMs) {
        return buildPath(nextSegmentStartPose, body, pathTimeoutMs);
    }

    public PathActionBuilder buildPath(Pose startPose, Consumer<TrackingPathBuilder> body) {
        return buildPath(startPose, body, null);
    }

    public PathActionBuilder buildPath(Pose startPose, Consumer<TrackingPathBuilder> body, int pathTimeoutMs) {
        return buildPath(startPose, body, Integer.valueOf(pathTimeoutMs));
    }

    private PathActionBuilder buildPath(Pose startPose, Consumer<TrackingPathBuilder> body, Integer pathTimeoutMs) {
        Pose safeStart = Objects.requireNonNull(
                startPose, "startPose is required; call setStartPose() before buildPath");

        // After a disabled segment, transit back if our expected start drifted.
        if (enabled && lastSegmentDisabled
                && nextSegmentStartPose != null
                && !posesApproxEqual(safeStart, nextSegmentStartPose)) {
            driveTo(safeStart);
            lastSegmentDisabled = false;
        }

        TrackingPathBuilder pathBuilder = new TrackingPathBuilder(safeStart, follower().pathBuilder());
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
     * Path resolved at execution time so the body sees live pose / sensors. {@code declaredEndPose}
     * only seeds the next segment's default start; the resolved path uses live pose at execution.
     */
    public PathActionBuilder buildPathDeferred(Pose declaredEndPose, Consumer<TrackingPathBuilder> body) {
        if (enabled) nextSegmentStartPose = declaredEndPose;

        Supplier<PathActionSegment.Resolved> resolver = () -> {
            TrackingPathBuilder pb = new TrackingPathBuilder(follower().getPose(), follower().pathBuilder());
            body.accept(pb);
            return new PathActionSegment.Resolved(pb.buildPath(), pb.getDuringActions(), pb.isHoldEnd());
        };

        segments.add(new PathActionSegment.Builder()
                .resolver(resolver)
                .preludeRunnables(drainQueuedRunnables())
                .duringActions(drainQueuedDuringActions())
                .moduleStates(drainQueuedStates())
                .enabled(enabled)
                .build());
        return this;
    }

    /** Blocking action; scheduler waits for it to complete before advancing. */
    public PathActionBuilder action(Action action) {
        segments.add(buildActionSegment(Collections.singletonList(action)));
        return this;
    }

    /** Fire-and-forget action that emits with the next real segment's prelude. */
    public PathActionBuilder actionDuring(Action action) {
        queuedDuringActions.add(action);
        return this;
    }

    public PathActionBuilder run(Runnable code) {
        queuedRunnables.add(code);
        return this;
    }

    public PathActionBuilder await(Callable<Boolean> condition, int timeoutMs) {
        segments.add(new PathActionSegment.Builder()
                .continueConditions(Collections.singletonList(condition))
                .continueMode(ConditionMode.OR)
                .timeoutMs(timeoutMs)
                .preludeRunnables(drainQueuedRunnables())
                .duringActions(drainQueuedDuringActions())
                .moduleStates(drainQueuedStates())
                .enabled(enabled)
                .build());
        return this;
    }

    public PathActionBuilder await(Callable<Boolean> condition) {
        return await(condition, Integer.MAX_VALUE);
    }

    public PathActionBuilder delay(int delayMs) {
        segments.add(new PathActionSegment.Builder()
                .preludeRunnables(drainQueuedRunnables())
                .duringActions(drainQueuedDuringActions())
                .moduleStates(drainQueuedStates())
                .timeoutMs(delayMs)
                .enabled(enabled)
                .build());
        return this;
    }

    /** Cancel-and-handle on a condition. Replaces any prior override. */
    public PathActionBuilder setOverride(Callable<Boolean> condition, Runnable handler) {
        this.overrideCondition = condition;
        this.overrideHandler = handler;
        return this;
    }

    public PathActionBuilder setTimeOverride(int timeMs, Runnable handler) {
        return setOverride(() -> Robot.robot.opMode.getGameTimer().milliseconds() >= timeMs, handler);
    }

    @CheckResult
    public PathActionScheduler build() {
        // Flush trailing queue items as a final no-op segment so they actually run.
        if (!queuedRunnables.isEmpty() || !queuedDuringActions.isEmpty() || !queuedStates.isEmpty()) {
            segments.add(new PathActionSegment.Builder()
                    .preludeRunnables(drainQueuedRunnables())
                    .duringActions(drainQueuedDuringActions())
                    .moduleStates(drainQueuedStates())
                    .enabled(enabled)
                    .build());
        }
        PathActionScheduler scheduler = new PathActionScheduler(follower, clock);
        for (int i = 0; i < segments.size(); i++) scheduler.addSegment(segments.get(i));
        if (overrideCondition != null) scheduler.setOverride(overrideCondition, overrideHandler);
        return scheduler;
    }

    private List<State> drainQueuedStates() {
        List<State> out = new ArrayList<>(queuedStates.values());
        queuedStates.clear();
        return out;
    }

    private List<Runnable> drainQueuedRunnables() {
        List<Runnable> out = new ArrayList<>(queuedRunnables);
        queuedRunnables.clear();
        return out;
    }

    private List<Action> drainQueuedDuringActions() {
        List<Action> out = new ArrayList<>(queuedDuringActions);
        queuedDuringActions.clear();
        return out;
    }

    private PathActionSegment buildPathSegment(
            PathChain path, List<Action> duringActions, boolean holdEnd,
            Integer pathTimeoutMs, Double holdAtDistance) {
        if (enabled) nextSegmentStartPose = path.endPose();
        List<Action> allDuring = new ArrayList<>(duringActions);
        allDuring.addAll(drainQueuedDuringActions());
        PathActionSegment.Builder b = new PathActionSegment.Builder()
                .path(path)
                .preludeRunnables(drainQueuedRunnables())
                .duringActions(allDuring)
                .moduleStates(drainQueuedStates())
                .holdEnd(holdEnd)
                .enabled(enabled);
        if (pathTimeoutMs != null) b.pathTimeoutMs(pathTimeoutMs);
        if (holdAtDistance != null) b.holdAtDistance(holdAtDistance);
        return b.build();
    }

    private PathActionSegment buildActionSegment(List<Action> afterActions) {
        return new PathActionSegment.Builder()
                .preludeRunnables(drainQueuedRunnables())
                .duringActions(drainQueuedDuringActions())
                .afterActions(afterActions)
                .moduleStates(drainQueuedStates())
                .enabled(enabled)
                .build();
    }

    private static boolean posesApproxEqual(Pose a, Pose b) {
        return Math.abs(a.getX() - b.getX()) < 0.1
                && Math.abs(a.getY() - b.getY()) < 0.1
                && Math.abs(a.getHeading() - b.getHeading()) < 0.001;
    }
}
