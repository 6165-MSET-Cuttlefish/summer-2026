package org.firstinspires.ftc.teamcode.architecture.auto.scheduler;

import android.os.SystemClock;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.core.action.Action;
import org.firstinspires.ftc.teamcode.core.action.Actions;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.State;

/**
 * Drives a list of {@link PathActionSegment}s through a small state machine. Each {@link #update()}
 * advances by exactly one transition, so the scheduler never blocks the OpMode loop.
 */
public class PathActionScheduler {
    private static final String TAG = "PathActionScheduler";

    private final Follower follower;
    private final LongSupplier clock;

    private final List<PathActionSegment> segments = new ArrayList<>();
    private int currentIndex = 0;
    private SchedulerState currentState = SchedulerState.IDLE;
    private long segmentStartTimeMs = 0L;

    private List<Action> pendingActions = new ArrayList<>();
    private List<Action> activeDuringActions = new ArrayList<>();

    private final List<Callable<Boolean>> overrideConditions = new ArrayList<>();
    private Runnable overrideCallback = null;
    private boolean overrideTriggered = false;

    public PathActionScheduler() {
        this(null, null);
    }

    public PathActionScheduler(Follower follower, LongSupplier clock) {
        // null follower → late-bind to Robot.robot.follower at use time.
        this.follower = follower;
        this.clock = clock != null ? clock : SystemClock::elapsedRealtime;
    }

    private Follower follower() {
        return follower != null ? follower : Robot.robot.follower;
    }

    private long now() {
        return clock.getAsLong();
    }

    public PathActionScheduler addSegment(PathActionSegment segment) {
        segments.add(segment);
        return this;
    }

    public PathActionSegment getCurrentSegment() {
        return currentIndex < segments.size() ? segments.get(currentIndex) : null;
    }

    public PathChain getCurrentPath() {
        PathActionSegment seg = getCurrentSegment();
        return seg == null ? null : seg.getPath();
    }

    public List<PathChain> getAllPaths() {
        List<PathChain> paths = new ArrayList<>();
        for (int i = 0; i < segments.size(); i++) {
            PathActionSegment s = segments.get(i);
            if (s.hasPath()) paths.add(s.getPath());
        }
        return paths;
    }

    public SchedulerState getCurrentState() { return currentState; }
    public int getCurrentIndex() { return currentIndex; }
    public int getTotalSegments() { return segments.size(); }
    public boolean isComplete() { return currentIndex >= segments.size(); }

    public void reset() {
        currentIndex = 0;
        currentState = SchedulerState.IDLE;
        segmentStartTimeMs = 0L;
        // Re-arm so a fired override doesn't permanently disable the scheduler; keep the config.
        overrideTriggered = false;
        cancelAsyncOperations();
    }

    public void cancelAll() {
        cancelAsyncOperations();
        stopCurrentPath();
        Actions.cancelAll();
    }

    public void shutdown() {
        cancelAsyncOperations();
    }

    public void setOverride(Callable<Boolean> condition, Runnable callback) {
        overrideConditions.clear();
        overrideConditions.add(condition);
        this.overrideCallback = callback;
        this.overrideTriggered = false;
    }

    public boolean isOverrideTriggered() { return overrideTriggered; }

    public void triggerOverride() {
        if (overrideTriggered) return;
        overrideTriggered = true;
        cancelAll();
        if (overrideCallback != null) overrideCallback.run();
    }

    /** Advance the scheduler by one transition. Call once per OpMode loop. */
    public void update() {
        if (!overrideTriggered && shouldTriggerOverride()) {
            triggerOverride();
            return;
        }
        if (overrideTriggered || isComplete()) return;

        PathActionSegment segment = getCurrentSegment();
        if (segment == null) return;

        // Honor after-actions on timeout so "drive then shoot" still shoots if the drive runs long.
        if (currentState != SchedulerState.IDLE && hasTimedOut(segment)) {
            if (currentState == SchedulerState.PATH_RUNNING) stopCurrentPath();
            if (tryStartAfterActions(segment)) return;
            advanceToNextSegment();
            return;
        }

        switch (currentState) {
            case IDLE:
                if (segment.isEnabled()) initializeSegment(segment);
                else advanceToNextSegment();
                break;
            case PATH_RUNNING:
                tickPathRunning(segment);
                break;
            case AFTER_ACTION:
                if (areActionsComplete()) beginWaiting(segment);
                break;
            case WAITING:
                if (canContinue(segment)) advanceToNextSegment();
                break;
            case COMPLETED:
                advanceToNextSegment();
                break;
        }
    }

    private void tickPathRunning(PathActionSegment segment) {
        if (hasPathTimedOut(segment) || shouldStopPath(segment)) {
            stopCurrentPath();
            executeAfterActions(segment);
            return;
        }
        if (shouldHoldAtDistance(segment)) {
            // Advance without touching the follower — Pedro finishes + engages end-hold on its
            // own, so subsequent segments overlap the final convergence and hand off cleanly.
            executeAfterActions(segment);
            return;
        }
        if (!follower().isBusy()) executeAfterActions(segment);
    }

    public String getDebugInfo() {
        PathActionSegment seg = getCurrentSegment();
        StringBuilder sb = new StringBuilder()
                .append(currentIndex + 1).append('/').append(segments.size())
                .append(" | ").append(currentState).append(" | ");
        if (seg == null) return sb.append("0/0ms | P:false D:0 A:0").toString();

        long elapsed = segmentStartTimeMs == 0 ? 0 : now() - segmentStartTimeMs;
        Integer timeout = seg.getTimeoutMs();
        Integer pathTimeout = seg.getPathTimeoutMs();
        sb.append(elapsed).append('/').append(timeout == null ? "inf" : timeout).append("ms");
        if (pathTimeout != null) sb.append(" PT:").append(pathTimeout).append("ms");
        sb.append(" | P:").append(seg.hasPath() || seg.isTransit());
        if (seg.isTransit()) sb.append("(transit)");
        if (!seg.isEnabled()) sb.append("(skip)");
        sb.append(" D:").append(seg.getDuringActions().size())
          .append(" A:").append(seg.getAfterActions().size());
        return sb.toString();
    }

    private void initializeSegment(PathActionSegment segment) {
        segmentStartTimeMs = now();
        cancelAsyncOperations();

        List<State> states = segment.getModuleStates();
        for (int i = 0; i < states.size(); i++) states.get(i).activate();
        List<Runnable> prelude = segment.getPreludeRunnables();
        for (int i = 0; i < prelude.size(); i++) prelude.get(i).run();

        if (segment.isTransit()) beginTransitPath(segment);
        else beginPathFollowing(segment);
    }

    private void beginTransitPath(PathActionSegment segment) {
        Pose current = follower().getPose();
        Pose target = segment.getTransitTarget();
        PathChain transit = follower().pathBuilder()
                .addPath(new BezierLine(current, target))
                .setLinearHeadingInterpolation(current.getHeading(), target.getHeading())
                .build();
        currentState = SchedulerState.PATH_RUNNING;
        follower().followPath(transit, segment.isHoldEnd());
    }

    private void beginPathFollowing(PathActionSegment segment) {
        if (segment.hasPath() || segment.hasResolver()) {
            PathChain chain;
            List<Action> during;
            boolean holdEnd;
            if (segment.hasResolver()) {
                PathActionSegment.Resolved r = segment.getResolver().get();
                chain = r.path; during = r.duringActions; holdEnd = r.holdEnd;
            } else {
                chain = segment.getPath();
                during = segment.getDuringActions();
                holdEnd = segment.isHoldEnd();
            }
            currentState = SchedulerState.PATH_RUNNING;
            follower().followPath(chain, holdEnd);
            for (int i = 0; i < during.size(); i++) during.get(i).schedule();
            activeDuringActions = new ArrayList<>(during);
        } else if (!segment.getDuringActions().isEmpty()) {
            // No path — don't track these so advancing past the segment doesn't cancel them.
            List<Action> during = segment.getDuringActions();
            for (int i = 0; i < during.size(); i++) during.get(i).schedule();
            executeAfterActions(segment);
        } else {
            executeAfterActions(segment);
        }
    }

    private void executeAfterActions(PathActionSegment segment) {
        List<Action> after = segment.getAfterActions();
        if (after.isEmpty()) {
            beginWaiting(segment);
            return;
        }
        currentState = SchedulerState.AFTER_ACTION;
        for (int i = 0; i < after.size(); i++) after.get(i).schedule();
        pendingActions = new ArrayList<>(after);
    }

    private void beginWaiting(PathActionSegment segment) {
        boolean hasTimeout = segment.getTimeoutMs() != null && segment.getTimeoutMs() > -1;
        currentState = (!segment.getContinueConditions().isEmpty() || hasTimeout)
                ? SchedulerState.WAITING : SchedulerState.COMPLETED;
    }

    private void advanceToNextSegment() {
        cancelAsyncOperations();
        currentIndex++;
        currentState = SchedulerState.IDLE;
        segmentStartTimeMs = 0L;
    }

    private void cancelAsyncOperations() {
        for (int i = 0; i < pendingActions.size(); i++) pendingActions.get(i).cancel();
        pendingActions.clear();
        for (int i = 0; i < activeDuringActions.size(); i++) activeDuringActions.get(i).cancel();
        activeDuringActions.clear();
    }

    private void stopCurrentPath() {
        PathActionSegment segment = getCurrentSegment();
        if (segment == null) return;

        if (segment.isTransit()) {
            follower().breakFollowing();
            return;
        }
        if (!segment.hasPath()) return;
        // Re-issuing followPath(path, true) is how we ask Pedro to engage its end-hold.
        if (segment.isHoldEnd()) follower().followPath(segment.getPath(), true);
        else follower().breakFollowing();
    }

    /** Skip the current segment but still fire its after-actions (cleanup still runs). */
    public void skipCurrentSegment() {
        PathActionSegment segment = getCurrentSegment();
        if (segment == null) return;
        if (currentState == SchedulerState.PATH_RUNNING) stopCurrentPath();
        if (tryStartAfterActions(segment)) return;
        advanceToNextSegment();
    }

    private boolean tryStartAfterActions(PathActionSegment segment) {
        if (currentState == SchedulerState.AFTER_ACTION
                || currentState == SchedulerState.WAITING
                || currentState == SchedulerState.COMPLETED) {
            return false;
        }
        if (segment.getAfterActions().isEmpty()) return false;
        executeAfterActions(segment);
        return true;
    }

    private boolean areActionsComplete() {
        for (int i = 0; i < pendingActions.size(); i++) {
            if (pendingActions.get(i).isRunning()) return false;
        }
        return true;
    }

    private boolean hasTimedOut(PathActionSegment segment) {
        Integer timeout = segment.getTimeoutMs();
        if (timeout == null) return false;
        Callable<Boolean> cond = segment.getTimeoutCondition();
        if (cond != null && !evaluateCondition(cond)) return false;
        return now() - segmentStartTimeMs > timeout;
    }

    private boolean hasPathTimedOut(PathActionSegment segment) {
        Integer pathTimeout = segment.getPathTimeoutMs();
        if (pathTimeout == null) return false;
        return now() - segmentStartTimeMs > pathTimeout;
    }

    private boolean canContinue(PathActionSegment segment) {
        List<Callable<Boolean>> conditions = segment.getContinueConditions();
        if (conditions.isEmpty()) {
            boolean hasTimeout = segment.getTimeoutMs() != null && segment.getTimeoutMs() > -1;
            return !hasTimeout;
        }
        return combine(conditions, segment.getContinueMode());
    }

    private boolean shouldHoldAtDistance(PathActionSegment segment) {
        Double distance = segment.getHoldAtDistance();
        return distance != null && follower().getDistanceRemaining() <= distance;
    }

    private boolean shouldStopPath(PathActionSegment segment) {
        List<Callable<Boolean>> conditions = segment.getStopConditions();
        if (conditions.isEmpty()) return false;
        return combine(conditions, segment.getStopMode());
    }

    private boolean combine(List<Callable<Boolean>> conditions, ConditionMode mode) {
        if (mode == ConditionMode.AND) {
            for (int i = 0; i < conditions.size(); i++) {
                if (!evaluateCondition(conditions.get(i))) return false;
            }
            return true;
        }
        for (int i = 0; i < conditions.size(); i++) {
            if (evaluateCondition(conditions.get(i))) return true;
        }
        return false;
    }

    private boolean evaluateCondition(Callable<Boolean> condition) {
        try {
            return condition.call();
        } catch (Exception e) {
            RobotLog.ee(TAG, "condition threw " + e.getClass().getSimpleName() + ": " + e.getMessage(), e);
            return false;
        }
    }

    private boolean shouldTriggerOverride() {
        for (int i = 0; i < overrideConditions.size(); i++) {
            if (evaluateCondition(overrideConditions.get(i))) return true;
        }
        return false;
    }
}
