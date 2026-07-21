package org.firstinspires.ftc.teamcode.architecture.auto.scheduler;

import android.os.SystemClock;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.Callable;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.architecture.action.Action;
import org.firstinspires.ftc.teamcode.architecture.action.Actions;
import org.firstinspires.ftc.teamcode.architecture.core.State;

/** Each {@link #update()} advances by exactly one transition, so it never blocks the OpMode loop. */
public class PathActionScheduler {
    private final Follower follower;
    private final LongSupplier clock;

    private final List<PathActionSegment> segments = new ArrayList<>();
    private int currentIndex = 0;
    private SchedulerState currentState = SchedulerState.IDLE;
    private long segmentStartTimeMs = 0L;

    private List<Action> pendingActions = new ArrayList<>();
    private List<Action> activeDuringActions = new ArrayList<>();

    // Captured at follow time: a resolver segment has no stable PathChain for stopCurrentPath() to re-issue.
    private PathChain activeResolvedChain = null;
    private boolean activeResolvedHoldEnd = false;
    // Armed once remaining exceeds holdAtDistance, so a short path can't skip the drive on tick one.
    private boolean leftHoldStart = false;
    private Double activeResolvedHoldAtDistance = null;

    private final List<Callable<Boolean>> overrideConditions = new ArrayList<>();
    private Runnable overrideCallback = null;
    private boolean overrideTriggered = false;

    public PathActionScheduler(Follower follower) {
        this(follower, null);
    }

    public PathActionScheduler(Follower follower, LongSupplier clock) {
        this.follower = Objects.requireNonNull(follower, "follower");
        this.clock = clock != null ? clock : SystemClock::elapsedRealtime;
    }

    private Follower follower() {
        return follower;
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
        // Re-arm so a fired override doesn't permanently disable the scheduler; the config is kept.
        overrideTriggered = false;
        cancelAsyncOperations();
    }

    public void cancelAll() {
        cancelAsyncOperations();
        // abort=true: a hard cancel must stop in place — re-engaging Pedro's end-hold drives to the chain end.
        stopCurrentPath(true);
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
        // Mark finished so getCurrentState()/skipCurrentSegment() don't report a stale state post-abort.
        currentState = SchedulerState.COMPLETED;
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
            // Advancing now would cancel in-flight after-actions mid-run, breaking that contract.
            if (currentState == SchedulerState.AFTER_ACTION) {
                if (areActionsComplete()) beginWaiting(segment);
                return;
            }
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
            // Don't touch the follower: Pedro finishes + end-holds itself, so the next segment overlaps.
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
        leftHoldStart = false;
        activeResolvedChain = null;
        activeResolvedHoldEnd = false;
        activeResolvedHoldAtDistance = null;
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
                chain = r.path;
                // Merge queued during-actions so actionDuring(...) before buildPathDeferred(...) still runs.
                during = new ArrayList<>(segment.getDuringActions());
                during.addAll(r.duringActions);
                holdEnd = r.holdEnd;
                activeResolvedChain = chain;
                activeResolvedHoldEnd = holdEnd;
                activeResolvedHoldAtDistance = r.holdAtDistance;
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
        leftHoldStart = false;
        activeResolvedChain = null;
        activeResolvedHoldEnd = false;
        activeResolvedHoldAtDistance = null;
    }

    private void cancelAsyncOperations() {
        for (int i = 0; i < pendingActions.size(); i++) pendingActions.get(i).cancel();
        pendingActions.clear();
        for (int i = 0; i < activeDuringActions.size(); i++) activeDuringActions.get(i).cancel();
        activeDuringActions.clear();
    }

    /** Skip/timeout stop: re-engage Pedro's end-hold so the next segment starts from a known pose. */
    private void stopCurrentPath() {
        stopCurrentPath(false);
    }

    private void stopCurrentPath(boolean abort) {
        PathActionSegment segment = getCurrentSegment();
        if (segment == null) return;

        if (segment.isTransit()) {
            follower().breakFollowing();
            return;
        }

        PathChain chain;
        boolean holdEnd;
        if (segment.hasResolver()) {
            chain = activeResolvedChain;
            holdEnd = activeResolvedHoldEnd;
        } else if (segment.hasPath()) {
            chain = segment.getPath();
            holdEnd = segment.isHoldEnd();
        } else {
            return;
        }

        // Re-issuing followPath(chain, true) is how Pedro engages end-hold; without it the robot coasts.
        // On abort always break: re-engaging end-hold would drive to the chain end.
        if (!abort && holdEnd && chain != null) follower().followPath(chain, true);
        else follower().breakFollowing();
    }

    /** Skip the current segment; its after-actions still fire. */
    public void skipCurrentSegment() {
        if (overrideTriggered) return;
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
        Double distance = segment.hasResolver() ? activeResolvedHoldAtDistance : segment.getHoldAtDistance();
        if (distance == null) return false;
        // Whole-chain remaining, not current-leg: a multi-leg chain must not advance on the first leg alone.
        double remaining = follower().getTotalDistanceRemaining();
        // Pedro reports -1 for a setNoDeceleration() chain; fall back to current-leg remaining.
        if (remaining < 0) remaining = follower().getDistanceRemaining();
        if (remaining > distance) {
            leftHoldStart = true;
            return false;
        }
        return leftHoldStart;
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
            throw new RuntimeException(e);
        }
    }

    private boolean shouldTriggerOverride() {
        for (int i = 0; i < overrideConditions.size(); i++) {
            if (evaluateCondition(overrideConditions.get(i))) return true;
        }
        return false;
    }
}
