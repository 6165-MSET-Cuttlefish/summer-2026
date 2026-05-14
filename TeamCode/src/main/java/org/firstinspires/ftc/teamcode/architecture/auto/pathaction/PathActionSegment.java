package org.firstinspires.ftc.teamcode.architecture.auto.pathaction;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.core.Action;
import org.firstinspires.ftc.teamcode.core.State;

public class PathActionSegment {

    /**
     * Bundle returned by a deferred-path resolver so the scheduler can
     * launch during-actions produced by the body along with the path.
     */
    public static final class Resolved {
        public final PathChain path;
        public final List<Action> duringActions;
        public final boolean holdEnd;

        public Resolved(PathChain path, List<Action> duringActions, boolean holdEnd) {
            this.path = path;
            this.duringActions = duringActions;
            this.holdEnd = holdEnd;
        }
    }

    private final PathChain path;
    private final Supplier<Resolved> resolver;
    private final List<Callable<Boolean>> stopConditions;
    private final ConditionMode stopMode;
    private final List<Runnable> preludeRunnables;
    private final List<Action> duringActions;
    private final List<Action> afterActions;
    private final List<Callable<Boolean>> continueConditions;
    private final ConditionMode continueMode;
    private final Integer timeoutMs;
    private final Callable<Boolean> timeoutCondition;
    private final Integer pathTimeoutMs;
    private final List<State> moduleStates;
    private final boolean holdEnd;
    private final boolean enabled;
    private final Pose transitTarget;
    private final Double holdAtDistance;

    private PathActionSegment(Builder builder) {
        this.path = builder.path;
        this.resolver = builder.resolver;
        this.stopConditions = Collections.unmodifiableList(new ArrayList<>(builder.stopConditions));
        this.stopMode = builder.stopMode;
        this.preludeRunnables = Collections.unmodifiableList(new ArrayList<>(builder.preludeRunnables));
        this.duringActions = Collections.unmodifiableList(new ArrayList<>(builder.duringActions));
        this.afterActions = Collections.unmodifiableList(new ArrayList<>(builder.afterActions));
        this.continueConditions =
                Collections.unmodifiableList(new ArrayList<>(builder.continueConditions));
        this.continueMode = builder.continueMode;
        this.timeoutMs = builder.timeoutMs;
        this.timeoutCondition = builder.timeoutCondition;
        this.pathTimeoutMs = builder.pathTimeoutMs;
        this.moduleStates = Collections.unmodifiableList(new ArrayList<>(builder.moduleStates));
        this.holdEnd = builder.holdEnd;
        this.enabled = builder.enabled;
        this.transitTarget = builder.transitTarget;
        this.holdAtDistance = builder.holdAtDistance;
    }

    public Double getHoldAtDistance() {
        return holdAtDistance;
    }

    public PathChain getPath() {
        return path;
    }

    public List<Callable<Boolean>> getStopConditions() {
        return stopConditions;
    }

    public ConditionMode getStopMode() {
        return stopMode;
    }

    public List<Runnable> getPreludeRunnables() {
        return preludeRunnables;
    }

    public List<Action> getDuringActions() {
        return duringActions;
    }

    public List<Action> getAfterActions() {
        return afterActions;
    }

    public List<Callable<Boolean>> getContinueConditions() {
        return continueConditions;
    }

    public ConditionMode getContinueMode() {
        return continueMode;
    }

    public Integer getTimeoutMs() {
        return timeoutMs;
    }

    public Callable<Boolean> getTimeoutCondition() {
        return timeoutCondition;
    }

    public Integer getPathTimeoutMs() {
        return pathTimeoutMs;
    }

    public List<State> getModuleStates() {
        return moduleStates;
    }

    public boolean isHoldEnd() {
        return holdEnd;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isTransit() {
        return transitTarget != null;
    }

    public Pose getTransitTarget() {
        return transitTarget;
    }

    public boolean hasPath() {
        return path != null;
    }

    public Supplier<Resolved> getResolver() {
        return resolver;
    }

    public boolean hasResolver() {
        return resolver != null;
    }

    public boolean hasActions() {
        return !duringActions.isEmpty() || !afterActions.isEmpty();
    }

    public boolean isWait() {
        return !hasPath() && !hasActions() && !continueConditions.isEmpty();
    }

    public boolean isDelay() {
        return !hasPath() && !hasActions() && continueConditions.isEmpty() && timeoutMs != null
                && timeoutMs > -1;
    }

    public static PathActionSegment path(PathChain path) {
        return new Builder().path(path).build();
    }

    public static PathActionSegment action(Action... actions) {
        return new Builder().afterActions(Arrays.asList(actions)).build();
    }

    public static PathActionSegment delay(int delayMs) {
        return new Builder().timeoutMs(delayMs).build();
    }

    public static PathActionSegment waitFor(Callable<Boolean> condition, int timeoutMs) {
        return new Builder()
                .continueConditions(Collections.singletonList(condition))
                .continueMode(ConditionMode.OR)
                .timeoutMs(timeoutMs)
                .build();
    }

    public static class Builder {
        private PathChain path = null;
        private Supplier<Resolved> resolver = null;
        private List<Callable<Boolean>> stopConditions = new ArrayList<>();
        private ConditionMode stopMode = ConditionMode.OR;
        private List<Runnable> preludeRunnables = new ArrayList<>();
        private List<Action> duringActions = new ArrayList<>();
        private List<Action> afterActions = new ArrayList<>();
        private List<Callable<Boolean>> continueConditions = new ArrayList<>();
        private ConditionMode continueMode = ConditionMode.AND;
        private Integer timeoutMs = null;
        private Callable<Boolean> timeoutCondition = null;
        private Integer pathTimeoutMs = null;
        private List<State> moduleStates = new ArrayList<>();
        private boolean holdEnd = true;
        private boolean enabled = true;
        private Pose transitTarget = null;
        private Double holdAtDistance = null;

        public Builder path(PathChain path) {
            this.path = path;
            return this;
        }

        public Builder resolver(Supplier<Resolved> resolver) {
            this.resolver = resolver;
            return this;
        }

        public Builder stopConditions(List<Callable<Boolean>> c) {
            this.stopConditions = c;
            return this;
        }

        public Builder stopMode(ConditionMode mode) {
            this.stopMode = mode;
            return this;
        }

        public Builder preludeRunnables(List<Runnable> r) {
            this.preludeRunnables = r;
            return this;
        }

        public Builder duringActions(List<Action> a) {
            this.duringActions = a;
            return this;
        }

        public Builder afterActions(List<Action> a) {
            this.afterActions = a;
            return this;
        }

        public Builder continueConditions(List<Callable<Boolean>> c) {
            this.continueConditions = c;
            return this;
        }

        public Builder continueMode(ConditionMode mode) {
            this.continueMode = mode;
            return this;
        }

        public Builder timeoutMs(Integer ms) {
            this.timeoutMs = ms;
            return this;
        }

        public Builder timeoutCondition(Callable<Boolean> condition) {
            this.timeoutCondition = condition;
            return this;
        }

        public Builder pathTimeoutMs(Integer ms) {
            this.pathTimeoutMs = ms;
            return this;
        }

        public Builder moduleStates(List<State> states) {
            this.moduleStates = states;
            return this;
        }

        public Builder holdEnd(boolean holdEnd) {
            this.holdEnd = holdEnd;
            return this;
        }

        public Builder enabled(boolean enabled) {
            this.enabled = enabled;
            return this;
        }

        public Builder transitTarget(Pose transitTarget) {
            this.transitTarget = transitTarget;
            return this;
        }

        public Builder holdAtDistance(Double distance) {
            this.holdAtDistance = distance;
            return this;
        }

        public PathActionSegment build() {
            return new PathActionSegment(this);
        }
    }
}
