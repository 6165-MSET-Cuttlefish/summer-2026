package org.firstinspires.ftc.teamcode.architecture.action;

import java.util.List;
import java.util.Set;

import org.firstinspires.ftc.teamcode.architecture.core.Module;

/** Composable action ticked cooperatively by the {@link Actions} scheduler; build via {@link ActionBuilder}. */
public final class Action {

    final List<Step> steps;
    private final Set<Module> targets;
    private final String name;

    private int currentStepIndex = -1;
    private boolean stepStarted = false;
    private boolean done = false;
    private boolean cancelled = false;
    private boolean completedEarly = false;
    private boolean embedded = false;

    Action(List<Step> steps, Set<Module> targets, String name) {
        this.steps = steps;
        this.targets = targets;
        this.name = name;
    }

    /** Schedule; conflicting actions (shared target module) are cancelled. Returns immediately. */
    public void schedule() {
        Actions.schedule(this);
    }

    public void cancel() {
        if (done || cancelled) return;
        cancelled = true;
        Step current = currentStepIfRunning();
        if (current != null) current.cancel(this);
    }

    public boolean isRunning() {
        return !done && !cancelled && currentStepIndex != -1 && Actions.contains(this);
    }

    public boolean isComplete() {
        return done && !cancelled;
    }

    public boolean isCancelled() {
        return cancelled;
    }

    /** Composing one Action instance twice shares its mutable Step state (Delay start, Retry attempts), so an embedded action is refused — build a fresh one per use. */
    void markEmbedded() {
        if (embedded) {
            throw new IllegalStateException(
                    "Action '" + name + "' is being composed into a second action. A composed action "
                    + "shares mutable step state with its host; reusing one instance corrupts both. "
                    + "Build a fresh action each time (e.g. return a new Actions.builder()...build()).");
        }
        embedded = true;
    }
    boolean isEmbedded() { return embedded; }

    public float getProgress() {
        if (steps.isEmpty() || done) return 1f;
        if (currentStepIndex < 0) return 0f;
        return (float) currentStepIndex / steps.size();
    }

    public Set<Module> getTargets() {
        return targets;
    }

    public String getName() {
        return name;
    }

    /** Stop after the current step finishes without flagging the action as cancelled. */
    public void completeEarly() {
        completedEarly = true;
    }

    void reset() {
        currentStepIndex = -1;
        stepStarted = false;
        done = false;
        cancelled = false;
        completedEarly = false;
    }

    boolean tick() {
        if (done || cancelled) return true;

        while (true) {
            // A step's start/tick may have indirectly cancelled us (e.g. via scheduled conflict).
            if (done || cancelled) return true;
            if (completedEarly) {
                done = true;
                return true;
            }
            if (currentStepIndex == -1) {
                currentStepIndex = 0;
                stepStarted = false;
            }
            if (currentStepIndex >= steps.size()) {
                done = true;
                return true;
            }
            Step step = steps.get(currentStepIndex);
            if (!stepStarted) {
                // No try/catch by design: a throwing step must surface as a crash, not be swallowed.
                step.start(this);
                stepStarted = true;
            }
            if (!step.tick(this)) return false;
            currentStepIndex++;
            stepStarted = false;
        }
    }

    private Step currentStepIfRunning() {
        if (currentStepIndex < 0 || currentStepIndex >= steps.size()) return null;
        return steps.get(currentStepIndex);
    }

    public Action then(Action other) { return Actions.sequence(this, other); }
    public Action with(Action other) { return Actions.parallel(this, other); }
    public Action timeout(long ms) { return Actions.timeout(this, ms); }

    /** Relabelled view sharing this action's mutable steps; marks the source embedded so only the copy is schedulable. */
    public Action withName(String newName) {
        markEmbedded();
        return new Action(steps, targets, newName);
    }

    @Override
    public String toString() {
        return name + "(" + steps.size() + " steps)";
    }

    /** One unit of work: {@link #start} runs once on activation, {@link #tick} is polled until it returns true. */
    public abstract static class Step {
        protected void start(Action parent) {}
        protected abstract boolean tick(Action parent);
        protected void cancel(Action parent) {}
    }
}
