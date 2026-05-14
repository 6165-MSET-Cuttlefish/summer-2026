package org.firstinspires.ftc.teamcode.core.action;

import java.util.List;
import java.util.Set;

import org.firstinspires.ftc.teamcode.core.Module;

/**
 * A composable action: a sequence of steps that the {@link Actions} scheduler ticks
 * cooperatively on the OpMode thread. No background threads; no hardware-write races.
 *
 * <p>Use {@link ActionBuilder} (or the factories on {@link Actions}) to construct one.
 */
public final class Action {

    final List<Step> steps;
    private final Set<Module> targets;
    private final String name;

    private int currentStepIndex = -1;
    private boolean stepStarted = false;
    private boolean done = false;
    private boolean cancelled = false;
    private boolean completedEarly = false;

    Action(List<Step> steps, Set<Module> targets, String name) {
        this.steps = steps;
        this.targets = targets;
        this.name = name;
    }

    /** Schedule on {@link Actions}. Conflicting actions (same target module) are cancelled. */
    public void run() {
        Actions.run(this);
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

    /**
     * Advance one scheduler tick. Returns true when the action is done or cancelled. Instant
     * steps chain within a single tick — yields only when a step returns false from {@link Step#tick}.
     */
    boolean tick() {
        if (done || cancelled) return true;

        while (true) {
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
                try {
                    step.start(this);
                } catch (Exception e) {
                    logStepFailure(currentStepIndex, e);
                    cancelled = true;
                    return true;
                }
                stepStarted = true;
            }
            boolean stepDone;
            try {
                stepDone = step.tick(this);
            } catch (Exception e) {
                logStepFailure(currentStepIndex, e);
                cancelled = true;
                return true;
            }
            if (!stepDone) return false;
            currentStepIndex++;
            stepStarted = false;
        }
    }

    private Step currentStepIfRunning() {
        if (currentStepIndex < 0 || currentStepIndex >= steps.size()) return null;
        return steps.get(currentStepIndex);
    }

    private void logStepFailure(int stepIndex, Exception e) {
        StringBuilder info = new StringBuilder();
        if (targets.isEmpty()) {
            info.append("no targets");
        } else {
            info.append("targets: [");
            int i = 0;
            for (Module m : targets) {
                if (i++ > 0) info.append(", ");
                info.append(m.getName());
            }
            info.append(']');
        }
        System.err.println("[Action] '" + name + "' failed at step "
                + (stepIndex + 1) + "/" + steps.size() + " (" + info + "): " + e.getMessage());
        e.printStackTrace();
    }

    public Action then(Action other) { return Actions.sequence(this, other); }
    public Action with(Action other) { return Actions.parallel(this, other); }
    public Action timeout(long ms) { return Actions.timeout(this, ms); }
    public Action named(String newName) { return new Action(steps, targets, newName); }

    @Override
    public String toString() {
        return name + "(" + steps.size() + " steps)";
    }

    /**
     * A single unit of work in an Action. Subclasses hold their own cross-tick state (timers,
     * indices, child actions). {@link #start} runs once when the step first activates;
     * {@link #tick} is polled until it returns true; {@link #cancel} runs when the parent
     * action is cancelled mid-step.
     */
    public abstract static class Step {
        protected void start(Action parent) {}
        protected abstract boolean tick(Action parent);
        protected void cancel(Action parent) {}
    }
}
