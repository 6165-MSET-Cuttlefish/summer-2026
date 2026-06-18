package org.firstinspires.ftc.teamcode.core.action;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import org.firstinspires.ftc.teamcode.core.Module;
import org.firstinspires.ftc.teamcode.core.State;

public final class ActionBuilder {
    private final List<Action.Step> steps = new ArrayList<>();
    private final Set<Module> targets = new LinkedHashSet<>();
    private String name = "Action";

    public ActionBuilder set(State... states) {
        for (State state : states) {
            Module module = state.getModule();
            if (module != null) {
                targets.add(module);
            } else {
                System.err.println("[ActionBuilder] Warning: State "
                        + state.getClass().getSimpleName()
                        + " has no module attached. Register it with a Module before using it.");
            }
            steps.add(new ActivateState(state));
        }
        return this;
    }

    /** Resolve states at execution time; targets are not pre-registered. */
    @SafeVarargs
    public final ActionBuilder setLazy(Supplier<? extends State>... suppliers) {
        steps.add(new ActivateStateLazy(Arrays.asList(suppliers)));
        return this;
    }

    public ActionBuilder run(Runnable code) {
        steps.add(new Run(code));
        return this;
    }

    public ActionBuilder delay(long ms) {
        steps.add(new Delay(ms));
        return this;
    }

    public ActionBuilder waitUntil(BooleanSupplier condition) {
        return waitUntil(condition, Long.MAX_VALUE);
    }

    public ActionBuilder waitUntil(BooleanSupplier condition, long timeoutMs) {
        steps.add(new WaitUntil(condition, timeoutMs));
        return this;
    }

    public ActionBuilder waitWhile(BooleanSupplier condition) {
        return waitUntil(() -> !condition.getAsBoolean());
    }

    public ActionBuilder waitWhile(BooleanSupplier condition, long timeoutMs) {
        return waitUntil(() -> !condition.getAsBoolean(), timeoutMs);
    }

    /** Inline another action's steps + targets, flattening nested sequences. */
    public ActionBuilder action(Action action) {
        action.markEmbedded();
        targets.addAll(action.getTargets());
        steps.addAll(action.steps);
        return this;
    }

    public ActionBuilder stopIf(BooleanSupplier condition) {
        steps.add(new StopIf(condition));
        return this;
    }

    public ActionBuilder ifThen(BooleanSupplier condition, Action ifTrue) {
        ifTrue.markEmbedded();
        targets.addAll(ifTrue.getTargets());
        steps.add(new Branch(condition, ifTrue, null));
        return this;
    }

    public ActionBuilder ifElse(BooleanSupplier condition, Action ifTrue, Action ifFalse) {
        ifTrue.markEmbedded();
        ifFalse.markEmbedded();
        targets.addAll(ifTrue.getTargets());
        targets.addAll(ifFalse.getTargets());
        steps.add(new Branch(condition, ifTrue, ifFalse));
        return this;
    }

    public ActionBuilder parallel(Action... actions) {
        for (Action a : actions) { a.markEmbedded(); targets.addAll(a.getTargets()); }
        steps.add(new Parallel(Arrays.asList(actions)));
        return this;
    }

    public ActionBuilder race(Action... actions) {
        for (Action a : actions) { a.markEmbedded(); targets.addAll(a.getTargets()); }
        steps.add(new Race(Arrays.asList(actions)));
        return this;
    }

    public ActionBuilder timeout(Action action, long ms) {
        action.markEmbedded();
        targets.addAll(action.getTargets());
        steps.add(new Timeout(action, ms));
        return this;
    }

    public ActionBuilder repeat(Action action, int times) {
        return repeat(action, () -> times);
    }

    public ActionBuilder repeat(Action action, IntSupplier times) {
        action.markEmbedded();
        targets.addAll(action.getTargets());
        steps.add(new Repeat(action, times));
        return this;
    }

    public ActionBuilder loop(Action action, BooleanSupplier condition) {
        action.markEmbedded();
        targets.addAll(action.getTargets());
        steps.add(new Loop(action, condition));
        return this;
    }

    public ActionBuilder retry(Action action, int maxAttempts, BooleanSupplier success) {
        return retry(action, maxAttempts, 0L, success);
    }

    public ActionBuilder retry(Action action, int maxAttempts, long delayMs, BooleanSupplier success) {
        if (maxAttempts < 1) throw new IllegalArgumentException("maxAttempts must be at least 1");
        if (delayMs < 0) throw new IllegalArgumentException("delayMs must be non-negative");
        action.markEmbedded();
        targets.addAll(action.getTargets());
        steps.add(new Retry(action, maxAttempts, delayMs, success));
        return this;
    }

    public ActionBuilder retry(Runnable code, int maxAttempts, BooleanSupplier success) {
        return retry(code, maxAttempts, 0L, success);
    }

    public ActionBuilder retry(Runnable code, int maxAttempts, long delayMs, BooleanSupplier success) {
        if (maxAttempts < 1) throw new IllegalArgumentException("maxAttempts must be at least 1");
        if (delayMs < 0) throw new IllegalArgumentException("delayMs must be non-negative");
        Action inner = new ActionBuilder().run(code).build();
        steps.add(new Retry(inner, maxAttempts, delayMs, success));
        return this;
    }

    public ActionBuilder withName(String name) {
        this.name = name;
        return this;
    }

    public ActionBuilder targets(Module module) {
        targets.add(module);
        return this;
    }

    public Action build() {
        return new Action(new ArrayList<>(steps), new LinkedHashSet<>(targets), name);
    }

    private static final class ActivateState extends Action.Step {
        private final State state;
        ActivateState(State state) { this.state = state; }
        @Override protected void start(Action parent) { state.activate(); }
        @Override protected boolean tick(Action parent) { return true; }
    }

    private static final class ActivateStateLazy extends Action.Step {
        private final List<Supplier<? extends State>> suppliers;
        ActivateStateLazy(List<Supplier<? extends State>> suppliers) { this.suppliers = suppliers; }
        @Override
        protected void start(Action parent) {
            for (Supplier<? extends State> s : suppliers) s.get().activate();
        }
        @Override protected boolean tick(Action parent) { return true; }
    }

    private static final class Run extends Action.Step {
        private final Runnable code;
        Run(Runnable code) { this.code = code; }
        @Override protected void start(Action parent) { code.run(); }
        @Override protected boolean tick(Action parent) { return true; }
    }

    private static final class Delay extends Action.Step {
        private final long ms;
        private long startMs;
        Delay(long ms) { this.ms = ms; }
        @Override protected void start(Action parent) { startMs = System.currentTimeMillis(); }
        @Override
        protected boolean tick(Action parent) {
            return System.currentTimeMillis() - startMs >= ms;
        }
    }

    private static final class WaitUntil extends Action.Step {
        private final BooleanSupplier condition;
        private final long timeoutMs;
        private long startMs;
        WaitUntil(BooleanSupplier condition, long timeoutMs) {
            this.condition = condition;
            this.timeoutMs = timeoutMs;
        }
        @Override protected void start(Action parent) { startMs = System.currentTimeMillis(); }
        @Override
        protected boolean tick(Action parent) {
            if (condition.getAsBoolean()) return true;
            if (timeoutMs == Long.MAX_VALUE) return false;
            return System.currentTimeMillis() - startMs >= timeoutMs;
        }
    }

    private static final class StopIf extends Action.Step {
        private final BooleanSupplier condition;
        StopIf(BooleanSupplier condition) { this.condition = condition; }
        @Override
        protected boolean tick(Action parent) {
            if (condition.getAsBoolean()) parent.completeEarly();
            return true;
        }
    }

    private static final class Branch extends Action.Step {
        private final BooleanSupplier condition;
        private final Action ifTrue;
        private final Action ifFalse;
        private Action chosen;

        Branch(BooleanSupplier condition, Action ifTrue, Action ifFalse) {
            this.condition = condition;
            this.ifTrue = ifTrue;
            this.ifFalse = ifFalse;
        }

        @Override
        protected void start(Action parent) {
            chosen = condition.getAsBoolean() ? ifTrue : ifFalse;
            if (chosen != null) chosen.reset();
        }

        @Override
        protected boolean tick(Action parent) {
            return chosen == null || chosen.tick();
        }

        @Override
        protected void cancel(Action parent) {
            if (chosen != null) chosen.cancel();
        }
    }

    private static final class Parallel extends Action.Step {
        private final List<Action> actions;
        Parallel(List<Action> actions) { this.actions = actions; }

        @Override
        protected void start(Action parent) {
            for (Action a : actions) a.reset();
        }

        @Override
        protected boolean tick(Action parent) {
            boolean allDone = true;
            for (int i = 0; i < actions.size(); i++) {
                if (!actions.get(i).tick()) allDone = false;
            }
            return allDone;
        }

        @Override
        protected void cancel(Action parent) {
            for (Action a : actions) a.cancel();
        }
    }

    private static final class Race extends Action.Step {
        private final List<Action> actions;
        Race(List<Action> actions) { this.actions = actions; }

        @Override
        protected void start(Action parent) {
            for (Action a : actions) a.reset();
        }

        @Override
        protected boolean tick(Action parent) {
            // Tick in order and stop at the first child that terminates, so losers that haven't
            // reached their instant step this tick don't fire it. Only a genuinely completed child
            // (not a cancelled/errored one) is the winner.
            Action winner = null;
            boolean decided = false;
            for (int i = 0; i < actions.size(); i++) {
                Action a = actions.get(i);
                if (a.tick()) {
                    decided = true;
                    if (a.isComplete()) winner = a;
                    break;
                }
            }
            if (decided) {
                for (int i = 0; i < actions.size(); i++) {
                    Action a = actions.get(i);
                    if (a != winner && !a.isComplete()) a.cancel();
                }
                return true;
            }
            return false;
        }

        @Override
        protected void cancel(Action parent) {
            for (Action a : actions) a.cancel();
        }
    }

    private static final class Timeout extends Action.Step {
        private final Action action;
        private final long ms;
        private long startMs;

        Timeout(Action action, long ms) {
            this.action = action;
            this.ms = ms;
        }

        @Override
        protected void start(Action parent) {
            action.reset();
            startMs = System.currentTimeMillis();
        }

        @Override
        protected boolean tick(Action parent) {
            if (action.tick()) return true;
            if (System.currentTimeMillis() - startMs >= ms) {
                action.cancel();
                return true;
            }
            return false;
        }

        @Override
        protected void cancel(Action parent) {
            action.cancel();
        }
    }

    private static final class Repeat extends Action.Step {
        private final Action action;
        private final IntSupplier times;
        private int total;
        private int done;

        Repeat(Action action, IntSupplier times) {
            this.action = action;
            this.times = times;
        }

        @Override
        protected void start(Action parent) {
            total = times.getAsInt();
            done = 0;
            if (done < total) action.reset();
        }

        @Override
        protected boolean tick(Action parent) {
            if (done >= total) return true;
            if (action.tick()) {
                if (action.isCancelled()) { parent.cancel(); return true; }
                done++;
                if (done < total) action.reset();
            }
            return done >= total;
        }

        @Override
        protected void cancel(Action parent) {
            action.cancel();
        }
    }

    private static final class Loop extends Action.Step {
        private final Action action;
        private final BooleanSupplier condition;
        private boolean active;

        Loop(Action action, BooleanSupplier condition) {
            this.action = action;
            this.condition = condition;
        }

        @Override
        protected void start(Action parent) {
            active = condition.getAsBoolean();
            if (active) action.reset();
        }

        @Override
        protected boolean tick(Action parent) {
            if (!active) return true;
            if (action.tick()) {
                if (action.isCancelled()) { parent.cancel(); return true; }
                if (condition.getAsBoolean()) {
                    action.reset();
                } else {
                    active = false;
                    return true;
                }
            }
            return false;
        }

        @Override
        protected void cancel(Action parent) {
            action.cancel();
        }
    }

    private static final class Retry extends Action.Step {
        private final Action action;
        private final int maxAttempts;
        private final long delayMs;
        private final BooleanSupplier success;
        private int attempts;
        private long resumeAtMs;

        Retry(Action action, int maxAttempts, long delayMs, BooleanSupplier success) {
            this.action = action;
            this.maxAttempts = maxAttempts;
            this.delayMs = delayMs;
            this.success = success;
        }

        @Override
        protected void start(Action parent) {
            attempts = 0;
            resumeAtMs = 0L;
            action.reset();
        }

        @Override
        protected boolean tick(Action parent) {
            if (resumeAtMs > 0L) {
                if (System.currentTimeMillis() < resumeAtMs) return false;
                resumeAtMs = 0L;
                action.reset();
            }
            if (action.tick()) {
                if (action.isCancelled()) { parent.cancel(); return true; }
                attempts++;
                if (success.getAsBoolean() || attempts >= maxAttempts) return true;
                if (delayMs > 0) {
                    resumeAtMs = System.currentTimeMillis() + delayMs;
                } else {
                    action.reset();
                }
            }
            return false;
        }

        @Override
        protected void cancel(Action parent) {
            action.cancel();
        }
    }
}
