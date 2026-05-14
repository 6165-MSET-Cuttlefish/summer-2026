package org.firstinspires.ftc.teamcode.core.action;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.firstinspires.ftc.teamcode.core.Module;
import org.firstinspires.ftc.teamcode.core.state.State;

/**
 * Single-threaded cooperative scheduler. Holds the active action list and pumps every action
 * once when {@link #update} is called. EnhancedOpMode calls update() each loop between
 * primaryLoop() and writeModules(). All step code runs on the OpMode thread.
 */
public final class Actions {
    private Actions() {}

    private static final ArrayList<Action> active = new ArrayList<>();

    public static Action set(State... states) {
        ActionBuilder b = new ActionBuilder();
        for (State s : states) b.set(s);
        return b.build();
    }

    @SafeVarargs
    public static Action setLazy(Supplier<? extends State>... suppliers) {
        return new ActionBuilder().setLazy(suppliers).build();
    }

    public static Action run(Runnable code) {
        return new ActionBuilder().run(code).build();
    }

    public static Action delay(long ms) {
        return new ActionBuilder().delay(ms).build();
    }

    public static Action noop() {
        return new ActionBuilder().build();
    }

    public static Action waitUntil(BooleanSupplier condition) {
        return new ActionBuilder().waitUntil(condition).build();
    }

    public static Action waitUntil(BooleanSupplier condition, long timeoutMs) {
        return new ActionBuilder().waitUntil(condition, timeoutMs).build();
    }

    public static Action sequence(Action... actions) {
        ActionBuilder b = new ActionBuilder();
        for (Action a : actions) b.action(a);
        return b.named("Sequence").build();
    }

    public static Action parallel(Action... actions) {
        return new ActionBuilder().parallel(actions).named("Parallel").build();
    }

    public static Action race(Action... actions) {
        return new ActionBuilder().race(actions).named("Race").build();
    }

    public static Action timeout(Action action, long ms) {
        return new ActionBuilder().timeout(action, ms).named("Timeout").build();
    }

    public static Action repeat(Action action, int times) {
        return new ActionBuilder().repeat(action, times).named("Repeat").build();
    }

    public static Action loop(Action action, BooleanSupplier condition) {
        return new ActionBuilder().loop(action, condition).named("Loop").build();
    }

    public static Action ifThen(BooleanSupplier condition, Action ifTrue) {
        return new ActionBuilder().ifThen(condition, ifTrue).named("IfThen").build();
    }

    public static Action ifElse(BooleanSupplier condition, Action ifTrue, Action ifFalse) {
        return new ActionBuilder().ifElse(condition, ifTrue, ifFalse).named("IfElse").build();
    }

    public static Action retry(Action action, int maxAttempts, BooleanSupplier success) {
        return new ActionBuilder().retry(action, maxAttempts, success).named("Retry").build();
    }

    public static Action retry(Action action, int maxAttempts, long delayMs, BooleanSupplier success) {
        return new ActionBuilder().retry(action, maxAttempts, delayMs, success).named("Retry").build();
    }

    public static Action retry(Runnable code, int maxAttempts, BooleanSupplier success) {
        return new ActionBuilder().retry(code, maxAttempts, success).named("Retry").build();
    }

    public static Action retry(Runnable code, int maxAttempts, long delayMs, BooleanSupplier success) {
        return new ActionBuilder().retry(code, maxAttempts, delayMs, success).named("Retry").build();
    }

    public static ActionBuilder builder() {
        return new ActionBuilder();
    }

    static void run(Action action) {
        int i = 0;
        while (i < active.size()) {
            Action a = active.get(i);
            if (conflicts(a, action)) {
                a.cancel();
                active.remove(i);
            } else {
                i++;
            }
        }
        // Drop a stale entry for the same action (re-schedule case).
        active.remove(action);
        action.reset();
        active.add(action);
    }

    private static boolean conflicts(Action a, Action b) {
        for (Module m : a.getTargets()) {
            if (b.getTargets().contains(m)) return true;
        }
        return false;
    }

    /** Tick every active action once. Called by EnhancedOpMode each loop. */
    public static void update() {
        int i = 0;
        while (i < active.size()) {
            Action a = active.get(i);
            if (a.tick()) {
                active.remove(i);
            } else {
                i++;
            }
        }
    }

    public static void cancelAll() {
        for (Action a : active) a.cancel();
        active.clear();
    }

    public static void cancelFor(Module... modules) {
        int i = 0;
        while (i < active.size()) {
            Action a = active.get(i);
            boolean match = false;
            for (Module m : modules) {
                if (a.getTargets().contains(m)) { match = true; break; }
            }
            if (match) {
                a.cancel();
                active.remove(i);
            } else {
                i++;
            }
        }
    }

    public static boolean hasActive() {
        return !active.isEmpty();
    }

    public static int activeCount() {
        return active.size();
    }

    public static boolean isModuleActive(Module module) {
        for (Action a : active) {
            if (a.getTargets().contains(module)) return true;
        }
        return false;
    }

    public static List<Action> getActive() {
        return new ArrayList<>(active);
    }

    static boolean contains(Action action) {
        return active.contains(action);
    }

    /** Alias for {@link #cancelAll}; kept so OpMode.stop() lifecycle callers don't change. */
    public static void shutdown() {
        cancelAll();
    }

    public static void reset() {
        cancelAll();
    }
}
