package org.firstinspires.ftc.teamcode.architecture.action;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;

/**
 * Single-threaded cooperative scheduler; {@link #update} ticks every active action once.
 * EnhancedOpMode pumps between gameLoop() and writeModules() on the OpMode thread.
 */
public final class Actions {
    private Actions() {}

    private static final ArrayList<Action> active = new ArrayList<>();

    // Reentrancy guard: update() iterates `active` by index while ticking, and a Step's
    // start()/tick() may call schedule()/cancelFor()/cancelAll() reentrantly. Mutating `active`
    // mid-iteration would skip or double-visit actions, so during update() those calls are queued
    // here and applied once iteration finishes.
    private static boolean updating = false;
    private static final ArrayList<Runnable> deferredOps = new ArrayList<>();

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
        return b.withName("Sequence").build();
    }

    public static Action parallel(Action... actions) {
        return new ActionBuilder().parallel(actions).withName("Parallel").build();
    }

    public static Action race(Action... actions) {
        return new ActionBuilder().race(actions).withName("Race").build();
    }

    public static Action timeout(Action action, long ms) {
        return new ActionBuilder().timeout(action, ms).withName("Timeout").build();
    }

    public static Action repeat(Action action, int times) {
        return new ActionBuilder().repeat(action, times).withName("Repeat").build();
    }

    public static Action loop(Action action, BooleanSupplier condition) {
        return new ActionBuilder().loop(action, condition).withName("Loop").build();
    }

    public static Action ifThen(BooleanSupplier condition, Action ifTrue) {
        return new ActionBuilder().ifThen(condition, ifTrue).withName("IfThen").build();
    }

    public static Action ifElse(BooleanSupplier condition, Action ifTrue, Action ifFalse) {
        return new ActionBuilder().ifElse(condition, ifTrue, ifFalse).withName("IfElse").build();
    }

    public static Action retry(Action action, int maxAttempts, BooleanSupplier success) {
        return new ActionBuilder().retry(action, maxAttempts, success).withName("Retry").build();
    }

    public static Action retry(Action action, int maxAttempts, long delayMs, BooleanSupplier success) {
        return new ActionBuilder().retry(action, maxAttempts, delayMs, success).withName("Retry").build();
    }

    public static Action retry(Runnable code, int maxAttempts, BooleanSupplier success) {
        return new ActionBuilder().retry(code, maxAttempts, success).withName("Retry").build();
    }

    public static Action retry(Runnable code, int maxAttempts, long delayMs, BooleanSupplier success) {
        return new ActionBuilder().retry(code, maxAttempts, delayMs, success).withName("Retry").build();
    }

    public static ActionBuilder builder() {
        return new ActionBuilder();
    }

    static void schedule(Action action) {
        if (updating) { deferredOps.add(() -> schedule(action)); return; }
        if (action.isEmbedded()) {
            System.err.println("[Actions] Refusing to schedule '" + action.getName()
                    + "': it was composed into another action and shares mutable step state. "
                    + "Build a fresh action instead of reusing a composed one.");
            return;
        }
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
        // Re-schedule case: drop any stale entry for this action.
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

    public static void update() {
        updating = true;
        try {
            int i = 0;
            while (i < active.size()) {
                Action a = active.get(i);
                // A throwing step propagates out by design (fail-fast); the finally still clears
                // the reentrancy guard so a crashed tick can't wedge the scheduler.
                if (a.tick()) {
                    // Tick may have already removed us (self-conflicting schedule from within tick).
                    if (i < active.size() && active.get(i) == a) {
                        active.remove(i);
                    }
                } else {
                    i++;
                }
            }
        } finally {
            updating = false;
        }
        drainDeferredOps();
    }

    private static void drainDeferredOps() {
        // Ops may enqueue further ops (e.g. a cancel callback that schedules); drain to empty.
        while (!deferredOps.isEmpty()) {
            ArrayList<Runnable> batch = new ArrayList<>(deferredOps);
            deferredOps.clear();
            for (int i = 0; i < batch.size(); i++) batch.get(i).run();
        }
    }

    public static void cancelAll() {
        if (updating) { deferredOps.add(Actions::cancelAll); return; }
        // Drop any ops orphaned by a crashed update() so stale statics can't survive a Sloth reload.
        deferredOps.clear();
        // Snapshot + clear first so a schedule from a cancel callback lands in an empty list.
        Action[] snapshot = active.toArray(new Action[0]);
        active.clear();
        for (Action a : snapshot) a.cancel();
    }

    public static void cancelFor(Module... modules) {
        if (updating) {
            final Module[] mods = modules;
            deferredOps.add(() -> cancelFor(mods));
            return;
        }
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

    public static void shutdown() {
        cancelAll();
    }

    public static void reset() {
        cancelAll();
    }
}
