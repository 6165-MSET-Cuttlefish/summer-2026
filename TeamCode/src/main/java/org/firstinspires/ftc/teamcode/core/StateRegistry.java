package org.firstinspires.ftc.teamcode.core;

import java.util.concurrent.ConcurrentHashMap;

/**
 * Backing store for {@link State}'s default value/module accessors. Lives in its own class so
 * the maps can be cleared between OpMode runs (statics persist across opmode restarts in the
 * same JVM, which is fine for the values themselves, but means a stale mapping from a state to
 * an old Module instance can survive a teardown).
 *
 * <p>Maps are keyed by State; each State enum constant is its own key.
 */
public final class StateRegistry {
    private StateRegistry() {}

    /** Current numeric value associated with a state (e.g. servo position, motor power). */
    static final ConcurrentHashMap<State, Double> VALUES = new ConcurrentHashMap<>();

    /** First value ever assigned to each state; used by {@link State#resetValue()}. */
    static final ConcurrentHashMap<State, Double> INITIAL_VALUES = new ConcurrentHashMap<>();

    /** The module that owns each state and handles its transitions. */
    static final ConcurrentHashMap<State, Module> MODULES = new ConcurrentHashMap<>();

    /**
     * Drop every cached state→module association. Initial values are left intact so
     * {@link State#resetValue()} keeps working across opmode runs. Call from OpMode init when
     * you want a clean slate.
     */
    public static void clearModuleBindings() {
        MODULES.clear();
    }

    /**
     * Wipe every map. Use only when you genuinely want to forget initial values too — e.g.
     * after a config-tunable change that should re-baseline.
     */
    public static void clearAll() {
        VALUES.clear();
        INITIAL_VALUES.clear();
        MODULES.clear();
    }
}
