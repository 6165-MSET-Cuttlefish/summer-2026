package org.firstinspires.ftc.teamcode.core.state;

import org.firstinspires.ftc.teamcode.core.Module;

import java.util.concurrent.ConcurrentHashMap;

/**
 * Backing maps for {@link State}'s default methods. Held as statics so values persist across
 * OpMode reconstruction within the same JVM (Sloth hot-reload, repeated re-runs); call
 * {@link #clearModuleBindings()} from OpMode init to drop stale State→Module pointers.
 */
public final class StateRegistry {
    private StateRegistry() {}

    static final ConcurrentHashMap<State, Double> VALUES = new ConcurrentHashMap<>();
    static final ConcurrentHashMap<State, Double> INITIAL_VALUES = new ConcurrentHashMap<>();
    static final ConcurrentHashMap<State, Module> MODULES = new ConcurrentHashMap<>();

    public static void clearModuleBindings() {
        MODULES.clear();
    }

    public static void clearAll() {
        VALUES.clear();
        INITIAL_VALUES.clear();
        MODULES.clear();
    }
}
