package org.firstinspires.ftc.teamcode.core;

/**
 * Marker interface for state-machine states in robot modules. Implement on an enum and use the
 * default methods to attach a numeric value (servo position, motor power, etc.) and bind the
 * state to its owning {@link Module}.
 *
 * <p>Storage lives in {@link StateRegistry} so it can be cleared between OpMode runs.
 *
 * <p>Example:
 * <pre>{@code
 * public enum DoorState implements State {
 *     OPEN(0.5),
 *     CLOSED(0.0);
 *
 *     DoorState(double value) {
 *         setValue(value);
 *     }
 * }
 * }</pre>
 */
public interface State {

    /** Numeric value associated with this state, or 0 if none has been set. */
    default double getValue() {
        return StateRegistry.VALUES.getOrDefault(this, 0.0);
    }

    /**
     * Update the cached numeric value for this state. The first call also stores the value as
     * the initial baseline for {@link #resetValue()}.
     */
    default void setValue(double value) {
        StateRegistry.INITIAL_VALUES.putIfAbsent(this, value);
        StateRegistry.VALUES.put(this, value);
    }

    /** Restore this state to its initial value, if one was ever recorded. */
    default void resetValue() {
        Double initial = StateRegistry.INITIAL_VALUES.get(this);
        if (initial != null) StateRegistry.VALUES.put(this, initial);
    }

    /** The module that owns this state. {@code null} until {@link #setModule(Module)} is called. */
    default Module getModule() {
        return StateRegistry.MODULES.get(this);
    }

    /** Bind this state to a module. Called from {@link Module#setStates}. */
    default void setModule(Module module) {
        StateRegistry.MODULES.put(this, module);
    }

    /**
     * Transition to this state via its owning module. Returns true if the transition succeeded,
     * false if the module rejected it (e.g. blocked by a guard).
     */
    default boolean apply() {
        Module m = getModule();
        if (m == null) {
            System.err.println("[State] apply() called on " + this
                    + " but no module is registered");
            return false;
        }
        return m.set(this);
    }

    /**
     * Like {@link #getModule()} but throws if no module is bound. Useful for fail-fast lookups
     * that don't have a sensible null-handling path.
     */
    default Module requireModule() {
        Module m = getModule();
        if (m == null) {
            throw new IllegalStateException(
                    "State " + this + " has no registered module. "
                    + "Call setStates() with this state during Module.initStates().");
        }
        return m;
    }

    /** Convert this state to an Action that applies it. */
    default Action asAction() {
        return Actions.set(this);
    }
}
