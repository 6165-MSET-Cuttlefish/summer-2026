package org.firstinspires.ftc.teamcode.core.state;

import org.firstinspires.ftc.teamcode.core.Module;
import org.firstinspires.ftc.teamcode.core.action.Action;
import org.firstinspires.ftc.teamcode.core.action.Actions;

/**
 * Marker interface for state-machine states. Implement on an enum and call {@link #setValue(double)}
 * in the constructor to attach a numeric setpoint (servo position, motor power, RPM).
 *
 * <p>Backing maps live in {@link StateRegistry} so they can be cleared between OpMode runs.
 */
public interface State {

    default double getValue() {
        return StateRegistry.VALUES.getOrDefault(this, 0.0);
    }

    default void setValue(double value) {
        StateRegistry.INITIAL_VALUES.putIfAbsent(this, value);
        StateRegistry.VALUES.put(this, value);
    }

    default void resetValue() {
        Double initial = StateRegistry.INITIAL_VALUES.get(this);
        if (initial != null) StateRegistry.VALUES.put(this, initial);
    }

    default Module getModule() {
        return StateRegistry.MODULES.get(this);
    }

    default void setModule(Module module) {
        StateRegistry.MODULES.put(this, module);
    }

    /** Transition this state's owning module to this state. False if no module is bound or a guard rejected. */
    default boolean apply() {
        Module m = getModule();
        if (m == null) {
            System.err.println("[State] apply() called on " + this + " but no module is registered");
            return false;
        }
        return m.set(this);
    }

    default Module requireModule() {
        Module m = getModule();
        if (m == null) {
            throw new IllegalStateException(
                    "State " + this + " has no registered module. "
                    + "Call setStates() with this state during Module.initStates().");
        }
        return m;
    }

    default Action asAction() {
        return Actions.set(this);
    }
}
