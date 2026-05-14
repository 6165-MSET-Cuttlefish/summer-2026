package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.architecture.telemetry.DualTelemetry;
import org.firstinspires.ftc.teamcode.core.action.Action;
import org.firstinspires.ftc.teamcode.core.State;

/** Base class for robot subsystem modules. */
public abstract class Module {
    private static final int DEFAULT_HISTORY_SIZE = 50;
    private int maxHistorySize = DEFAULT_HISTORY_SIZE;

    private final List<State> states = new ArrayList<>();
    private final Map<Class<? extends State>, State> stateMap = new HashMap<>();
    private final List<TransitionGuard<?>> guards = new ArrayList<>();
    private final List<StateHook> enterHooks = new ArrayList<>();
    private final List<StateHook> exitHooks = new ArrayList<>();
    private final Deque<State> stateHistory = new ArrayDeque<>();
    private final List<Runnable> tunableRefreshers = new ArrayList<>();

    private final ElapsedTime stateTimer = new ElapsedTime();
    private Telemetry telemetry;
    private Action defaultAction;
    private boolean telemetryEnabled = true;
    private boolean writeEnabled = true;
    private String name;
    private String readSectionName;
    private String writeSectionName;

    public Module() {
        this.name = getClass().getSimpleName();
        recomputeSectionNames();
        // initStates() runs from EnhancedOpMode.initModules() after the subclass ctor returns,
        // so state→module bindings aren't yet in place during the subclass ctor — use init()
        // or initStates() for code that references registered states.
    }

    private void recomputeSectionNames() {
        this.readSectionName = "read." + this.name;
        this.writeSectionName = "write." + this.name;
    }

    public final String getReadSectionName() { return readSectionName; }
    public final String getWriteSectionName() { return writeSectionName; }

    protected abstract void initStates();
    protected abstract void read();
    protected abstract void write();

    public void init() {}

    /** Override to put hardware in a safe state on OpMode stop. */
    public void stop() {}

    protected void onStateChange() {}
    protected void onTelemetry() {}

    /**
     * Wire a State's setpoint to a live source (typically a {@code @Config static} field). The
     * framework refreshes via {@code state.setValue(supplier.getAsDouble())} once per loop
     * before {@link #read()}. Call after {@link #setStates(State...)} in {@link #initStates()}.
     */
    protected final void bindTunable(State state, DoubleSupplier supplier) {
        Runnable refresh = () -> state.setValue(supplier.getAsDouble());
        refresh.run();
        tunableRefreshers.add(refresh);
    }

    final void refreshTunables() {
        for (int i = 0; i < tunableRefreshers.size(); i++) {
            tunableRefreshers.get(i).run();
        }
    }

    /** Render order in the MODULES telemetry section; lower first. */
    public int telemetryOrder() { return 0; }

    /**
     * Register the initial state for each state-class. For enums, every variant is bound to
     * this module and reset to its initial value so a re-run starts clean.
     */
    protected final void setStates(State... initialStates) {
        states.clear();
        stateMap.clear();
        for (State s : initialStates) {
            s.setModule(this);
            states.add(s);
            stateMap.put(s.getClass(), s);

            if (s instanceof Enum<?>) {
                // getDeclaringClass handles enum constants with bodies (anonymous subclasses
                // whose own getEnumConstants() returns null).
                Class<?> enumClass = ((Enum<?>) s).getDeclaringClass();
                Object[] constants = enumClass.getEnumConstants();
                if (constants != null) {
                    for (Object c : constants) {
                        State variant = (State) c;
                        variant.setModule(this);
                        variant.resetValue();
                    }
                }
            }
        }
    }

    /** O(1) map lookup, with a list-scan fallback to preserve isInstance() semantics. */
    @SuppressWarnings("unchecked")
    public final <T extends State> T getState(Class<T> stateClass) {
        State s = stateMap.get(stateClass);
        if (s != null) return (T) s;
        for (int i = 0; i < states.size(); i++) {
            State candidate = states.get(i);
            if (stateClass.isInstance(candidate)) return (T) candidate;
        }
        throw new IllegalArgumentException("No state of type: " + stateClass.getSimpleName());
    }

    /**
     * Transition to {@code newState}. True on success/no-op; false when the state class isn't
     * registered or a guard rejected the transition.
     */
    public final boolean setState(State newState) {
        for (int i = 0; i < states.size(); i++) {
            State current = states.get(i);
            if (newState.getClass() != current.getClass()) continue;
            if (newState.equals(current)) return true;

            if (!checkGuards(current, newState)) return false;

            fireExitHooks(current);

            stateHistory.addLast(current);
            if (stateHistory.size() > maxHistorySize) stateHistory.pollFirst();

            states.set(i, newState);
            stateMap.put(newState.getClass(), newState);
            newState.setModule(this);
            stateTimer.reset();

            fireEnterHooks(newState);
            onStateChange();
            return true;
        }
        return false;
    }

    public final boolean isInAny(State... checkStates) {
        for (State check : checkStates) {
            if (getState(check.getClass()).equals(check)) return true;
        }
        return false;
    }

    public final boolean isInAll(State... checkStates) {
        for (State check : checkStates) {
            if (!getState(check.getClass()).equals(check)) return false;
        }
        return true;
    }

    public final long stateTimeMs() {
        return (long) stateTimer.milliseconds();
    }

    public final List<State> getHistory() {
        return Collections.unmodifiableList(new ArrayList<>(stateHistory));
    }

    public final void setMaxHistorySize(int size) {
        this.maxHistorySize = Math.max(1, size);
    }

    public final int getMaxHistorySize() {
        return maxHistorySize;
    }

    protected final <T extends State> void guard(Class<T> stateClass, TransitionCheck<T> guard) {
        guards.add(new TransitionGuard<>(stateClass, guard));
    }

    protected final void onEnter(State state, Runnable action) {
        enterHooks.add(new StateHook(state, action));
    }

    protected final void onExit(State state, Runnable action) {
        exitHooks.add(new StateHook(state, action));
    }

    @SuppressWarnings("unchecked")
    private <T extends State> boolean checkGuards(State from, State to) {
        // Enum constants with method bodies return an anonymous subclass from getClass();
        // getDeclaringClass() returns the declared type that callers actually register.
        Class<?> fromKey = (from instanceof Enum<?>)
                ? ((Enum<?>) from).getDeclaringClass()
                : from.getClass();
        for (TransitionGuard<?> g : guards) {
            if (g.stateClass == fromKey) {
                TransitionGuard<T> typedGuard = (TransitionGuard<T>) g;
                if (!typedGuard.check.test((T) from, (T) to)) return false;
            }
        }
        return true;
    }

    private void fireEnterHooks(State state) {
        for (StateHook h : enterHooks) {
            if (h.state.equals(state)) h.action.run();
        }
    }

    private void fireExitHooks(State state) {
        for (StateHook h : exitHooks) {
            if (h.state.equals(state)) h.action.run();
        }
    }

    final void setTelemetry(Telemetry t) { this.telemetry = t; }

    protected final Telemetry getTelemetry() { return telemetry; }

    protected final String getStateString() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < states.size(); i++) {
            if (i > 0) sb.append(" | ");
            sb.append(states.get(i));
        }
        return sb.toString();
    }

    protected void telemetry() {
        if (!telemetryEnabled || telemetry == null) return;

        DualTelemetry et = getDualTelemetry();
        if (et != null) {
            et.addDashboardData(name, getStateString());
        } else {
            telemetry.addData(name, getStateString());
        }
        onTelemetry();
    }

    public final Module withName(String name) {
        this.name = name;
        recomputeSectionNames();
        return this;
    }

    public final String getName() { return name; }

    public final void setTelemetryEnabled(boolean enabled) { this.telemetryEnabled = enabled; }

    public final void setWriteEnabled(boolean enabled) { this.writeEnabled = enabled; }
    public final boolean isWriteEnabled() { return writeEnabled; }

    public final void setDefaultAction(Action action) { this.defaultAction = action; }
    public final Action getDefaultAction() { return defaultAction; }

    protected final DualTelemetry getDualTelemetry() {
        return telemetry instanceof DualTelemetry ? (DualTelemetry) telemetry : null;
    }

    protected final void logDS(String caption, Object value) {
        DualTelemetry et = getDualTelemetry();
        if (et != null) et.addDSData(name + " " + caption, value);
        else log(caption, value);
    }

    protected final void logDS(String caption, String format, Object... args) {
        DualTelemetry et = getDualTelemetry();
        if (et != null) et.addDSData(name + " " + caption, format, args);
        else log(caption, format, args);
    }

    protected final void logDashboard(String caption, Object value) {
        DualTelemetry et = getDualTelemetry();
        if (et != null) et.addDashboardData(name + " " + caption, value);
        else log(caption, value);
    }

    protected final void logDashboard(String caption, String format, Object... args) {
        DualTelemetry et = getDualTelemetry();
        if (et != null) et.addDashboardData(name + " " + caption, format, args);
        else log(caption, format, args);
    }

    protected final void log(String caption, Object value) {
        if (telemetryEnabled && telemetry != null) {
            telemetry.addData(name + " " + caption, value);
        }
    }

    protected final void log(String caption, String format, Object... args) {
        if (telemetryEnabled && telemetry != null) {
            telemetry.addData(name + " " + caption, String.format(format, args));
        }
    }

    @FunctionalInterface
    public interface TransitionCheck<T extends State> {
        boolean test(T from, T to);
    }

    private static class TransitionGuard<T extends State> {
        final Class<T> stateClass;
        final TransitionCheck<T> check;

        TransitionGuard(Class<T> stateClass, TransitionCheck<T> check) {
            this.stateClass = stateClass;
            this.check = check;
        }
    }

    private static class StateHook {
        final State state;
        final Runnable action;

        StateHook(State state, Runnable action) {
            this.state = state;
            this.action = action;
        }
    }
}
