package org.firstinspires.ftc.teamcode.architecture.input.suppliers;

import java.util.function.BooleanSupplier;

public class EnhancedBooleanSupplier {
    private final BooleanSupplier booleanSupplier;
    private final long risingDebounce;
    private final long fallingDebounce;

    private boolean previous;
    private boolean current;
    private boolean toggleTrue;
    private boolean toggleFalse;
    private long timeMarker = 0L;
    private boolean valid = false;

    private long lastRisingEdgeTime = 0L;
    private boolean doubleClickDetected = false;
    private static final long DOUBLE_CLICK_WINDOW = 300_000_000L;
    private long doubleClickWindow = DOUBLE_CLICK_WINDOW;

    public EnhancedBooleanSupplier(
            BooleanSupplier booleanSupplier, double risingDebounce, double fallingDebounce) {
        this.booleanSupplier = booleanSupplier;
        this.risingDebounce = (long) (risingDebounce * 1E9);
        this.fallingDebounce = (long) (fallingDebounce * 1E9);
        this.previous = booleanSupplier.getAsBoolean();
        this.current = previous;
        this.toggleTrue = current;
        this.toggleFalse = current;
    }

    public EnhancedBooleanSupplier(BooleanSupplier booleanSupplier) {
        this(booleanSupplier, 0.0, 0.0);
    }

    private void update() {
        previous = current;
        long time = System.nanoTime();
        doubleClickDetected = false;
        // Read the source once — supplier may be a chained predicate that's not free to call twice.
        boolean raw = booleanSupplier.getAsBoolean();

        if (!current && raw) {
            if (time - timeMarker >= risingDebounce) {
                current = true;
                toggleTrue = !toggleTrue;

                if (time - lastRisingEdgeTime <= doubleClickWindow) {
                    doubleClickDetected = true;
                }
                lastRisingEdgeTime = time;
                timeMarker = time;
            }

        } else if (current && !raw) {
            if (time - timeMarker >= fallingDebounce) {
                current = false;
                toggleFalse = !toggleFalse;
                timeMarker = time;
            }
        } else {
            timeMarker = time;
        }
    }

    public void invalidate() {
        // Invalidate is called once per loop by the input system; update immediately
        // so edge state advances even if this supplier is not queried this frame.
        update();
        valid = true;
    }

    public void primeToCurrentState() {
        boolean state = booleanSupplier.getAsBoolean();
        previous = state;
        current = state;
        toggleTrue = state;
        toggleFalse = state;
        valid = true;
        doubleClickDetected = false;
        timeMarker = System.nanoTime();
    }

    public boolean getState() {
        if (!valid) {
            update();
            valid = true;
        }
        return current;
    }

    public boolean wasJustPressed() {
        return getState() && !previous;
    }

    public boolean wasJustReleased() {
        return !getState() && previous;
    }

    public boolean isDown() {
        return getState();
    }

    public boolean isToggledOn() {
        getState();
        return toggleTrue;
    }

    public boolean getDoubleClick() {
        getState();
        return doubleClickDetected;
    }

    public EnhancedBooleanSupplier setDoubleClickWindow(double windowSeconds) {
        this.doubleClickWindow = (long) (windowSeconds * 1E9);
        return this;
    }

    public EnhancedBooleanSupplier debounce(double debounce) {
        return new EnhancedBooleanSupplier(this.booleanSupplier, debounce, debounce);
    }

    public EnhancedBooleanSupplier debounce(double rising, double falling) {
        return new EnhancedBooleanSupplier(this.booleanSupplier, rising, falling);
    }

    public EnhancedBooleanSupplier debounceRisingEdge(double debounce) {
        return new EnhancedBooleanSupplier(
                this.booleanSupplier, debounce, this.fallingDebounce / 1E9);
    }

    public EnhancedBooleanSupplier debounceFallingEdge(double debounce) {
        return new EnhancedBooleanSupplier(
                this.booleanSupplier, this.risingDebounce / 1E9, debounce);
    }

    public EnhancedBooleanSupplier and(BooleanSupplier other) {
        return new EnhancedBooleanSupplier(() -> this.getState() && other.getAsBoolean());
    }

    public EnhancedBooleanSupplier and(EnhancedBooleanSupplier other) {
        return new EnhancedBooleanSupplier(() -> this.getState() && other.getState());
    }

    public EnhancedBooleanSupplier or(BooleanSupplier other) {
        return new EnhancedBooleanSupplier(() -> this.getState() || other.getAsBoolean());
    }

    public EnhancedBooleanSupplier or(EnhancedBooleanSupplier other) {
        return new EnhancedBooleanSupplier(() -> this.getState() || other.getState());
    }

    public EnhancedBooleanSupplier xor(BooleanSupplier other) {
        return new EnhancedBooleanSupplier(() -> this.getState() ^ other.getAsBoolean());
    }

    public EnhancedBooleanSupplier xor(EnhancedBooleanSupplier other) {
        return new EnhancedBooleanSupplier(() -> this.getState() ^ other.getState());
    }

    public EnhancedBooleanSupplier not() {
        return new EnhancedBooleanSupplier(
                () -> !this.getState(), fallingDebounce / 1E9, risingDebounce / 1E9);
    }
}
