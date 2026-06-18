package org.firstinspires.ftc.teamcode.architecture.input;

import java.util.function.BooleanSupplier;

public class EdgeBooleanSupplier {
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
    private static final long DOUBLE_CLICK_TIMEOUT_NS = 300_000_000L;
    private long doubleClickTimeoutNs = DOUBLE_CLICK_TIMEOUT_NS;

    public EdgeBooleanSupplier(
            BooleanSupplier booleanSupplier, double risingDebounce, double fallingDebounce) {
        this.booleanSupplier = booleanSupplier;
        this.risingDebounce = (long) (risingDebounce * 1E9);
        this.fallingDebounce = (long) (fallingDebounce * 1E9);
        this.previous = booleanSupplier.getAsBoolean();
        this.current = previous;
        this.toggleTrue = current;
        this.toggleFalse = current;
    }

    public EdgeBooleanSupplier(BooleanSupplier booleanSupplier) {
        this(booleanSupplier, 0.0, 0.0);
    }

    private void update() {
        previous = current;
        long time = System.nanoTime();
        doubleClickDetected = false;
        // Source may be a chained predicate; sample once.
        boolean raw = booleanSupplier.getAsBoolean();

        if (!current && raw) {
            if (time - timeMarker >= risingDebounce) {
                current = true;
                toggleTrue = !toggleTrue;

                if (time - lastRisingEdgeTime <= doubleClickTimeoutNs) {
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
        // Update immediately so edge state advances even if this supplier isn't queried this frame.
        update();
        valid = true;
    }

    public void primeToCurrentState() {
        boolean state = booleanSupplier.getAsBoolean();
        previous = state;
        current = state;
        valid = true;
        doubleClickDetected = false;
        // Preserve toggleTrue/toggleFalse: priming suppresses spurious edges across a layer
        // switch / re-init — it must NOT clear toggle latches the operator already set.
        // Drop the prior rising-edge timestamp so presses across a prime gap (e.g. layer switch)
        // don't register as a double-click.
        lastRisingEdgeTime = 0L;
        timeMarker = System.nanoTime();
    }

    private void ensureFresh() {
        if (!valid) {
            update();
            valid = true;
        }
    }

    public boolean getValue() {
        ensureFresh();
        return current;
    }

    public boolean wasJustPressed() {
        ensureFresh();
        return current && !previous;
    }

    public boolean wasJustReleased() {
        ensureFresh();
        return !current && previous;
    }

    public boolean isDown() {
        ensureFresh();
        return current;
    }

    public boolean isToggledOn() {
        ensureFresh();
        return toggleTrue;
    }

    public boolean getDoubleClick() {
        ensureFresh();
        return doubleClickDetected;
    }

    public EdgeBooleanSupplier setDoubleClickTimeout(double timeoutSeconds) {
        this.doubleClickTimeoutNs = (long) (timeoutSeconds * 1E9);
        return this;
    }

    public EdgeBooleanSupplier debounce(double debounce) {
        return new EdgeBooleanSupplier(this.booleanSupplier, debounce, debounce);
    }

    public EdgeBooleanSupplier debounce(double rising, double falling) {
        return new EdgeBooleanSupplier(this.booleanSupplier, rising, falling);
    }

    public EdgeBooleanSupplier debounceRisingEdge(double debounce) {
        return new EdgeBooleanSupplier(
                this.booleanSupplier, debounce, this.fallingDebounce / 1E9);
    }

    public EdgeBooleanSupplier debounceFallingEdge(double debounce) {
        return new EdgeBooleanSupplier(
                this.booleanSupplier, this.risingDebounce / 1E9, debounce);
    }

    // Combinators compose on the RAW underlying sources and let only the returned supplier own the
    // edge/debounce state. Composing on this.getValue()/other.getValue() would re-run each operand's
    // update() lazily and apply its debounce a second time, corrupting the combined edge timing.
    // The returned supplier is the single edge-detector for the combination; debounce it if needed.

    public EdgeBooleanSupplier and(BooleanSupplier other) {
        return new EdgeBooleanSupplier(() -> booleanSupplier.getAsBoolean() && other.getAsBoolean());
    }

    public EdgeBooleanSupplier and(EdgeBooleanSupplier other) {
        return new EdgeBooleanSupplier(() -> booleanSupplier.getAsBoolean() && other.booleanSupplier.getAsBoolean());
    }

    public EdgeBooleanSupplier or(BooleanSupplier other) {
        return new EdgeBooleanSupplier(() -> booleanSupplier.getAsBoolean() || other.getAsBoolean());
    }

    public EdgeBooleanSupplier or(EdgeBooleanSupplier other) {
        return new EdgeBooleanSupplier(() -> booleanSupplier.getAsBoolean() || other.booleanSupplier.getAsBoolean());
    }

    public EdgeBooleanSupplier xor(BooleanSupplier other) {
        return new EdgeBooleanSupplier(() -> booleanSupplier.getAsBoolean() ^ other.getAsBoolean());
    }

    public EdgeBooleanSupplier xor(EdgeBooleanSupplier other) {
        return new EdgeBooleanSupplier(() -> booleanSupplier.getAsBoolean() ^ other.booleanSupplier.getAsBoolean());
    }

    public EdgeBooleanSupplier not() {
        // Invert the raw source; rising/falling debounce swap because press/release swap.
        return new EdgeBooleanSupplier(
                () -> !booleanSupplier.getAsBoolean(), fallingDebounce / 1E9, risingDebounce / 1E9);
    }
}
