package org.firstinspires.ftc.teamcode.architecture.hardware;

/**
 * Per-instance state for hardware wrappers that want to (a) cache the last value written and
 * skip rewrites within a tolerance, (b) clamp values into a min/max range, and (c) optionally
 * scale by battery voltage.
 *
 * <p>Used via composition by {@link EnhancedMotor}, {@link EnhancedCRServo}, and (in a
 * tolerance-only flavor) {@link EnhancedServo}, so the actual hardware wrappers stay short and
 * focused on forwarding the rest of their interface.
 */
final class WriteCache {
    double tolerance = 0.0;
    double cached = Double.NaN;
    double min = -1.0;
    double max = 1.0;

    boolean voltageCompensationEnabled = false;
    double referenceVoltage = 13.0;

    /**
     * Decide whether {@code newValue} differs enough from the cached value to be worth writing
     * to hardware. Forces a write whenever crossing zero, hitting a rail freshly, or starting
     * up (cached is NaN).
     */
    boolean shouldWrite(double newValue) {
        return Math.abs(newValue - cached) > tolerance
                || (newValue == 0.0 && cached != 0.0)
                || (newValue >= max && !(cached >= max))
                || (newValue <= min && !(cached <= min))
                || Double.isNaN(cached);
    }

    /** Clamp {@code value} into [min, max]. */
    double clamp(double value) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * If voltage compensation is enabled and {@code power} is non-zero, scale by
     * {@code referenceVoltage / currentVoltage}. Symmetric: scales down on a fresh battery,
     * up on a drained one.
     */
    double applyVoltageScaling(double power) {
        if (!voltageCompensationEnabled || power == 0.0) return power;
        return power * (referenceVoltage / HardwareVoltage.get());
    }

    /** Store {@code value} as the new cached value (caller does this after a successful write). */
    void store(double value) {
        cached = value;
    }
}
