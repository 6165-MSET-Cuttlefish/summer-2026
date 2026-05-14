package org.firstinspires.ftc.teamcode.architecture.hardware;

/**
 * Per-instance state for hardware wrappers: last-written value (skip rewrite within tolerance),
 * clamp bounds, optional voltage scaling. Composed into EnhancedMotor / EnhancedCRServo /
 * EnhancedServo so the wrappers stay short.
 */
final class WriteCache {
    double tolerance = 0.0;
    double cached = Double.NaN;
    double min = -1.0;
    double max = 1.0;

    boolean voltageCompensationEnabled = false;
    double referenceVoltage = 13.0;

    /** Force a write when crossing zero, hitting a rail freshly, or on first call. */
    boolean shouldWrite(double newValue) {
        return Math.abs(newValue - cached) > tolerance
                || (newValue == 0.0 && cached != 0.0)
                || (newValue >= max && !(cached >= max))
                || (newValue <= min && !(cached <= min))
                || Double.isNaN(cached);
    }

    double clamp(double value) {
        return Math.max(min, Math.min(max, value));
    }

    /** Symmetric: scales down on a fresh battery, up on a drained one. Skips zero-power. */
    double applyVoltageScaling(double power) {
        if (!voltageCompensationEnabled || power == 0.0) return power;
        return power * (referenceVoltage / HardwareVoltage.get());
    }

    void store(double value) {
        cached = value;
    }
}
