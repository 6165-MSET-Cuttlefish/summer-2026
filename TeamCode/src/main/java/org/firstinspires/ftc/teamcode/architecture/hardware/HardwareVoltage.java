package org.firstinspires.ftc.teamcode.architecture.hardware;

/**
 * Shared battery-voltage state used by hardware wrappers that do voltage compensation.
 * EnhancedOpMode publishes the latest reading once per loop; readers pull from
 * {@link #get()} on demand.
 */
public final class HardwareVoltage {
    private static volatile double current = 12.0;

    private HardwareVoltage() {}

    /** Publish the latest voltage reading. Called once per loop by EnhancedOpMode. */
    public static void update(double voltage) {
        current = voltage;
    }

    /** Most recent voltage reading. Defaults to 12.0 V before the first update. */
    public static double get() {
        return current;
    }
}
