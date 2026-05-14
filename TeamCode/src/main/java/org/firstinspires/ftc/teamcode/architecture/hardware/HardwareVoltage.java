package org.firstinspires.ftc.teamcode.architecture.hardware;

/**
 * Shared battery-voltage state. EnhancedOpMode publishes once per loop; voltage-compensating
 * hardware wrappers pull on demand.
 */
public final class HardwareVoltage {
    private static volatile double current = 12.0;

    private HardwareVoltage() {}

    public static void update(double voltage) {
        current = voltage;
    }

    public static double get() {
        return current;
    }
}
