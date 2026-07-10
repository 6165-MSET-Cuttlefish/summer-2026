package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;

/**
 * Mock module for the architecture smoke test. By default it touches no hardware (just drives the
 * State machine + telemetry). Set {@link Tuning#pulseMotorName} on FtcDashboard to a motor in the
 * config and it will pulse that motor at {@link Tuning#pulsePower} while ACTIVE and 0 while IDLE —
 * proving the EnhancedMotor write path (clamp → voltage-comp → write cache → I2C) reaches hardware.
 *
 * <p>Blank name = no hardware, so the no-motion smoke test stays safe by default. Don't name a
 * drivetrain motor (fl/bl/fr/br) — it fights the Pedro follower.
 */
public class MockMechanism extends Module {
    public enum Status implements State {
        IDLE(0), ACTIVE(1);
        Status(double value) { setValue(value); }
    }

    @Config
    public static class Tuning {
        /** Hardware-map name of a motor to pulse when ACTIVE. Blank = no hardware (safe default). */
        public static String pulseMotorName = "";
        /** Power applied while ACTIVE (0 while IDLE). Keep small. */
        public static double pulsePower = 0.2;
    }

    private final HardwareMap hardwareMap;
    private EnhancedMotor motor;
    private String boundName = "";

    public MockMechanism(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override protected void initStates() { setStates(Status.IDLE); }

    @Override
    protected void read() {
        // Re-bind only when the dashboard name changes.
        String name = Tuning.pulseMotorName == null ? "" : Tuning.pulseMotorName.trim();
        if (!name.equals(boundName)) {
            boundName = name;
            // Safe the previously-bound motor before dropping it, or switching test targets
            // mid-run leaves the old one spinning (write()/stop() only touch the current motor).
            if (motor != null) motor.setPower(0.0);
            motor = null;
            // No try/catch: an unknown hardware name throws (fail-fast) instead of silently
            // staying hardware-free. boundName is already updated, so it throws once, not every loop.
            if (!name.isEmpty()) {
                motor = new EnhancedMotor(hardwareMap, name);
            }
        }
    }

    @Override
    protected void write() {
        if (motor != null) motor.setPower(isInAny(Status.ACTIVE) ? Tuning.pulsePower : 0.0);
    }

    @Override
    public void stop() {
        if (motor != null) motor.setPower(0.0);
    }

    public boolean isMotorBound() { return motor != null; }
    public String boundMotorName() { return boundName.isEmpty() ? "(none)" : boundName; }
}
