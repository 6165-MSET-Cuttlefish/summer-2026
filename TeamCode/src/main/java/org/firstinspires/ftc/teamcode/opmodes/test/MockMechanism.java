package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.core.Module;
import org.firstinspires.ftc.teamcode.core.State;

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
        // Re-bind only when the dashboard name changes, so a bad name doesn't re-throw every loop.
        String name = Tuning.pulseMotorName == null ? "" : Tuning.pulseMotorName.trim();
        if (!name.equals(boundName)) {
            boundName = name;
            motor = null;
            if (!name.isEmpty()) {
                try {
                    motor = new EnhancedMotor(hardwareMap, name);
                } catch (Exception e) {
                    // Unknown name — stay hardware-free instead of crashing the opmode.
                }
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
