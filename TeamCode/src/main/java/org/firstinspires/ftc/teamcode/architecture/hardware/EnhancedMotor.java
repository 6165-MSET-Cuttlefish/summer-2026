package org.firstinspires.ftc.teamcode.architecture.hardware;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Enhanced DcMotorEx wrapper with power caching, bounds, and voltage compensation. Configure
 * by chaining: {@code new EnhancedMotor(motor).withCachingTolerance(0.02).withVoltageCompensation(13.5)}.
 */
public class EnhancedMotor implements DcMotorEx {
    private final DcMotorEx motor;
    private final WriteCache cache = new WriteCache();

    public EnhancedMotor(DcMotorEx motor) {
        this.motor = motor;
    }

    public EnhancedMotor(HardwareMap hardwareMap, String name) {
        this.motor = hardwareMap.get(DcMotorEx.class, name);
    }

    // ─── caching / bounds / voltage configuration ────────────────────────────

    /**
     * Set how much power must change before writing to hardware.
     * Reduces communication traffic and improves loop times.
     * @param tolerance Power change threshold (0.0 to 2.0).
     */
    public EnhancedMotor withCachingTolerance(double tolerance) {
        cache.tolerance = tolerance;
        return this;
    }

    public EnhancedMotor withPowerBounds(double min, double max) {
        cache.min = min;
        cache.max = max;
        return this;
    }

    /**
     * Enable voltage compensation; {@code referenceVoltage} is the battery voltage at which a
     * power command of 1.0 would produce full output (typically 13.5 V).
     */
    public EnhancedMotor withVoltageCompensation(double referenceVoltage) {
        cache.voltageCompensationEnabled = true;
        cache.referenceVoltage = referenceVoltage;
        return this;
    }

    /** Publish a fresh battery voltage reading; called by EnhancedOpMode each loop. */
    public static void updateVoltage(double voltage) {
        HardwareVoltage.update(voltage);
    }

    public static double getCurrentVoltage() {
        return HardwareVoltage.get();
    }

    // ─── setPower with caching ───────────────────────────────────────────────

    @Override
    public void setPower(double power) {
        double corrected = cache.clamp(cache.applyVoltageScaling(power));
        if (cache.shouldWrite(corrected)) {
            cache.store(corrected);
            motor.setPower(corrected);
        }
    }

    /** Set power directly without voltage compensation. Useful for testing. */
    public void setPowerRaw(double power) {
        double corrected = cache.clamp(power);
        cache.store(corrected);
        motor.setPower(corrected);
    }

    // ─── setters / getters for compensation config ───────────────────────────

    public void setCachingTolerance(double tolerance) {
        cache.tolerance = Math.max(0.0, Math.min(1.0, tolerance));
    }

    public double getCachingTolerance() { return cache.tolerance; }

    public void setReferenceVoltage(double voltage) { cache.referenceVoltage = voltage; }
    public double getReferenceVoltage() { return cache.referenceVoltage; }

    public void setVoltageCompensationEnabled(boolean enabled) {
        cache.voltageCompensationEnabled = enabled;
    }

    public boolean isVoltageCompensationEnabled() {
        return cache.voltageCompensationEnabled;
    }

    public DcMotorEx getUnderlying() { return motor; }
    public double getCachedPower() { return cache.cached; }

    // ─── DcMotorEx interface (forwarded) ─────────────────────────────────────

    @Override public void setMotorEnable() { motor.setMotorEnable(); }
    @Override public void setMotorDisable() { motor.setMotorDisable(); }
    @Override public boolean isMotorEnabled() { return motor.isMotorEnabled(); }

    @Override public void setVelocity(double angularRate) { motor.setVelocity(angularRate); }
    @Override public void setVelocity(double angularRate, AngleUnit unit) { motor.setVelocity(angularRate, unit); }
    @Override public double getVelocity() { return motor.getVelocity(); }
    @Override public double getVelocity(AngleUnit unit) { return motor.getVelocity(unit); }

    @Override public void setPIDCoefficients(RunMode mode, PIDCoefficients pid) { motor.setPIDCoefficients(mode, pid); }
    @Override public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidf) { motor.setPIDFCoefficients(mode, pidf); }
    @Override public void setVelocityPIDFCoefficients(double p, double i, double d, double f) { motor.setVelocityPIDFCoefficients(p, i, d, f); }
    @Override public void setPositionPIDFCoefficients(double p) { motor.setPositionPIDFCoefficients(p); }
    @Override public PIDCoefficients getPIDCoefficients(RunMode mode) { return motor.getPIDCoefficients(mode); }
    @Override public PIDFCoefficients getPIDFCoefficients(RunMode mode) { return motor.getPIDFCoefficients(mode); }

    @Override public int getTargetPositionTolerance() { return motor.getTargetPositionTolerance(); }
    @Override public void setTargetPositionTolerance(int tolerance) { motor.setTargetPositionTolerance(tolerance); }

    @Override public double getCurrent(CurrentUnit unit) { return motor.getCurrent(unit); }
    @Override public double getCurrentAlert(CurrentUnit unit) { return motor.getCurrentAlert(unit); }
    @Override public void setCurrentAlert(double current, CurrentUnit unit) { motor.setCurrentAlert(current, unit); }
    @Override public boolean isOverCurrent() { return motor.isOverCurrent(); }

    @Override public RunMode getMode() { return motor.getMode(); }
    @Override public void setMode(RunMode mode) { motor.setMode(mode); }

    @Override public MotorConfigurationType getMotorType() { return motor.getMotorType(); }
    @Override public void setMotorType(MotorConfigurationType motorType) { motor.setMotorType(motorType); }

    @Override public DcMotorController getController() { return motor.getController(); }
    @Override public int getPortNumber() { return motor.getPortNumber(); }

    @Override public ZeroPowerBehavior getZeroPowerBehavior() { return motor.getZeroPowerBehavior(); }
    @Override public void setZeroPowerBehavior(ZeroPowerBehavior behavior) { motor.setZeroPowerBehavior(behavior); }

    @Override public void setPowerFloat() { motor.setPowerFloat(); }
    @Override public boolean getPowerFloat() { return motor.getPowerFloat(); }

    @Override public boolean isBusy() { return motor.isBusy(); }
    @Override public int getTargetPosition() { return motor.getTargetPosition(); }
    @Override public void setTargetPosition(int position) { motor.setTargetPosition(position); }
    @Override public int getCurrentPosition() { return motor.getCurrentPosition(); }

    @Override public Direction getDirection() { return motor.getDirection(); }
    @Override public void setDirection(Direction direction) { motor.setDirection(direction); }
    @Override public double getPower() { return motor.getPower(); }

    @Override public Manufacturer getManufacturer() { return motor.getManufacturer(); }
    @Override public String getDeviceName() { return motor.getDeviceName(); }
    @Override public String getConnectionInfo() { return motor.getConnectionInfo(); }
    @Override public int getVersion() { return motor.getVersion(); }
    @Override public void resetDeviceConfigurationForOpMode() { motor.resetDeviceConfigurationForOpMode(); }
    @Override public void close() { motor.close(); }
}
