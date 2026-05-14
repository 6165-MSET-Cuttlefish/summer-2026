package org.firstinspires.ftc.teamcode.architecture.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Enhanced CRServo wrapper with power caching, bounds, and voltage compensation. Configure by
 * chaining: {@code new EnhancedCRServo(servo).withCachingTolerance(0.02).withVoltageCompensation(13.5)}.
 */
public class EnhancedCRServo implements CRServo, PwmControl {
    private final CRServoImplEx crServo;
    private final WriteCache cache = new WriteCache();

    public EnhancedCRServo(CRServoImplEx crServo) {
        this.crServo = crServo;
        cache.referenceVoltage = 13.5;
    }

    public EnhancedCRServo(HardwareMap hardwareMap, String name) {
        this.crServo = hardwareMap.get(CRServoImplEx.class, name);
        cache.referenceVoltage = 13.5;
    }

    // ─── caching / bounds / voltage configuration ────────────────────────────

    public EnhancedCRServo withCachingTolerance(double tolerance) {
        cache.tolerance = tolerance;
        return this;
    }

    public EnhancedCRServo withPowerBounds(double min, double max) {
        cache.min = min;
        cache.max = max;
        return this;
    }

    public EnhancedCRServo withVoltageCompensation(double referenceVoltage) {
        cache.voltageCompensationEnabled = true;
        cache.referenceVoltage = referenceVoltage;
        return this;
    }

    /** Publish a fresh battery voltage reading; called by EnhancedOpMode each loop. */
    public static void updateVoltage(double voltage) {
        HardwareVoltage.update(voltage);
    }

    // ─── setPower with caching ───────────────────────────────────────────────

    @Override
    public void setPower(double power) {
        double corrected = cache.clamp(cache.applyVoltageScaling(power));
        if (cache.shouldWrite(corrected)) {
            cache.store(corrected);
            crServo.setPower(corrected);
        }
    }

    /** Set power directly without voltage compensation. Useful for testing. */
    public void setPowerRaw(double power) {
        double corrected = cache.clamp(power);
        cache.store(corrected);
        crServo.setPower(corrected);
    }

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

    public CRServoImplEx getUnderlying() { return crServo; }
    public double getCachedPower() { return cache.cached; }

    // ─── CRServo / PwmControl interface (forwarded) ──────────────────────────

    @Override public ServoController getController() { return crServo.getController(); }
    @Override public int getPortNumber() { return crServo.getPortNumber(); }

    @Override public Direction getDirection() { return crServo.getDirection(); }
    @Override public void setDirection(Direction direction) { crServo.setDirection(direction); }

    @Override public double getPower() { return crServo.getPower(); }

    @Override public Manufacturer getManufacturer() { return crServo.getManufacturer(); }
    @Override public String getDeviceName() { return crServo.getDeviceName(); }
    @Override public String getConnectionInfo() { return crServo.getConnectionInfo(); }
    @Override public int getVersion() { return crServo.getVersion(); }
    @Override public void resetDeviceConfigurationForOpMode() { crServo.resetDeviceConfigurationForOpMode(); }
    @Override public void close() { crServo.close(); }

    @Override public void setPwmRange(PwmRange range) { crServo.setPwmRange(range); }
    @Override public PwmRange getPwmRange() { return crServo.getPwmRange(); }
    @Override public void setPwmEnable() { crServo.setPwmEnable(); }
    @Override public void setPwmDisable() { crServo.setPwmDisable(); }
    @Override public boolean isPwmEnabled() { return crServo.isPwmEnabled(); }
}
