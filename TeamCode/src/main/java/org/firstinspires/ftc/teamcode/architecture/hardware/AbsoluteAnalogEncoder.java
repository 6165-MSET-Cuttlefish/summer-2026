package org.firstinspires.ftc.teamcode.architecture.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import java.util.Objects;

/**
 * Wraps an analog absolute encoder. Two different reads:
 * <ul>
 *   <li>{@link #getAbsoluteAngle()} — single-turn, 0–360°, post-offset/inversion. Always
 *       reflects the current physical shaft angle.</li>
 *   <li>{@link #getRelativePosition()} — accumulated multi-turn rotation since the first call,
 *       multiplied by {@code gearRatio}. The first call snapshots the current shaft as the
 *       zero reference.</li>
 * </ul>
 */
public class AbsoluteAnalogEncoder implements HardwareDevice {
    protected final AnalogInput analogInput;
    protected final double offset;
    protected final double gearRatio;
    protected final boolean inverted;

    private double lastRawAngle;
    private boolean initialized;
    private double position;
    private double delta;

    public AbsoluteAnalogEncoder(AnalogInput analogInput) {
        this(analogInput, 0.0, 1.0, false);
    }

    public AbsoluteAnalogEncoder(
            AnalogInput analogInput, double offset, double gearRatio, boolean inverted) {
        this.analogInput = Objects.requireNonNull(analogInput, "analogInput");
        this.offset = offset;
        this.gearRatio = gearRatio;
        this.inverted = inverted;
    }

    /** Multi-turn accumulated rotation in degrees × gearRatio, zeroed at first call. */
    public double getRelativePosition() {
        double rawAngle = getAbsoluteAngle();

        if (!initialized) {
            initialized = true;
            lastRawAngle = rawAngle;
            delta = 0.0;
            return position * gearRatio;
        }

        delta = rawAngle - lastRawAngle;
        if (delta > 180.0) delta -= 360.0;
        if (delta < -180.0) delta += 360.0;

        position += delta;
        lastRawAngle = rawAngle;

        return position * gearRatio;
    }

    /** Re-snapshot the current shaft angle as the new zero reference. */
    public void zero() {
        position = 0.0;
        initialized = false;
    }

    /** Single-turn shaft angle, 0–360°, after applying offset and inversion. */
    public double getAbsoluteAngle() {
        double encoderPosition = getVoltageAsAngle();

        if (inverted) {
            encoderPosition = 360.0 - encoderPosition;
        }

        encoderPosition = (encoderPosition - offset) % 360.0;
        if (encoderPosition < 0) {
            encoderPosition += 360.0;
        }

        return encoderPosition;
    }

    protected double getVoltageAsAngle() {
        double voltage = analogInput.getVoltage();
        double maxVoltage = analogInput.getMaxVoltage();
        return (voltage / maxVoltage) * 360.0;
    }

    /** Last shaft delta (degrees) seen by {@link #getRelativePosition()}. */
    public double getDelta() {
        return delta;
    }

    public int getRevolutionCount() {
        return (int) Math.round(position / 360.0);
    }

    public double getVoltage() {
        return analogInput.getVoltage();
    }

    public double getNormalizedValue() {
        double maxVoltage = analogInput.getMaxVoltage();
        if (maxVoltage == 0.0) return 0.0;
        return analogInput.getVoltage() / maxVoltage;
    }

    @Override
    public Manufacturer getManufacturer() {
        return analogInput.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return "Absolute Analog Encoder";
    }

    @Override
    public String getConnectionInfo() {
        return analogInput.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {}

    @Override
    public void close() {
        analogInput.close();
    }
}
