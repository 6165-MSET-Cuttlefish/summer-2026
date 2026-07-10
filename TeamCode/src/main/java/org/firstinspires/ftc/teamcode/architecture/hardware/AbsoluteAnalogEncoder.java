package org.firstinspires.ftc.teamcode.architecture.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import java.util.Objects;

/**
 * Analog absolute encoder. Two reads:
 * <ul>
 *   <li>{@link #getAbsoluteAngle()} — single-turn, 0–360°, post-offset/inversion.</li>
 *   <li>{@link #getRelativePosition()} — multi-turn accumulated rotation × {@code gearRatio};
 *       the first call snapshots the current shaft as zero.</li>
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

    // Reject a per-sample jump larger than this as sensor noise (see getRelativePosition).
    // Disabled by default (infinite) so behavior is unchanged unless a caller opts in.
    private double maxDeltaDegrees = Double.POSITIVE_INFINITY;

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

    /** Multi-turn rotation in degrees × gearRatio, zeroed at first call. */
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

        if (Math.abs(delta) > maxDeltaDegrees) {
            // Implausible jump for one sample interval — treat as a glitch and skip it. Leave
            // lastRawAngle at the last good value so a single bad sample can't corrupt position.
            delta = 0.0;
            return position * gearRatio;
        }

        position += delta;
        lastRawAngle = rawAngle;

        return position * gearRatio;
    }

    /**
     * Reject any single-sample shaft delta larger than {@code degrees} as sensor noise. Set this
     * above the maximum real rotation between two {@link #getRelativePosition()} calls (e.g. shaft
     * RPM / loop rate) — too low a value freezes accumulation during fast motion. Default: disabled.
     */
    public AbsoluteAnalogEncoder withMaxDelta(double degrees) {
        this.maxDeltaDegrees = degrees;
        return this;
    }

    /** Reset the zero reference to the current shaft angle. */
    public void zero() {
        position = 0.0;
        delta = 0.0;
        initialized = false;
    }

    /** Single-turn shaft angle in [0, 360°), post-offset/inversion. */
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
        double maxVoltage = analogInput.getMaxVoltage();
        if (maxVoltage == 0.0) return 0.0;
        return (analogInput.getVoltage() / maxVoltage) * 360.0;
    }

    /** Last shaft delta in degrees from {@link #getRelativePosition()}. */
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
