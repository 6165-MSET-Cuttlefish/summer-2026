package org.firstinspires.ftc.teamcode.architecture.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareDevice;

public class AbsoluteAnalogEncoder implements HardwareDevice {
    protected final AnalogInput analogInput;
    protected final double offset;
    protected final double gearRatio;
    protected final boolean inverted;

    private double lastRawAngle;
    private boolean initialized;
    private double position;
    public double delta;

    public AbsoluteAnalogEncoder(AnalogInput analogInput) {
        this(analogInput, 0.0, 1.0, false);
    }

    public AbsoluteAnalogEncoder(
            AnalogInput analogInput, double offset, double gearRatio, boolean inverted) {
        this.analogInput = analogInput;
        this.offset = offset;
        this.gearRatio = gearRatio;
        this.inverted = inverted;
        this.lastRawAngle = 0.0;
        this.initialized = false;
        this.position = 0.0;
        this.delta = 0.0;
    }

    /**
     * Returns the accumulated position in degrees (times gear ratio), starting from 0.
     * Tracks multi-turn rotation by accumulating wrap-corrected deltas between calls.
     */
    public double getPosition() {
        double rawAngle = getRawEncoderPosition();

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

    /**
     * Resets the accumulated position to 0. The next getPosition() call will
     * re-snapshot the current physical angle as the new reference and report 0.
     */
    public void zero() {
        position = 0.0;
        initialized = false;
    }

    public double getRawEncoderPosition() {
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
        if (analogInput == null) {
            return Double.NaN;
        }
        double maxVoltage = analogInput.getMaxVoltage();
        if (maxVoltage == 0.0) {
            return 0.0;
        }
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
