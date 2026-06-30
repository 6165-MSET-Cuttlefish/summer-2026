package org.firstinspires.ftc.teamcode.architecture.hardware;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

/**
 * Driver for the Brushland Labs Laser Rangefinder over I2C.
 * Configure as "Rev Color Sensor V3" named "Laser" in the robot config.
 * Docs: https://docs.brushlandlabs.com/sensors/laser-rangefinder/getting-started
 */
public class LaserRangefinder {

    public final LynxI2cDeviceSynch i2c;

    public LaserRangefinder(RevColorSensorV3 device) {
        this.i2c = (LynxI2cDeviceSynch) device.getDeviceClient();
        this.i2c.enableWriteCoalescing(true);
    }

    // ── Pin configuration ──────────────────────────────────────────────────

    public void setPin0Digital(int thresh_lo, int thresh_hi) {
        setPin(R_PIN0_MODE, M_DIG, thresh_lo, thresh_hi);
    }

    public void setPin1Digital(int thresh_lo, int thresh_hi) {
        setPin(R_PIN1_MODE, M_DIG, thresh_lo, thresh_hi);
    }

    public void setPin0Analog(int bound_lo, int bound_hi) {
        setPin(R_PIN0_MODE, M_ANA, bound_lo, bound_hi);
    }

    public void setPin1Analog(int bound_lo, int bound_hi) {
        setPin(R_PIN1_MODE, M_ANA, bound_lo, bound_hi);
    }

    private void setPin(byte reg, byte mode, int lo, int hi) {
        byte lo0 = (byte) (lo & 0xFF);
        byte lo1 = (byte) ((lo & 0xFF00) >> 8);
        byte hi0 = (byte) (hi & 0xFF);
        byte hi1 = (byte) ((hi & 0xFF00) >> 8);
        write(reg, new byte[]{mode, lo0, lo1, hi0, hi1});
    }

    public int getPin0Mode() { return i2c.read8(R_PIN0_MODE); }
    public int getPin1Mode() { return i2c.read8(R_PIN1_MODE); }

    // ── Distance mode ──────────────────────────────────────────────────────

    public void setDistanceMode(DistanceMode mode) {
        write(R_DISTMODE, new byte[]{(byte) (mode.ordinal() + 1)});
    }

    public DistanceMode getDistanceMode() {
        byte v = i2c.read8(R_DISTMODE);
        switch (v) {
            case 1: return DistanceMode.SHORT;
            case 2: return DistanceMode.MEDIUM;
            case 3: return DistanceMode.LONG;
            default: throw new RuntimeException("Could not get distance mode: " + v);
        }
    }

    // ── Timing ─────────────────────────────────────────────────────────────

    /**
     * @param budget  timing budget in ms (5–1000)
     * @param period  measurement period in ms; 0 = start next range immediately
     */
    public void setTiming(int budget, int period) {
        if (budget < 5 || budget > 1000) throw new RuntimeException("Invalid timing budget: " + budget);
        if (period != 0 && period < budget + 3) throw new RuntimeException("Period must be >= budget + 4ms, or 0.");
        write(R_TIMING, new byte[]{
                (byte) (budget & 0xFF), (byte) ((budget & 0xFF00) >> 8),
                (byte) (period & 0xFF), (byte) ((period & 0xFF00) >> 8)
        });
    }

    public int[] getTiming() {
        ByteBuffer buf = ByteBuffer.wrap(i2c.read(R_TIMING, 4)).order(ByteOrder.LITTLE_ENDIAN);
        return new int[]{buf.getShort(), buf.getShort()};
    }

    // ── Region of interest ─────────────────────────────────────────────────

    /** ROI must be at least 4×4; coordinates are 0–15 (wire side = +y). */
    public void setROI(int topLeftX, int topLeftY, int botRightX, int botRightY) {
        if (botRightX - topLeftX < 3 || topLeftY - botRightY < 3)
            throw new RuntimeException("ROI too small, must be at least 4×4.");
        for (int v : new int[]{topLeftX, topLeftY, botRightX, botRightY})
            if (v < 0 || v > 15) throw new RuntimeException("Invalid ROI coordinate: " + v);
        write(R_ROI_TLX, new byte[]{(byte) topLeftX, (byte) topLeftY, (byte) botRightX, (byte) botRightY});
    }

    public int[] getROI() {
        byte[] data = i2c.read(R_ROI_TLX, 4);
        return new int[]{data[0], data[1], data[2], data[3]};
    }

    // ── I2C address ────────────────────────────────────────────────────────

    /** Change the I2C address from the default 0x52. Valid range: 0x01–0x7F. */
    public void setI2CAddress(int newAddress) {
        if (newAddress < 1 || newAddress > 127) throw new RuntimeException("Invalid I2C address: " + newAddress);
        write(R_IIC_ADDR, new byte[]{(byte) newAddress});
    }

    // ── Optical center ─────────────────────────────────────────────────────

    /** Factory-calibrated center (x, y) of the 16×16 photodiode grid. */
    public int[] getOpticalCenter() {
        byte[] data = i2c.read(R_OPTCENTERX, 2);
        return new int[]{data[0], data[1]};
    }

    // ── I2C reset ──────────────────────────────────────────────────────────

    /** Reset both pins to I2C mode, clearing any custom address. */
    public void setI2C() {
        write(R_PIN0_MODE, new byte[]{M_I2C, 0, 0, 0, 0});
    }

    // ── Scan mode builders ─────────────────────────────────────────────────

    public ScanSequenceBuilder setAnalogScanMode() { return new ScanSequenceBuilder(M_AN2); }
    public ScanSequenceBuilder setI2CScanMode()    { return new ScanSequenceBuilder(M_II2); }

    public class ScanSequenceBuilder {
        private final byte mode;
        private final List<byte[]> rois = new ArrayList<>();

        public ScanSequenceBuilder(byte mode) { this.mode = mode; }

        public ScanSequenceBuilder addScanROI(int topLeftX, int topLeftY, int botRightX, int botRightY) {
            rois.add(new byte[]{(byte) topLeftX, (byte) topLeftY, (byte) botRightX, (byte) botRightY});
            return this;
        }

        public void setScanROIs() {
            for (byte[] roi : rois) i2c.write(R_PIN0_MODE, new byte[]{mode, roi[0], roi[1], roi[2], roi[3]});
            write(R_PIN0_MODE, new byte[]{mode, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF, (byte) 0xFF});
        }
    }

    // ── Distance reads ─────────────────────────────────────────────────────

    private int status = 0;

    /** 0 = good read, 1–2 = okay, higher = bad. */
    public int getStatus() { return status; }

    /** Read distance over I2C (~4 ms per call). Also updates {@link #getStatus()}. */
    public double getDistance(DistanceUnit unit) {
        byte[] data = i2c.read(R_PS_DATA_0, 2);
        status = (data[1] & 0xE0) >> 5;
        data[1] &= 0x1F;
        return unit.fromUnit(DistanceUnit.MM,
                ByteBuffer.wrap(data, 0, 2).order(ByteOrder.LITTLE_ENDIAN).getShort());
    }

    private int roiNum = 0;

    /** In I2C scan mode, the ROI index of the last {@link #getScanDistance} read. */
    public int getROINum() { return roiNum; }

    /** Read distance in I2C scan mode — returns the ROI index alongside the reading. */
    public double getScanDistance(DistanceUnit unit) {
        byte[] data = i2c.read(R_PS_DATA_0, 3);
        roiNum = data[2];
        status = data[1] & 0xE0;
        data[1] &= 0x1F;
        return unit.fromUnit(DistanceUnit.MM,
                ByteBuffer.wrap(data, 0, 2).order(ByteOrder.LITTLE_ENDIAN).getShort());
    }

    // ── Internal helpers ───────────────────────────────────────────────────

    private void write(int reg, byte[] bytes) {
        i2c.write(reg, bytes);
        try { Thread.sleep(10); } catch (InterruptedException e) { throw new RuntimeException(e); }
    }

    // ── Enums & constants ──────────────────────────────────────────────────

    public enum DistanceMode {
        SHORT,   // max 1.3 m, best ambient light immunity
        MEDIUM,  // max 3 m
        LONG     // max 4 m, most susceptible to ambient light
    }

    private static final byte M_I2C = 0;
    private static final byte M_ANA = 1;
    private static final byte M_DIG = 2;
    private static final byte M_AN2 = 3;
    private static final byte M_II2 = 4;

    private static final byte R_PS_DATA_0 = 0x08;
    private static final byte R_PIN0_MODE = 0x28;
    private static final byte R_PIN1_MODE = 0x2D;
    private static final byte R_ROI_TLX   = 0x32;
    private static final byte R_DISTMODE  = 0x36;
    private static final byte R_TIMING    = 0x37;
    private static final byte R_IIC_ADDR  = 0x3B;
    private static final byte R_OPTCENTERX = 0x3C;
}
