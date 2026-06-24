package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.architecture.hardware.LaserRangefinder;

import java.util.Arrays;

/**
 * One-shot configuration opmode for the Brushland Labs Laser Rangefinder.
 *
 * Hardware config: add a "Rev Color Sensor V3" named "Laser" to an I2C port.
 *
 * Usage:
 *  1. Run this opmode to inspect the current sensor config (printed before Start).
 *  2. Uncomment / edit the configuration block below, deploy, and run once.
 *  3. Unplug and replug the sensor wire — changes take effect on next power-on.
 *
 * To reset to I2C after switching to digital/analog output:
 *  1. Init this opmode with the sensor unplugged (you'll see "failed to communicate").
 *  2. Stop the opmode — the warning stays on screen.
 *  3. Plug in the sensor — warning clears, LED turns magenta.
 *
 * Docs: https://docs.brushlandlabs.com/sensors/laser-rangefinder/getting-started
 */
@Autonomous(name = "Configure Laser Rangefinder", group = "Test")
public class ConfigureLaserRangefinder extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        LaserRangefinder lrf = new LaserRangefinder(
                hardwareMap.get(RevColorSensorV3.class, "Laser"));

        telemetry.addLine("=== Current sensor config ===");
        telemetry.addData("Pin0 mode",      lrf.getPin0Mode());
        telemetry.addData("Pin1 mode",      lrf.getPin1Mode());
        telemetry.addData("Distance mode",  lrf.getDistanceMode().name());
        telemetry.addData("Timing [budget, period]", Arrays.toString(lrf.getTiming()));
        telemetry.addData("ROI",            Arrays.toString(lrf.getROI()));
        telemetry.addData("Optical center", Arrays.toString(lrf.getOpticalCenter()));
        telemetry.addLine();
        telemetry.addLine("Press Start to write configuration.");
        telemetry.update();

        waitForStart();

        // ── Write persistent configuration ─────────────────────────────────
        // Changes take effect after unplugging and replugging the sensor.
        // Edit these values as needed, then run the opmode once.

        lrf.setDistanceMode(LaserRangefinder.DistanceMode.LONG);
        lrf.setTiming(33, 0);   // 30 Hz, less noisy; period=0 → start next range immediately
        lrf.setROI(0, 15, 15, 0); // full 16×16 FOV (27°)

        telemetry.addLine("Configuration written. Unplug and replug the sensor.");
        telemetry.update();
        sleep(2000);
    }
}
