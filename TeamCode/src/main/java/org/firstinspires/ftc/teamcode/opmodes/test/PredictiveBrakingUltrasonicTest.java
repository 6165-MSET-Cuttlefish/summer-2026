package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.maxbotix.MaxSonarI2CXL;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Drives straight forward at full power and brakes on the MaxBotix MB1242 ultrasonic, settling at
 * {@link PredictiveBraking.Tuning#stopDistanceCm}. No gamepad input — it starts driving on Play.
 *
 * Hardware config:
 *   - Motors "fl", "bl", "fr", "br" on Control Hub motor ports 0-3.
 *   - I2C device of type "MaxSonar I2CXL" named "sonar" (put it on bus 1, not 0).
 *
 * Run with {@link Tuning#wheelTest} true first (the default). All four wheels spin at 20% and the
 * braking logic is skipped, so you can confirm every wheel drives the robot forward. Flip the
 * matching reverse{Corner} flag on the dashboard for any wheel spinning backwards — the flags are
 * applied live, no restart needed. Once all four agree, set wheelTest false and run for real.
 *
 * The MB1242 floors at 20 cm — anything closer still reports 20 — which happens to be exactly the
 * stop distance here, so the floor works in our favor. Don't lower stopDistanceCm below 20 for this
 * sensor; the reading simply won't follow you down.
 */
@TeleOp(name = "Predictive Braking (Ultrasonic)", group = "Test")
public class PredictiveBrakingUltrasonicTest extends OpMode {

    @Config
    public static class Tuning {
        /** True: skip braking, spin every wheel at 20% so you can spot a reversed one. */
        public static boolean wheelTest = true;
        public static double wheelTestPower = 0.2;
        /** Forward power the braking logic gets to clamp down from. */
        public static double driveSpeed = 1.0;

        // Starting guess copied from BettaHardwareConfig (left side reversed). The wheel test is
        // there because this is a guess — correct it there, then port it back to the real config.
        public static boolean reverseFrontLeft  = true;
        public static boolean reverseBackLeft   = true;
        public static boolean reverseFrontRight = false;
        public static boolean reverseBackRight  = false;

        /** Ping-to-read delay. 100 ms is the datasheet's full-range figure; lower updates faster. */
        public static int propagationDelayMs = 100;
    }

    /** SDK-side address. Change only if the sensor's EEPROM was rewritten by ConfigureUltrasonicAddress. */
    private static final int I2C_ADDR_8BIT = 0xE0;

    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private MaxSonarI2CXL sonar;
    private PredictiveBraking braking;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        frontLeft  = initMotor("fl");
        backLeft   = initMotor("bl");
        frontRight = initMotor("fr");
        backRight  = initMotor("br");

        sonar = hardwareMap.get(MaxSonarI2CXL.class, "sonar");
        sonar.setI2cAddress(I2cAddr.create8bit(I2C_ADDR_8BIT));
        // Before the first ping the sensor answers a read with power-up info bytes, not a range.
        // Burn one blocking ping/read here so the braking logic never sees them.
        sonar.getDistanceSync(Tuning.propagationDelayMs, DistanceUnit.CM);

        braking = new PredictiveBraking(
                () -> sonar.getDistanceAsync(Tuning.propagationDelayMs, DistanceUnit.CM));
        dashboard = FtcDashboard.getInstance();

        telemetry.addLine(Tuning.wheelTest
                ? "WHEEL TEST mode — wheels will spin at 20% on Play."
                : "Braking mode — robot will drive forward at speed on Play.");
        telemetry.update();
    }

    private DcMotorEx initMotor(String name) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    @Override
    public void loop() {
        if (Tuning.wheelTest) {
            drive(Tuning.wheelTestPower);
            telemetry.addLine("WHEEL TEST — every wheel should be driving the robot FORWARD.");
            telemetry.addData("power", "%.2f", Tuning.wheelTestPower);
            telemetry.update();
            return;
        }

        braking.read();
        double power = braking.clampApproachPower(Tuning.driveSpeed);
        drive(power);

        double cm = braking.getDistanceCm();

        telemetry.addData("distance cm", "%.0f", cm);
        telemetry.addData("distance in", "%.2f", DistanceUnit.INCH.fromCm(cm));
        telemetry.addData("power",       "%.3f", power);
        telemetry.addData("ceiling",     "%.3f", braking.getPowerCeiling());
        telemetry.addData("braking",     braking.isBraking());
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("distance_cm", cm);
        packet.put("power",       power);
        packet.put("ceiling",     braking.getPowerCeiling());
        packet.put("stop_cm",     PredictiveBraking.Tuning.stopDistanceCm);
        dashboard.sendTelemetryPacket(packet);
    }

    /** Same power to all four wheels — straight forward, no strafe, no turn. */
    private void drive(double power) {
        frontLeft.setPower(power  * (Tuning.reverseFrontLeft  ? -1 : 1));
        backLeft.setPower(power   * (Tuning.reverseBackLeft   ? -1 : 1));
        frontRight.setPower(power * (Tuning.reverseFrontRight ? -1 : 1));
        backRight.setPower(power  * (Tuning.reverseBackRight  ? -1 : 1));
    }

    @Override
    public void stop() {
        drive(0);
    }
}
