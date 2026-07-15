package org.firstinspires.ftc.teamcode.architecture.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Single-device bench-test templates. Subclass, supply hardwareName(), add {@code @TeleOp}.
 *
 * <pre>{@code
 * @TeleOp(name = "Test leftClaw")
 * public class LeftClawTest extends HardwareTest.ServoPosition {
 *     @Override protected String hardwareName() { return "leftClaw"; }
 * }
 * }</pre>
 */
public final class HardwareTest {
    private HardwareTest() {}

    /** D-pad up/down nudges; A/B/Y snap to 0/0.5/1. */
    public abstract static class ServoPosition extends OpMode {
        protected abstract String hardwareName();
        protected double initialPosition() { return 0.5; }
        protected double stepSize() { return 0.005; }

        private Servo servo;
        private double position;

        @Override
        public final void init() {
            servo = hardwareMap.get(Servo.class, hardwareName());
            position = initialPosition();
            servo.setPosition(position);
        }

        @Override
        public final void loop() {
            if (gamepad1.dpad_up)   position = Math.min(1.0, position + stepSize());
            if (gamepad1.dpad_down) position = Math.max(0.0, position - stepSize());
            if (gamepad1.a) position = 0.0;
            if (gamepad1.b) position = 0.5;
            if (gamepad1.y) position = 1.0;
            servo.setPosition(position);

            telemetry.addData("servo",    hardwareName());
            telemetry.addData("position", "%.3f", position);
            telemetry.addLine("D-pad up/down: step  |  A: 0  |  B: 0.5  |  Y: 1");
            telemetry.update();
        }
    }

    /** Left stick Y drives power; logs encoder position. */
    public abstract static class MotorPower extends OpMode {
        protected abstract String hardwareName();
        protected double maxPower() { return 1.0; }
        protected DcMotor.ZeroPowerBehavior zeroPowerBehavior() { return DcMotor.ZeroPowerBehavior.BRAKE; }

        private DcMotor motor;

        @Override
        public final void init() {
            motor = hardwareMap.get(DcMotor.class, hardwareName());
            motor.setZeroPowerBehavior(zeroPowerBehavior());
        }

        @Override
        public final void loop() {
            // Stick up = positive power.
            double power = -gamepad1.left_stick_y * maxPower();
            motor.setPower(power);

            telemetry.addData("motor",    hardwareName());
            telemetry.addData("power",    "%.2f", power);
            telemetry.addData("position", motor.getCurrentPosition());
            telemetry.addLine("Left stick Y: power");
            telemetry.update();
        }

        @Override
        public final void stop() {
            // The SDK doesn't zero motor power on STOP; do it so the mechanism can't keep driving.
            if (motor != null) motor.setPower(0.0);
        }
    }

    /** Left stick Y drives power. */
    public abstract static class CRServoPower extends OpMode {
        protected abstract String hardwareName();
        protected double maxPower() { return 1.0; }

        private CRServo servo;

        @Override
        public final void init() {
            servo = hardwareMap.get(CRServo.class, hardwareName());
        }

        @Override
        public final void loop() {
            double power = -gamepad1.left_stick_y * maxPower();
            servo.setPower(power);

            telemetry.addData("CR servo", hardwareName());
            telemetry.addData("power",    "%.2f", power);
            telemetry.addLine("Left stick Y: power");
            telemetry.update();
        }

        @Override
        public final void stop() {
            // The SDK doesn't zero CR-servo power on STOP; do it so the mechanism can't keep driving.
            if (servo != null) servo.setPower(0.0);
        }
    }
}
