package org.firstinspires.ftc.teamcode.architecture.auto;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Hardware wiring for the <strong>DECODE robot</strong>: motor names/directions + Pinpoint pod
 * offsets, ported from the DECODE repo's {@code architecture/auto/Constants.java}. Tuning lives in
 * {@link DecodePedroSetup}. {@link PedroSetup#activeRobot} selects between this and
 * {@link BettaHardwareConfig}; DECODE OpModes pin it via {@code DecodeOpMode.createRobot()}.
 *
 * <p>Differs from the betta bot in the strafe-pod encoder direction (DECODE = REVERSED).
 * Values marked "measured 2/17" came from the DECODE robot; re-measure if the odometry moves.
 */
public final class DecodeHardwareConfig {
    private DecodeHardwareConfig() {}

    public static final String LEFT_FRONT_MOTOR  = "fl";
    public static final String LEFT_REAR_MOTOR   = "bl";
    public static final String RIGHT_FRONT_MOTOR = "fr";
    public static final String RIGHT_REAR_MOTOR  = "br";

    public static final DcMotorSimple.Direction LEFT_FRONT_DIR  = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction LEFT_REAR_DIR   = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RIGHT_FRONT_DIR = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction RIGHT_REAR_DIR  = DcMotorSimple.Direction.FORWARD;

    public static final String PINPOINT_NAME = "pinpoint";

    /** Y-offset of the forward (parallel) pod from robot center, inches. Measured 2/17 on DECODE. */
    public static final double PINPOINT_FORWARD_POD_Y = -5.5;
    /** X-offset of the strafe (perpendicular) pod from robot center, inches. Measured 2/17 on DECODE. */
    public static final double PINPOINT_STRAFE_POD_X = 1.0;

    public static final DistanceUnit PINPOINT_DISTANCE_UNIT = DistanceUnit.INCH;

    public static final GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_FORWARD_DIR =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    // DECODE strafe pod is REVERSED (the betta bot's is FORWARD) — a real per-robot difference.
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_STRAFE_DIR =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
}
