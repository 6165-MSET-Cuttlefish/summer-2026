package org.firstinspires.ftc.teamcode.architecture.auto;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public final class BettaHardwareConfig {
    private BettaHardwareConfig() {}

    public static final String LEFT_FRONT_MOTOR  = "fl";
    public static final String LEFT_REAR_MOTOR   = "bl";
    public static final String RIGHT_FRONT_MOTOR = "fr";
    public static final String RIGHT_REAR_MOTOR  = "br";

    public static final DcMotorSimple.Direction LEFT_FRONT_DIR  = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction LEFT_REAR_DIR   = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RIGHT_FRONT_DIR = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction RIGHT_REAR_DIR  = DcMotorSimple.Direction.FORWARD;

    public static final String PINPOINT_NAME = "pinpoint";

    /** Forward (parallel) pod offset from robot center, inches. */
    public static final double PINPOINT_FORWARD_POD_Y = -5.5;
    /** Strafe (perpendicular) pod offset from robot center, inches. */
    public static final double PINPOINT_STRAFE_POD_X = 1.0;

    public static final DistanceUnit PINPOINT_DISTANCE_UNIT = DistanceUnit.INCH;

    public static final GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_FORWARD_DIR =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_STRAFE_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
}
