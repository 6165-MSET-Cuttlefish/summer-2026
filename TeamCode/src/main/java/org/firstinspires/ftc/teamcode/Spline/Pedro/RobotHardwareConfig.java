package org.firstinspires.ftc.teamcode.Spline.Pedro;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Hardware-level config: names, ports, physical offsets. <strong>Edit when wiring a new
 * robot.</strong> Tuning numbers live in {@link PedroSetup}. Fields are final — {@link PedroSetup}
 * reads them once at class load.
 */
public final class RobotHardwareConfig {
    private RobotHardwareConfig() {}

    public static final String LEFT_FRONT_MOTOR  = "fl";
    public static final String LEFT_REAR_MOTOR   = "bl";
    public static final String RIGHT_FRONT_MOTOR = "fr";
    public static final String RIGHT_REAR_MOTOR  = "br";

    public static final DcMotorSimple.Direction LEFT_FRONT_DIR  = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction LEFT_REAR_DIR   = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RIGHT_FRONT_DIR = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction RIGHT_REAR_DIR  = DcMotorSimple.Direction.FORWARD;

    public static final String PINPOINT_NAME = "pinpoint";

    /** Y-offset of the forward (parallel) pod from robot center, inches. */
    public static final double PINPOINT_FORWARD_POD_Y = -6.5;
    /** X-offset of the strafe (perpendicular) pod from robot center, inches. */
    public static final double PINPOINT_STRAFE_POD_X = 1.25;

    public static final DistanceUnit PINPOINT_DISTANCE_UNIT = DistanceUnit.INCH;

    public static final GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_FORWARD_DIR =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_STRAFE_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
}
