package org.firstinspires.ftc.teamcode.architecture.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Pedro Pathing tuning + follower factory for the <strong>DECODE robot</strong>, ported from the
 * DECODE repo's {@code architecture/auto/Constants.java}. Hardware names + offsets live in
 * {@link DecodeHardwareConfig}. Selected via {@link PedroSetup#activeRobot}; DECODE OpModes pin it
 * in {@code DecodeOpMode.createRobot()} before the follower is built.
 *
 * <p>Public statics are non-final so they appear as live-tunable on FTC Dashboard. These are the
 * DECODE-season tuned values (mass and x/y velocity were left at Pedro defaults on that robot);
 * re-tune against the real robot before trusting auto motion.
 */
@Config
public final class DecodePedroSetup {
    private DecodePedroSetup() {}

    public static FollowerConstants followerConstants =
            new FollowerConstants()
                    .headingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.05, 0.03))
                    .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(
                            0.14, 0.1973407216665132, 0.001005159209640557))
                    .centripetalScaling(0.0);

    // DECODE ran voltage compensation at 11.5V nominal; xVelocity/yVelocity were left at Pedro
    // defaults (not measured on that robot).
    public static MecanumConstants driveConstants =
            new MecanumConstants()
                    .maxPower(1.0)
                    .leftFrontMotorName(DecodeHardwareConfig.LEFT_FRONT_MOTOR)
                    .leftRearMotorName(DecodeHardwareConfig.LEFT_REAR_MOTOR)
                    .rightFrontMotorName(DecodeHardwareConfig.RIGHT_FRONT_MOTOR)
                    .rightRearMotorName(DecodeHardwareConfig.RIGHT_REAR_MOTOR)
                    .leftFrontMotorDirection(DecodeHardwareConfig.LEFT_FRONT_DIR)
                    .leftRearMotorDirection(DecodeHardwareConfig.LEFT_REAR_DIR)
                    .rightFrontMotorDirection(DecodeHardwareConfig.RIGHT_FRONT_DIR)
                    .rightRearMotorDirection(DecodeHardwareConfig.RIGHT_REAR_DIR)
                    .motorCachingThreshold(0.01)
                    .useVoltageCompensation(true)
                    .nominalVoltage(11.5);

    public static PinpointConstants localizerConstants =
            new PinpointConstants()
                    .forwardPodY(DecodeHardwareConfig.PINPOINT_FORWARD_POD_Y)
                    .strafePodX(DecodeHardwareConfig.PINPOINT_STRAFE_POD_X)
                    .distanceUnit(DecodeHardwareConfig.PINPOINT_DISTANCE_UNIT)
                    .hardwareMapName(DecodeHardwareConfig.PINPOINT_NAME)
                    .encoderResolution(DecodeHardwareConfig.PINPOINT_POD_TYPE)
                    .forwardEncoderDirection(DecodeHardwareConfig.PINPOINT_FORWARD_DIR)
                    .strafeEncoderDirection(DecodeHardwareConfig.PINPOINT_STRAFE_DIR);

    public static PathConstraints pathConstraints = new PathConstraints(0.95, 0, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
