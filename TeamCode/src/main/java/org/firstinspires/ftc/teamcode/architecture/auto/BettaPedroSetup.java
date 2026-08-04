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

/** Betta bot Pedro tuning; public statics are non-final so FTC Dashboard sees them as live-tunable. */
@Config
public final class BettaPedroSetup {
    private BettaPedroSetup() {}

    // predictiveBrakingCoefficients(...) flips usePredictiveBraking=true, so translational/drive PIDF
    // and zero-power-accel constants are never consulted.
    public static FollowerConstants followerConstants =
            new FollowerConstants()
                    .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0, 0.01))
                    .mass(10.65)
                    .predictiveBrakingCoefficients(
                            new PredictiveBrakingCoefficients(0.15, 0.1, 0.001))
                    .centripetalScaling(0.0005);

    public static MecanumConstants driveConstants =
            new MecanumConstants()
                    .maxPower(1.0)
                    .xVelocity(81.34056)
                    .yVelocity(65.43028)
                    .leftFrontMotorName(BettaHardwareConfig.LEFT_FRONT_MOTOR)
                    .leftRearMotorName(BettaHardwareConfig.LEFT_REAR_MOTOR)
                    .rightFrontMotorName(BettaHardwareConfig.RIGHT_FRONT_MOTOR)
                    .rightRearMotorName(BettaHardwareConfig.RIGHT_REAR_MOTOR)
                    .leftFrontMotorDirection(BettaHardwareConfig.LEFT_FRONT_DIR)
                    .leftRearMotorDirection(BettaHardwareConfig.LEFT_REAR_DIR)
                    .rightFrontMotorDirection(BettaHardwareConfig.RIGHT_FRONT_DIR)
                    .rightRearMotorDirection(BettaHardwareConfig.RIGHT_REAR_DIR)
                    .motorCachingThreshold(0.01)
                    .useVoltageCompensation(false)
                    .nominalVoltage(12.0);

    public static PinpointConstants localizerConstants =
            new PinpointConstants()
                    .forwardPodY(BettaHardwareConfig.PINPOINT_FORWARD_POD_Y)
                    .strafePodX(BettaHardwareConfig.PINPOINT_STRAFE_POD_X)
                    .distanceUnit(BettaHardwareConfig.PINPOINT_DISTANCE_UNIT)
                    .hardwareMapName(BettaHardwareConfig.PINPOINT_NAME)
                    .encoderResolution(BettaHardwareConfig.PINPOINT_POD_TYPE)
                    .forwardEncoderDirection(BettaHardwareConfig.PINPOINT_FORWARD_DIR)
                    .strafeEncoderDirection(BettaHardwareConfig.PINPOINT_STRAFE_DIR);

    // .copy() so downstream FollowerBuilder.pathConstraints(...) can't mutate the shared default.
    public static PathConstraints pathConstraints = PathConstraints.defaultConstraints.copy();

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
