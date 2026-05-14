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
 * Pedro Pathing tuning + the {@link #createFollower(HardwareMap)} factory. <strong>Edit when
 * retuning path-following.</strong> Hardware names + offsets live in {@link RobotHardwareConfig}.
 *
 * <p>Public statics are non-final so they appear as live-tunable on FTC Dashboard.
 */
@Config
public final class PedroSetup {
    private PedroSetup() {}

    // Pedro 2.1.2 defaults. Predictive braking is enabled (predictiveBrakingCoefficients(...)
    // flips usePredictiveBraking=true), so translational/drive PIDF and zero-power-accel
    // constants are NOT consulted — the predictive controller drives both inner loops.
    public static FollowerConstants followerConstants =
            new FollowerConstants()
                    .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0, 0.01))
                    .mass(10.65)
                    .predictiveBrakingCoefficients(
                            new PredictiveBrakingCoefficients(0.15, 0.1, 0.001))
                    .centripetalScaling(0.0005);

    // xVelocity/yVelocity are Pedro defaults; measure forward + strafe full-power free-run
    // speed (in/s) on the real robot for path-following accuracy.
    public static MecanumConstants driveConstants =
            new MecanumConstants()
                    .maxPower(1.0)
                    .xVelocity(81.34056)
                    .yVelocity(65.43028)
                    .leftFrontMotorName(RobotHardwareConfig.LEFT_FRONT_MOTOR)
                    .leftRearMotorName(RobotHardwareConfig.LEFT_REAR_MOTOR)
                    .rightFrontMotorName(RobotHardwareConfig.RIGHT_FRONT_MOTOR)
                    .rightRearMotorName(RobotHardwareConfig.RIGHT_REAR_MOTOR)
                    .leftFrontMotorDirection(RobotHardwareConfig.LEFT_FRONT_DIR)
                    .leftRearMotorDirection(RobotHardwareConfig.LEFT_REAR_DIR)
                    .rightFrontMotorDirection(RobotHardwareConfig.RIGHT_FRONT_DIR)
                    .rightRearMotorDirection(RobotHardwareConfig.RIGHT_REAR_DIR)
                    .motorCachingThreshold(0.01)
                    .useVoltageCompensation(false)
                    .nominalVoltage(12.0);

    public static PinpointConstants localizerConstants =
            new PinpointConstants()
                    .forwardPodY(RobotHardwareConfig.PINPOINT_FORWARD_POD_Y)
                    .strafePodX(RobotHardwareConfig.PINPOINT_STRAFE_POD_X)
                    .distanceUnit(RobotHardwareConfig.PINPOINT_DISTANCE_UNIT)
                    .hardwareMapName(RobotHardwareConfig.PINPOINT_NAME)
                    .encoderResolution(RobotHardwareConfig.PINPOINT_POD_TYPE)
                    .forwardEncoderDirection(RobotHardwareConfig.PINPOINT_FORWARD_DIR)
                    .strafeEncoderDirection(RobotHardwareConfig.PINPOINT_STRAFE_DIR);

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
