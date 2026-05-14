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
 * Pedro Pathing tuning + the {@link #createFollower(HardwareMap)} factory. <strong>Edit this
 * file when retuning</strong> path-following constants. Wire-it-up names + physical offsets
 * live in {@link RobotHardwareConfig}.
 *
 * <p>All public statics are non-final so they show up as live-tunable on FTC Dashboard.
 */
@Config
public final class PedroSetup {
    private PedroSetup() {}

    // ── Follower / control loop tuning ─────────────────────────────────────

    public static FollowerConstants followerConstants =
            new FollowerConstants()
                    // Pre-season placeholders — retune for the summer test robot.
                    .headingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.05, 0.03))
                    .predictiveBrakingCoefficients(
                            new PredictiveBrakingCoefficients(0.14, 0.1973407216665132, 0.001005159209640557))
                    .centripetalScaling(0.0);

    // ── Drivetrain hookup (names/dirs come from RobotHardwareConfig) ───────

    public static MecanumConstants driveConstants =
            new MecanumConstants()
                    .maxPower(1.0)
                    .leftFrontMotorName(RobotHardwareConfig.LEFT_FRONT_MOTOR)
                    .leftRearMotorName(RobotHardwareConfig.LEFT_REAR_MOTOR)
                    .rightFrontMotorName(RobotHardwareConfig.RIGHT_FRONT_MOTOR)
                    .rightRearMotorName(RobotHardwareConfig.RIGHT_REAR_MOTOR)
                    .leftFrontMotorDirection(RobotHardwareConfig.LEFT_FRONT_DIR)
                    .leftRearMotorDirection(RobotHardwareConfig.LEFT_REAR_DIR)
                    .rightFrontMotorDirection(RobotHardwareConfig.RIGHT_FRONT_DIR)
                    .rightRearMotorDirection(RobotHardwareConfig.RIGHT_REAR_DIR)
                    .motorCachingThreshold(0.01)
                    .useVoltageCompensation(true)
                    .nominalVoltage(11.5);

    public static PinpointConstants localizerConstants =
            new PinpointConstants()
                    .forwardPodY(RobotHardwareConfig.PINPOINT_FORWARD_POD_Y)
                    .strafePodX(RobotHardwareConfig.PINPOINT_STRAFE_POD_X)
                    .distanceUnit(RobotHardwareConfig.PINPOINT_DISTANCE_UNIT)
                    .hardwareMapName(RobotHardwareConfig.PINPOINT_NAME)
                    .encoderResolution(RobotHardwareConfig.PINPOINT_POD_TYPE)
                    .forwardEncoderDirection(RobotHardwareConfig.PINPOINT_FORWARD_DIR)
                    .strafeEncoderDirection(RobotHardwareConfig.PINPOINT_STRAFE_DIR);

    // ── Path constraints ───────────────────────────────────────────────────
    // Pedro 2.x defaults are (tValue=0.995, velocity=0.1, translational=0.1, heading=0.007,
    // timeout=100ms, brakingStrength=1, bezierSearchLimit=10, brakingStart=1). Start with
    // those and tune per-path via IntegratedPathBuilder.setConstraints(...) where needed.
    public static PathConstraints pathConstraints = PathConstraints.defaultConstraints;

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
