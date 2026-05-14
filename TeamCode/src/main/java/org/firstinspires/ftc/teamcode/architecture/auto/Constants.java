package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants =
            new FollowerConstants()
//                    .mass(16.3)
//                    .forwardZeroPowerAcceleration(-39.204194138505336)
//                    .lateralZeroPowerAcceleration(-75.65244466725856)
//                    .translationalPIDFCoefficients(new PIDFCoefficients(0.28, 0, 0.028, 0))
                    .headingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.05, 0.03))
//                    .drivePIDFCoefficients(
//                            new FilteredPIDFCoefficients(0.05, 0, 0.01, 0.6, 0))
                    .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.14, 0.1973407216665132,0.001005159209640557))
                    .centripetalScaling(0.00);

    public static MecanumConstants driveConstants =
            new MecanumConstants()
                    .maxPower(1)
                    .rightFrontMotorName("fr")
                    .rightRearMotorName("br")
                    .leftRearMotorName("bl")
                    .leftFrontMotorName("fl")
                    .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
//                    .xVelocity(79.31000572865403)
//                    .yVelocity(56.44304602915846)
                    .motorCachingThreshold(0.01)
                    .useVoltageCompensation(true)
                    .nominalVoltage(11.5);

    public static PinpointConstants localizerConstants =
            new PinpointConstants()
                    .forwardPodY(-5.5) // Measured 2/17
                    .strafePodX(1) // Measured 2/17
                    .distanceUnit(DistanceUnit.INCH)
                    .hardwareMapName("pinpoint")
                    .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                    .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                    .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.95, 0, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
