package org.firstinspires.ftc.teamcode.decode;

import static org.firstinspires.ftc.teamcode.modules.MagazineState.ArtifactColor.GREEN;
import static org.firstinspires.ftc.teamcode.modules.MagazineState.ArtifactColor.PURPLE;
import static org.firstinspires.ftc.teamcode.modules.Turret.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.modules.MagazineState;

@Config
public final class DecodeContext {
    public static MagazineState motif = new MagazineState(GREEN, PURPLE, PURPLE);
    public static boolean usedFrontStorage = false;

    public static final Pose redApriltagPose = new Pose(72+58.3727, 72+55.6425, Math.toRadians(54));
    public static final Pose blueApriltagPose = new Pose(72-58.3727, 72+55.6425, Math.toRadians(54));

    public static final Pose redTargetPose = new Pose(141.5 - 17, 144);
    public static final Pose blueTargetPose = new Pose(17, 144);

    public static double turretFieldX = 0;
    public static double turretFieldY = 0;

    /** Aim point corrected for robot velocity when SOTM is on — not the raw goal pose. */
    public static double targetX = 0;
    public static double targetY = 0;

    public static double distanceToGoal = 0;

    public static boolean sotmVelocity = true;
    public static boolean sotmAngle = false;
    public static boolean sotmAccel = true;
    public static double sotmAccelScale = 50;
    public static double sotmDragScale = 0.3;

    /** Must precede module reads; runs before follower.update(), so SOTM kinematics lag one loop (accepted). */
    public static void updateSharedPose(DecodeRobot robot) {
        Pose robotPose = robot.follower.getPose();
        double robotRad = robotPose.getHeading();

        turretFieldX = robotPose.getX() + turretX * Math.cos(robotRad) - turretY * Math.sin(robotRad);
        turretFieldY = robotPose.getY() + turretX * Math.sin(robotRad) + turretY * Math.cos(robotRad);

        double targetX = robot.targetPose.getX();
        double targetY = robot.targetPose.getY();

        if (sotmVelocity) {
            double currentDistance = Math.hypot(targetX - turretFieldX, targetY - turretFieldY);
            robot.turret.flightTime = 0.00103923 * currentDistance + 0.528024;

            Vector robotVelocity = robot.follower.getVelocity();
            double robotVx = robotVelocity.getXComponent();
            double robotVy = robotVelocity.getYComponent();

            targetX -= robotVx * robot.turret.flightTime;
            targetY -= robotVy * robot.turret.flightTime;

            if (sotmAccel) {
                double[] powers = robot.drivetrain.getMotorPowers();
                // powers = [fl, bl, br, fr]; robot +X = forward, +Y = strafe (left-positive)
                double localThrustX = (powers[0] + powers[1] + powers[2] + powers[3]) / 4.0;
                double localThrustY = (powers[0] - powers[1] - powers[2] + powers[3]) / 4.0;

                double heading = robotPose.getHeading(); // NOT negated — local→field uses +heading
                double cos = Math.cos(heading);
                double sin = Math.sin(heading);

                double thrustX = localThrustX * cos - localThrustY * sin;
                double thrustY = localThrustX * sin + localThrustY * cos;

                double accelX = thrustX * sotmAccelScale - robotVx * sotmDragScale;
                double accelY = thrustY * sotmAccelScale - robotVy * sotmDragScale;

                double t = robot.turret.flightTime;
                targetX -= 0.5 * accelX * t * t;
                targetY -= 0.5 * accelY * t * t;
            }

            if (sotmAngle) {
                double angleCompensation = -robot.follower.getAngularVelocity() * robot.turret.flightTime;

                double dx = targetX - turretFieldX;
                double dy = targetY - turretFieldY;

                double cosA = Math.cos(angleCompensation);
                double sinA = Math.sin(angleCompensation);

                targetX = turretFieldX + dx * cosA - dy * sinA;
                targetY = turretFieldY + dx * sinA + dy * cosA;
            }
        }

        DecodeContext.targetX = targetX;
        DecodeContext.targetY = targetY;
        distanceToGoal = Math.hypot(targetX - turretFieldX, targetY - turretFieldY);
    }

    private DecodeContext() {}
}
