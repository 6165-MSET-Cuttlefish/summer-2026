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

    // Heading is 54°; Pedro Pose headings are RADIANS, so convert. (Currently unused while Limelight
    // is stubbed, but leaving it in degrees is a latent unit-mismatch landmine when vision returns.)
    public static final Pose redApriltagPose = new Pose(72+58.3727, 72+55.6425, Math.toRadians(54));
    public static final Pose blueApriltagPose = new Pose(72-58.3727, 72+55.6425, Math.toRadians(54));

    public static final Pose redTargetPose = new Pose(141.5 - 17, 144);
    public static final Pose blueTargetPose = new Pose(17, 144);

    /** Turret position in field coordinates (inches). */
    public static double turretFieldX = 0;
    public static double turretFieldY = 0;

    /** Target position (corrected for robot translational velocity if on). */
    public static double targetX = 0;
    public static double targetY = 0;

    /** Straight-line distance from turret to (corrected) target (inches). */
    public static double distanceToGoal = 0;

    /** Shot-on-the-move compensation toggles. */
    public static boolean sotmVelocity = true;
    public static boolean sotmAngle = false;
    public static boolean sotmAccel = true;
    public static double sotmAccelScale = 50;
    public static double sotmDragScale = 0.3;

    /**
     * Recomputes all shared geometric values once per loop cycle.
     * Must be called before any module reads() that depend on these fields.
     *
     * <p>Called from {@code DecodeOpMode.onLoopStart()}, which the framework runs BEFORE
     * {@code follower.update()} (so it can precede module reads that consume these fields). Pedro's
     * getPose()/getVelocity()/getAngularVelocity() therefore return the PREVIOUS loop's cached
     * kinematics — the shot-on-the-move corrections here lag the robot by one loop (~a few ms; sub-inch
     * at match speeds). Accepted tradeoff, matching the DECODE-season pipeline. If SOTM precision ever
     * needs current-loop kinematics, move {@code follower.update()} ahead of the read pass framework-wide.
     *
     * @param robot the current DecodeRobot instance
     */
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
//            0.00103923x+0.528024

            Vector robotVelocity = robot.follower.getVelocity();
            double robotVx = robotVelocity.getXComponent();
            double robotVy = robotVelocity.getYComponent();

            targetX -= robotVx * robot.turret.flightTime;
            targetY -= robotVy * robot.turret.flightTime;

            if (sotmAccel) {
                double[] powers = robot.drivetrain.getMotorPowers();
                // powers = [fl, bl, br, fr]
                // Standard mecanum decomposition:
                //   forward (robot +X) = average of all four
                //   strafe  (robot +Y) = fl - bl - br + fr  (left-positive)
                double localThrustX = (powers[0] + powers[1] + powers[2] + powers[3]) / 4.0; // forward
                double localThrustY = (powers[0] - powers[1] - powers[2] + powers[3]) / 4.0; // strafe

                // Rotate robot-local thrust into field space using robot heading
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
