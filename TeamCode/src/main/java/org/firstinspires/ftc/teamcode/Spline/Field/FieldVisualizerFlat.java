package org.firstinspires.ftc.teamcode.Spline.Field;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;

import java.util.Arrays;
import java.util.List;

/**
 * BACKUP / FALLBACK VERSION of FieldVisualizer — single file, no nested
 * config objects.
 *
 * Functionally identical to the multi-file version (Ball.java + Obstacle.java
 * + FieldVisualizer.java), but every ball/obstacle value is flattened into
 * plain "public static double" fields directly on this class instead of
 * living inside nested Ball/Obstacle objects.
 *
 * The multi-file version with nested objects genuinely should work — FTC
 * Dashboard's @Config reflection does recurse into a custom class's public
 * instance fields regardless of where that class is defined (verified
 * directly against ReflectionConfig's source). But if you ever hit a
 * dashboard version, build setup, or ProGuard/R8 stripping issue where the
 * nested ball1/obstacle1 objects don't show up as expected, swap to this
 * version instead — flat primitive fields are about as close to
 * "guaranteed to show up" as @Config gets, since there's no reflection
 * recursion involved at all.
 *
 * IMPORTANT: this file defines its own public class named
 * FieldVisualizerFlat (not FieldVisualizer), so it can sit in the same
 * package alongside the original multi-file version without a duplicate
 * class name conflict. If you actually want to SWITCH to this version:
 *   1. Delete (or move out of the package) the old Ball.java, Obstacle.java,
 *      and FieldVisualizer.java.
 *   2. Rename this file to FieldVisualizer.java and rename the class below
 *      from FieldVisualizerFlat to FieldVisualizer.
 *   3. Update any OpMode calling FieldVisualizer.update() — no change needed
 *      if you did step 2, since the name matches again.
 * Until you do that, this coexists fine as a separate, unused-until-you-
 * need-it backup.
 */
@Config
public class FieldVisualizerFlat {

    public static double FIELD_SIZE_IN = 144.0;

    // ---- Dashboard Editable Flattened Primitives ----
    // Dashboard reads these perfectly, no reflection recursion required.
    public static double b1x = 24, b1y = 24, b1r = 2.0;
    public static double b2x = 100, b2y = 30, b2r = 2.0;
    public static double b3x = 72, b3y = 120, b3r = 2.0;

    public static double o1x = 40, o1y = 40, o1r = 6.0;
    public static double o2x = 100, o2y = 40, o2r = 6.0;
    public static double o3x = 40, o3y = 100, o3r = 6.0;
    public static double o4x = 100, o4y = 100, o4r = 6.0;
    public static double o5x = 72, o5y = 72, o5r = 6.0;

    public static double robotX = 72.0;
    public static double robotY = 18.0;
    public static double robotHeadingDeg = 90.0; // 0 = facing +x, CCW positive
    public static double ROBOT_SIZE_IN = 18.0;

    private FieldVisualizerFlat() {
        // static-only utility class
    }

    // ---- Object Getters ----
    // Rebuilds objects dynamically when called so they always have the live
    // dashboard values (no stale cached instances to worry about).
    public static List<Ball> getBalls() {
        return Arrays.asList(
                new Ball(b1x, b1y, b1r),
                new Ball(b2x, b2y, b2r),
                new Ball(b3x, b3y, b3r)
        );
    }

    public static List<Obstacle> getObstacles() {
        return Arrays.asList(
                new Obstacle(o1x, o1y, o1r),
                new Obstacle(o2x, o2y, o2r),
                new Obstacle(o3x, o3y, o3r),
                new Obstacle(o4x, o4y, o4r),
                new Obstacle(o5x, o5y, o5r)
        );
    }

    /** Convenience for path-planning code: balls as Pedro Poses (heading 0). */
    public static List<Pose> getBallPoses() {
        return Arrays.asList(
                new Pose(b1x, b1y, 0),
                new Pose(b2x, b2y, 0),
                new Pose(b3x, b3y, 0)
        );
    }

    /** Current dashboard-editable robot position/heading as a Pedro Pose. */
    public static Pose getRobotPose() {
        return new Pose(robotX, robotY, Math.toRadians(robotHeadingDeg));
    }

    /**
     * Draws the field boundary, balls, obstacles, and robot onto the given
     * packet's field overlay. Split out from update() so it can be combined
     * with other drawing (e.g. Pedro's own Drawing class) on the same packet
     * before sending — see FieldVisualizer's original header note on why
     * that matters (FTC Dashboard's field view only shows the most recently
     * sent packet).
     */
    public static void draw(TelemetryPacket packet) {
        Canvas field = packet.fieldOverlay();

        // Field boundary, drawn in Pedro's [0,144] frame
        field.setStroke("#444444");
        field.strokeRect(0, 0, FIELD_SIZE_IN, FIELD_SIZE_IN);

        // Obstacles: red outline + light red fill so overlap is visible
        field.setStrokeWidth(2);
        for (Obstacle o : getObstacles()) {
            field.setFill("#FF000033");
            field.fillCircle(o.x, o.y, o.radius);
            field.setStroke("#CC0000");
            field.strokeCircle(o.x, o.y, o.radius);
        }

        // Balls: solid orange circles with a small label
        int i = 1;
        for (Ball b : getBalls()) {
            field.setFill("#FFA500");
            field.fillCircle(b.x, b.y, b.radius);
            field.setStroke("#996300");
            field.strokeCircle(b.x, b.y, b.radius);
            field.setStrokeWidth(1);
            field.strokeText("B" + i, b.x + b.radius + 1, b.y, "8px", 0);
            i++;
        }

        // Robot: 18x18in square footprint, rotated by robotHeadingDeg, with
        // a heading tick line so you can see which way it's "facing"
        drawRobot(field);
    }

    /**
     * Draws the robot footprint as a square of side ROBOT_SIZE_IN, centered
     * at (robotX, robotY) and rotated by robotHeadingDeg.
     */
    private static void drawRobot(Canvas field) {
        double heading = Math.toRadians(robotHeadingDeg);
        double half = ROBOT_SIZE_IN / 2.0;
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double[] localX = {half, half, -half, -half};
        double[] localY = {half, -half, -half, half};
        double[] worldX = new double[4];
        double[] worldY = new double[4];
        for (int i = 0; i < 4; i++) {
            worldX[i] = robotX + localX[i] * cos - localY[i] * sin;
            worldY[i] = robotY + localX[i] * sin + localY[i] * cos;
        }

        field.setFill("#2196F333");
        field.fillPolygon(worldX, worldY);
        field.setStroke("#2196F3");
        field.setStrokeWidth(2);
        field.strokePolygon(worldX, worldY);

        // Heading tick: line from center to the middle of the "front" edge
        double frontX = robotX + half * cos;
        double frontY = robotY + half * sin;
        field.strokeLine(robotX, robotY, frontX, frontY);
    }

    /**
     * Convenience one-call method: builds a fresh packet, draws the field
     * state onto it, and sends it to the dashboard.
     */
    public static void update() {
        TelemetryPacket packet = new TelemetryPacket();
        draw(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    // ---- Helper classes, kept here so this file is fully self-contained ----

    /** Simple ball data holder. Same shape as the standalone Ball.java. */
    static class Ball {
        public double x, y, radius;

        Ball(double x, double y, double radius) {
            this.x = x;
            this.y = y;
            this.radius = radius;
        }

        Pose toPose() {
            return new Pose(x, y, 0);
        }

        @Override
        public String toString() {
            return String.format("Ball(%.1f, %.1f)", x, y);
        }
    }

    /** Simple obstacle data holder. Same shape as the standalone Obstacle.java. */
    static class Obstacle {
        public double x, y, radius;

        Obstacle(double x, double y, double radius) {
            this.x = x;
            this.y = y;
            this.radius = radius;
        }

        boolean contains(double px, double py) {
            double dx = px - x;
            double dy = py - y;
            return (dx * dx + dy * dy) <= (radius * radius);
        }

        boolean intersectsSegment(double x1, double y1, double x2, double y2, double clearance) {
            double dx = x2 - x1;
            double dy = y2 - y1;
            double lenSq = dx * dx + dy * dy;
            double t = lenSq == 0 ? 0 : ((x - x1) * dx + (y - y1) * dy) / lenSq;
            t = Math.max(0, Math.min(1, t));
            double closestX = x1 + t * dx;
            double closestY = y1 + t * dy;
            double distSq = (closestX - x) * (closestX - x) + (closestY - y) * (closestY - y);
            double minDist = radius + clearance;
            return distSq <= minDist * minDist;
        }

        @Override
        public String toString() {
            return String.format("Obstacle(%.1f, %.1f, r=%.1f)", x, y, radius);
        }
    }
}