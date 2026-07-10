package org.firstinspires.ftc.teamcode.Spline.Field;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Owns the live, editable field state (3 balls, 5 obstacles, and a robot
 * footprint) and draws it to the FTC Dashboard field overlay, using Pedro
 * Pathing's coordinate convention: inches, range [0, 144] on both axes,
 * (0, 0) at the bottom-left corner of the field. Pedro Poses you already
 * build will line up directly.
 *
 * BALL COLLECTION ORDER & OBSTACLE CHECKING
 * ------------------------------------------
 * Call getCollectionOrder() to get the 3 balls sorted nearest-neighbour from
 * the robot: Ball[0] is closest to the robot, Ball[1] is closest to Ball[0],
 * Ball[2] is the remaining ball.
 *
 * The full route is: Robot → Ball[0] → Ball[1] → Ball[2] → Robot (return).
 * Call getPathStatus() to get a PathStatus object describing which of these
 * four legs are blocked by an obstacle (accounting for the robot's width).
 *
 * draw() renders each leg as a green dashed line (clear) or red dashed line
 * (blocked) and adds a telemetry summary to the packet so it appears in the
 * Dashboard telemetry panel alongside the field view.
 *
 * HOW TO USE
 * ----------
 * 1. Open the FTC Dashboard web UI (usually http://192.168.43.1:8080/dash).
 * 2. Go to the "Configuration" tab -> find "FieldVisualizer" -> drag ball,
 *    obstacle, or robot positions around. Changes apply live.
 * 3. Go to the "Field" tab to see paths drawn green (clear) or red (blocked).
 *    The telemetry panel on the right shows a text summary.
 * 4. Call FieldVisualizer.update() once per loop from your OpMode.
 *
 * COMBINING WITH PEDRO'S Drawing CLASS
 * --------------------------------------
 * Dashboard only shows the last-sent packet per loop. Build one packet, draw
 * everything onto it, then send once:
 *
 *   TelemetryPacket packet = new TelemetryPacket();
 *   FieldVisualizer.draw(packet);
 *   Drawing.drawRobotOnCanvas(packet.fieldOverlay(), follower.getPose());
 *   FtcDashboard.getInstance().sendTelemetryPacket(packet);
 */
@Config
public class FieldVisualizer {

    public static double FIELD_SIZE_IN = 144.0;

    // ---- Balls (3) ----
    public static Ball ball1 = new Ball(24, 24, 2.0);
    public static Ball ball2 = new Ball(100, 30, 2.0);
    public static Ball ball3 = new Ball(72, 120, 2.0);

    // ---- Obstacles (5) ----
    public static Obstacle obstacle1 = new Obstacle(0, 0, 6.0);
    public static Obstacle obstacle2 = new Obstacle(0, 0, 6.0);
    public static Obstacle obstacle3 = new Obstacle(0, 0, 6.0);
    public static Obstacle obstacle4 = new Obstacle(0, 0, 6.0);
    public static Obstacle obstacle5 = new Obstacle(0, 0, 6.0);

    // ---- Robot (editable position/heading, 18x18in footprint) ----
    public static double robotX        = 72;
    public static double robotY        = 72;
    public static double robotHeadingDeg = 0; // 0 = facing +x, CCW positive
    public static double ROBOT_SIZE_IN = 18.0;

    // ---- Clearance used for obstacle checks ----
    // Defaults to half the robot width so the robot body is fully accounted
    // for. Editable via Dashboard config if you want tighter/looser margins.
    public static double CLEARANCE_IN = ROBOT_SIZE_IN / 2.0;

    // ── colours ──────────────────────────────────────────────────────────────
    private static final String COLOR_PATH_CLEAR   = "#00C853"; // green
    private static final String COLOR_PATH_BLOCKED = "#D50000"; // red

    // =========================================================================
    //  Data container for one leg's blocked status
    // =========================================================================

    /** Describes whether each leg of the collection route is blocked. */
    public static class PathStatus {
        /** Balls in the order they will be collected (nearest-neighbour). */
        public final Ball[] order;          // length 3

        /** True if the straight-line leg is blocked by at least one obstacle. */
        public final boolean robotToBall0;  // Robot  → order[0]
        public final boolean ball0ToBall1;  // order[0] → order[1]
        public final boolean ball1ToBall2;  // order[1] → order[2]
        public final boolean ball2ToRobot;  // order[2] → Robot (return leg)

        PathStatus(Ball[] order,
                   boolean robotToBall0,
                   boolean ball0ToBall1,
                   boolean ball1ToBall2,
                   boolean ball2ToRobot) {
            this.order        = order;
            this.robotToBall0 = robotToBall0;
            this.ball0ToBall1 = ball0ToBall1;
            this.ball1ToBall2 = ball1ToBall2;
            this.ball2ToRobot = ball2ToRobot;
        }

        /** True if every leg is clear. */
        public boolean allClear() {
            return !robotToBall0 && !ball0ToBall1 && !ball1ToBall2 && !ball2ToRobot;
        }

        /**
         * Human-readable label for a leg, e.g. "Robot→B2" where the number
         * is the ball's original index (1-based) in FieldVisualizer.
         */
        public String legLabel(int legIndex) {
            String[] names = new String[3];
            List<Ball> allBalls = Arrays.asList(ball1, ball2, ball3);
            for (int i = 0; i < 3; i++) {
                int idx = allBalls.indexOf(order[i]) + 1; // 1-based
                names[i] = "B" + idx;
            }
            switch (legIndex) {
                case 0: return "Robot→" + names[0];
                case 1: return names[0] + "→" + names[1];
                case 2: return names[1] + "→" + names[2];
                case 3: return names[2] + "→Robot";
                default: return "?";
            }
        }

        public boolean[] asArray() {
            return new boolean[]{robotToBall0, ball0ToBall1, ball1ToBall2, ball2ToRobot};
        }
    }

    // =========================================================================
    //  Static utility / public API
    // =========================================================================

    private FieldVisualizer() { /* static-only utility class */ }

    public static List<Ball> getBalls() {
        return Arrays.asList(ball1, ball2, ball3);
    }

    public static List<Obstacle> getObstacles() {
        return Arrays.asList(obstacle1, obstacle2, obstacle3, obstacle4, obstacle5);
    }

    /** Convenience for path-planning code: balls as Pedro Poses (heading 0). */
    public static List<Pose> getBallPoses() {
        return Arrays.asList(ball1.toPose(), ball2.toPose(), ball3.toPose());
    }

    /** Current dashboard-editable robot position/heading as a Pedro Pose. */
    public static Pose getRobotPose() {
        return new Pose(robotX, robotY, Math.toRadians(robotHeadingDeg));
    }

    /**
     * Returns the 3 balls sorted in collection order using a nearest-neighbour
     * greedy algorithm:
     *   1. Pick the ball closest to the robot.
     *   2. From that ball, pick the closest remaining ball.
     *   3. The last ball is whatever remains.
     *
     * This is the order the robot will visit them, so index 0 is first pickup,
     * index 2 is last pickup.
     */
    public static Ball[] getCollectionOrder() {
        List<Ball> remaining = new ArrayList<>(getBalls());
        Ball[] ordered = new Ball[3];

        // Step 1: closest ball to the robot
        double fromX = robotX;
        double fromY = robotY;
        for (int pick = 0; pick < 3; pick++) {
            Ball nearest = null;
            double bestDist = Double.MAX_VALUE;
            for (Ball b : remaining) {
                double d = Math.hypot(b.x - fromX, b.y - fromY);
                if (d < bestDist) {
                    bestDist = d;
                    nearest  = b;
                }
            }
            ordered[pick] = nearest;
            remaining.remove(nearest);
            // Next step starts from the ball just chosen
            fromX = nearest.x;
            fromY = nearest.y;
        }
        return ordered;
    }

    /**
     * Computes which legs of the route Robot→B0→B1→B2→Robot are blocked by
     * an obstacle (using CLEARANCE_IN as the robot half-width margin).
     *
     * @return a PathStatus with the collection order and per-leg blocked flags.
     */
    public static PathStatus getPathStatus() {
        Ball[]         order     = getCollectionOrder();
        List<Obstacle> obstacles = getObstacles();
        double         cl        = CLEARANCE_IN;

        boolean leg0 = Obstacle.anyBlocks(obstacles, robotX,     robotY,
                order[0].x, order[0].y, cl);
        boolean leg1 = Obstacle.anyBlocks(obstacles, order[0].x, order[0].y,
                order[1].x, order[1].y, cl);
        boolean leg2 = Obstacle.anyBlocks(obstacles, order[1].x, order[1].y,
                order[2].x, order[2].y, cl);
        boolean leg3 = Obstacle.anyBlocks(obstacles, order[2].x, order[2].y,
                robotX,     robotY,     cl);

        return new PathStatus(order, leg0, leg1, leg2, leg3);
    }

    // =========================================================================
    //  Drawing
    // =========================================================================

    /**
     * Draws the full field state onto the packet's field overlay and adds a
     * telemetry text summary of which legs are blocked.
     *
     * FTC Dashboard's Canvas origin is at the field CENTER. We apply
     * setTranslation so every call below uses Pedro's [0, 144] coordinates.
     */
    public static void draw(TelemetryPacket packet) {
        draw(packet, true);
    }

    /**
     * Same as {@link #draw(TelemetryPacket)}, but lets the caller choose
     * whether to draw the dashboard-editable SEED robot square.
     *
     * Pass {@code includeSeedRobot = false} once the robot is actually
     * driving and the live robot is being drawn via
     * {@link #drawLiveRobotOnCanvas} instead — otherwise the field view shows
     * two robots (the frozen seed square plus the moving live one), which is
     * confusing. Pass {@code true} while idle so the seed square is the one
     * (editable) robot on screen.
     */
    public static void draw(TelemetryPacket packet, boolean includeSeedRobot) {
        Canvas         field  = packet.fieldOverlay();
        PathStatus     status = getPathStatus();
        Ball[]         order  = status.order;
        boolean[]      blocked = status.asArray();

        // ── canvas transform: shift origin to Pedro's bottom-left (0,0) ──────
        field.setTranslation(-FIELD_SIZE_IN / 2.0, -FIELD_SIZE_IN / 2.0);
        field.setScale(1, 1);

        // ── field boundary ────────────────────────────────────────────────────
        field.setStroke("#444444");
        field.setStrokeWidth(1);
        field.strokeRect(0, 0, FIELD_SIZE_IN, FIELD_SIZE_IN);

        // ── path legs (drawn first so balls/obstacles render on top) ──────────
        // Waypoints in route order: Robot, B0, B1, B2, Robot
        double[] wx = {robotX,     order[0].x, order[1].x, order[2].x, robotX};
        double[] wy = {robotY,     order[0].y, order[1].y, order[2].y, robotY};

        for (int leg = 0; leg < 4; leg++) {
            drawDashedLine(field,
                    wx[leg], wy[leg], wx[leg + 1], wy[leg + 1],
                    blocked[leg] ? COLOR_PATH_BLOCKED : COLOR_PATH_CLEAR,
                    blocked[leg] ? 2 : 1);
        }

        // ── obstacles ─────────────────────────────────────────────────────────
        field.setStrokeWidth(2);
        for (Obstacle o : getObstacles()) {
            field.setFill("#FF000033");
            field.fillCircle(o.x, o.y, o.radius);
            field.setStroke("#CC0000");
            field.strokeCircle(o.x, o.y, o.radius);
        }

        // ── balls (labelled with collection-order index, not original index) ──
        List<Ball> allBalls = getBalls();
        int[] collectionIdx = new int[3]; // collectionIdx[i] = 1-based pick order for ball i
        for (int pick = 0; pick < 3; pick++) {
            for (int bi = 0; bi < allBalls.size(); bi++) {
                if (allBalls.get(bi) == order[pick]) {
                    collectionIdx[bi] = pick + 1;
                }
            }
        }

        int i = 0;
        for (Ball b : allBalls) {
            field.setFill("#FFA500");
            field.fillCircle(b.x, b.y, b.radius);
            field.setStroke("#996300");
            field.strokeCircle(b.x, b.y, b.radius);
            field.setStrokeWidth(1);
            // Label: "B1(#2)" means ball1, collected 2nd
            field.strokeText("B" + (i + 1) + "(#" + collectionIdx[i] + ")",
                    b.x + b.radius + 1, b.y, "8px", 0);
            i++;
        }

        // ── robot (seed pose; skipped while the live robot is drawn instead) ──
        if (includeSeedRobot) {
            drawRobot(field);
        }

        // ── telemetry text summary ────────────────────────────────────────────
        packet.addLine("=== Collection Route ===");
        for (int leg = 0; leg < 4; leg++) {
            String label  = status.legLabel(leg);
            String state  = blocked[leg] ? "BLOCKED" : "clear";
            packet.addLine(label + ": " + state);

            if (blocked[leg]) {
                // Report which specific obstacle(s) block this leg
                double x1 = wx[leg],   y1 = wy[leg];
                double x2 = wx[leg+1], y2 = wy[leg+1];
                int obsIdx = 1;
                for (Obstacle o : getObstacles()) {
                    if (o.intersectsSegment(x1, y1, x2, y2, CLEARANCE_IN)) {
                        packet.addLine("  ↳ blocked by Obstacle" + obsIdx
                                + " at (" + String.format("%.0f", o.x)
                                + ", " + String.format("%.0f", o.y) + ")");
                    }
                    obsIdx++;
                }
            }
        }
        packet.addLine(status.allClear() ? "✓ Route is fully clear" : "✗ Route has blocked legs");
        packet.addLine("Clearance: " + CLEARANCE_IN + " in");
    }

    /**
     * Draws a dashed line between two points in Pedro's [0,144] coordinate
     * space. Dash segments are 6 in long with 4 in gaps — a good balance
     * between visible detail and screen clutter at the 144-in scale.
     */
    private static void drawDashedLine(Canvas field,
                                       double x1, double y1,
                                       double x2, double y2,
                                       String color, int strokeWidth) {
        double totalLen = Math.hypot(x2 - x1, y2 - y1);
        if (totalLen < 0.001) return;

        double dashLen = 6.0;
        double gapLen  = 4.0;
        double period  = dashLen + gapLen;
        double ux      = (x2 - x1) / totalLen;
        double uy      = (y2 - y1) / totalLen;

        field.setStroke(color);
        field.setStrokeWidth(strokeWidth);

        double t = 0;
        while (t < totalLen) {
            double tEnd  = Math.min(t + dashLen, totalLen);
            double sx    = x1 + t    * ux;
            double sy    = y1 + t    * uy;
            double ex    = x1 + tEnd * ux;
            double ey    = y1 + tEnd * uy;
            field.strokeLine(sx, sy, ex, ey);
            t += period;
        }
    }

    /** Draws the robot square and heading tick in Pedro's coordinate frame. */
    private static void drawRobot(Canvas field) {
        double heading = Math.toRadians(robotHeadingDeg);
        double half    = ROBOT_SIZE_IN / 2.0;
        double cos     = Math.cos(heading);
        double sin     = Math.sin(heading);

        double[] localX  = { half,  half, -half, -half};
        double[] localY  = { half, -half, -half,  half};
        double[] worldX  = new double[4];
        double[] worldY  = new double[4];
        for (int i = 0; i < 4; i++) {
            worldX[i] = robotX + localX[i] * cos - localY[i] * sin;
            worldY[i] = robotY + localX[i] * sin + localY[i] * cos;
        }

        field.setFill("#2196F333");
        field.fillPolygon(worldX, worldY);
        field.setStroke("#2196F3");
        field.setStrokeWidth(2);
        field.strokePolygon(worldX, worldY);

        double frontX = robotX + half * cos;
        double frontY = robotY + half * sin;
        field.strokeLine(robotX, robotY, frontX, frontY);
    }

    /**
     * Draws the live robot (from Pedro's follower.getPose()) onto a canvas that
     * already has the Pedro [0,144] transform applied (i.e. after draw() has
     * been called on the same packet). Call this after FieldVisualizer.draw()
     * and before sending the packet, instead of Drawing.drawRobotOnCanvas()
     * which is not compatible with an externally-managed packet in Pedro 2.x.
     *
     * Renders a green square footprint + heading tick using the supplied Pose.
     */
    public static void drawLiveRobotOnCanvas(Canvas field, Pose pose) {
        double heading = pose.getHeading();
        double half    = ROBOT_SIZE_IN / 2.0;
        double cos     = Math.cos(heading);
        double sin     = Math.sin(heading);
        double cx      = pose.getX();
        double cy      = pose.getY();

        double[] localX = { half,  half, -half, -half};
        double[] localY = { half, -half, -half,  half};
        double[] worldX = new double[4];
        double[] worldY = new double[4];
        for (int i = 0; i < 4; i++) {
            worldX[i] = cx + localX[i] * cos - localY[i] * sin;
            worldY[i] = cy + localX[i] * sin + localY[i] * cos;
        }

        field.setFill("#4CAF5033");      // green fill (semi-transparent)
        field.fillPolygon(worldX, worldY);
        field.setStroke("#4CAF50");      // green outline
        field.setStrokeWidth(2);
        field.strokePolygon(worldX, worldY);

        // Heading tick from centre to front edge
        double frontX = cx + half * cos;
        double frontY = cy + half * sin;
        field.strokeLine(cx, cy, frontX, frontY);
    }

    /**
     * Draws the ACTUAL route the robot will drive — the sampled spline
     * curves from the built PathChains — as a smooth solid blue line.
     * Because the points passed in are dense samples along each BezierCurve,
     * the drawn line IS the spline shape, curves and all, not a straight
     * polyline approximation.
     *
     * Optionally pass the leg anchor poses (leg start/end points: robot
     * start, each ball, return point) to mark them with small hollow
     * circles, so you can see where one PathChain leg hands off to the next.
     *
     * Safe to call with null / too-short lists — it just does nothing.
     */
    public static void drawPlannedPath(Canvas field, List<Pose> splineSamples) {
        drawPlannedPath(field, splineSamples, null);
    }

    public static void drawPlannedPath(Canvas field, List<Pose> splineSamples,
                                       List<Pose> legAnchors) {
        if (splineSamples == null || splineSamples.size() < 2) return;

        // The spline itself: dense samples joined into a smooth curve
        field.setStroke("#2962FF");
        field.setStrokeWidth(2);
        for (int i = 0; i < splineSamples.size() - 1; i++) {
            Pose a = splineSamples.get(i);
            Pose b = splineSamples.get(i + 1);
            field.strokeLine(a.getX(), a.getY(), b.getX(), b.getY());
        }

        // Leg boundaries: where one PathChain ends and the next begins
        if (legAnchors != null) {
            field.setStroke("#0D47A1");
            field.setStrokeWidth(2);
            for (Pose p : legAnchors) {
                field.strokeCircle(p.getX(), p.getY(), 2.5);
            }
        }
    }

    /**
     * Convenience one-call method: builds a fresh packet, draws everything,
     * and sends to the dashboard. Use this only when nothing else is sending
     * its own packet this loop (see class header).
     */
    public static void update() {
        TelemetryPacket packet = new TelemetryPacket();
        draw(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}