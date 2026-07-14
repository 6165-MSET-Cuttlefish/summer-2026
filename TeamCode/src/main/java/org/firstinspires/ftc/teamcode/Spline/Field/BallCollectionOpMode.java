package org.firstinspires.ftc.teamcode.Spline.Field;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Spline.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Spline.Mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.Spline.Pedro.PedroSetup;

import java.util.ArrayList;
import java.util.List;

/**
 * Ball Collection OpMode.
 *
 * Two paths, built once while idle:
 *
 *  PATH 1 (intake) - up to ONE cubic BezierCurve, 4 points total:
 *      start (robot)  ->  control 1  ->  control 2  ->  end (last ball)
 *    The 2 control points are SOLVED (not just placed at ball positions)
 *    so the curve passes exactly through the first two balls (in visit
 *    order) at fixed points along it (t = INTAKE_T1/T2). The 3rd ball is
 *    the curve's actual endpoint, so it's touched exactly too. Heading
 *    stays tangent to the curve throughout.
 *
 *    EFFICIENCY: the intake is as wide as the whole robot
 *    (FieldVisualizer.INTAKE_WIDTH_IN), so a ball doesn't need to be an
 *    explicit point on the curve if driving straight past another stop
 *    already sweeps it up. essentialIntakeStops() drops any such ball
 *    first, so e.g. two balls sitting side-by-side within intake width of
 *    each other collapse into a single straight BezierLine instead of a
 *    curve bent to touch both centres - see buildIntakeCurve.
 *
 *  PATH 2 (return) - a straight BezierLine from wherever path 1 ends back
 *    to the original start pose. Heading interpolates linearly from the
 *    robot's actual heading at the end of path 1 back to the ORIGINAL
 *    start heading, so the robot returns to the exact spot AND
 *    orientation it started at.
 *
 * OBSTACLE REROUTING: the smooth curve/line above assumes nothing is in the
 * way. Whenever it actually clips an obstacle, each affected leg is rebuilt,
 * in order of preference:
 *   1. A single BIGGER SPLINE: one BezierCurve.through(...) solved so it
 *      passes exactly through the robot/ball points PLUS extra detour
 *      control points that route around the obstacle (the same waypoints
 *      RouteOptimizer already planned via VisibilityGraphPlanner while
 *      choosing ball visit order). More control points, still one smooth
 *      curve - no sharp corners.
 *   2. Only if that bigger spline still clips something (a tight obstacle
 *      cluster), a guaranteed-safe multi-segment polyline of straight
 *      BezierLines through the same waypoints, as a last resort.
 * See buildIntakeCurve/buildReturnPath.
 *
 * The intake and transfer motors spin up the moment the route starts and
 * shut off once the robot is back home (path 2 complete).
 *
 * Setting run=1 on the Dashboard freezes whatever is currently planned and
 * drives path 1 then path 2, back to back. follower.update() runs every
 * loop once started, regardless of `run`, so the robot is guaranteed to
 * actually brake to a stop once path 2 finishes.
 */
@Config
@TeleOp(name = "Ball Collection", group = "Spline")
public class BallCollectionOpMode extends OpMode {

    // Tunable via Dashboard
    public static double INTAKE_T1                 = 1.0 / 3.0; // where ball 1 sits on the curve (0-1)
    public static double INTAKE_T2                 = 2.0 / 3.0; // where ball 2 sits on the curve (0-1)
    public static double TIMEOUT_MIN_AVG_SPEED_IPS  = 10.0;
    public static double TIMEOUT_MIN_SEC            = 4.0;
    public static double END_DISTANCE_TOLERANCE_IN  = 2.0;
    public static double END_VELOCITY_TOLERANCE     = 1.0;
    public static int    CURVE_CHECK_SAMPLES        = 60; // used only for the obstacle warning check
    public static int    run                        = 0;  // set to 1 to start

    private Follower follower;
    private final Timer legTimer = new Timer();
    private Intake intake;
    private Transfer transfer;

    // pathState: -1 idle, 0 = driving intake curve, 1 = driving return path, 2 = done
    private int pathState = -1;
    private boolean prevRun = false;

    private PathChain intakePath;
    private PathChain returnPath;
    private Pose intakeEndPose;       // where path 1 ends (= last ball)
    private double intakeEndHeading;  // heading the robot is actually facing at the end of path 1
    private double intakeTimeoutSec;
    private double returnTimeoutSec;
    private boolean intakeRerouted;      // true if path 1 had to dodge an obstacle (spline or hard fallback)
    private boolean returnRerouted;      // true if path 2 had to dodge an obstacle (spline or hard fallback)
    private boolean intakeHardFallback;  // true if even the bigger-spline reroute still clipped something
    private boolean returnHardFallback;  // true if even the bigger-spline reroute still clipped something
    private int intakeEssentialStopCount; // how many of the 3 balls the intake curve actually had to touch

    private Pose startPose;
    private boolean planReady = false;
    private double lastConfigSignature = Double.NaN;

    // For dashboard drawing / telemetry only
    private RouteOptimizer.Route lastRoute;
    private List<Pose> intakeSamples; // dense points along path 1, for drawing
    private List<Pose> returnSamples; // dense points along path 2, for drawing
    private String lastEndReason = "";

    private boolean isIdle() { return pathState == -1; }
    private boolean isDone() { return pathState == 2; }

    // ===== init / start =====

    @Override
    public void init() {
        follower = PedroSetup.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        startPose = FieldVisualizer.getRobotPose();
        follower.setStartingPose(startPose);
        rebuildPlan(startPose);
        telemetry.addLine("Paths pre-built. Drag things on Dashboard, then set run=1 to go.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        rebuildIfConfigChanged();
        drawDashboard(startPose);
    }

    @Override
    public void start() {
        follower.setStartingPose(startPose); // re-seed after Pinpoint calibration
        prevRun = (run == 1); // don't count a pre-set run=1 as a fresh trigger
    }

    // ===== main loop =====

    @Override
    public void loop() {
        if (isIdle()) {
            rebuildIfConfigChanged();
        }

        // Only start on the 0 -> 1 edge of `run`. Whether we KEEP driving
        // is controlled purely by pathState, not by `run` itself.
        boolean startTriggered = (run == 1) && !prevRun;
        prevRun = (run == 1);
        if (startTriggered) {
            startRoute();
        }

        // Drive to completion every loop once started - never skip this,
        // or the robot keeps coasting with stale motor power.
        if (!isIdle() && !isDone()) {
            follower.update();
            advanceFSM();
        }

        Pose displayPose = isIdle() ? startPose : follower.getPose();
        drawDashboard(displayPose);
        showTelemetry(displayPose);
    }

    // ===== driving the two paths =====

    /** Starts path 1 (intake curve). Does no planning. */
    private void startRoute() {
        if (isIdle()) {
            if (!planReady) return;
            follower.setStartingPose(startPose);
        } else if (isDone()) {
            // Rebuild once from wherever we ended up (should be back at
            // start), then run the whole route again.
            startPose = follower.getPose();
            rebuildPlan(startPose);
            if (!planReady) return;
        } else {
            return; // already mid-route
        }

        pathState = 0;
        follower.followPath(intakePath, false);
        legTimer.resetTimer();
        lastEndReason = "";

        intake.intakeMotor.setPower(Intake.intakePower);
        transfer.transferMotor.setPower(Transfer.transferPower);
    }

    /** Moves from intake -> return -> done, braking once the whole route finishes. */
    private void advanceFSM() {
        Pose target = (pathState == 0) ? intakeEndPose : startPose;
        double timeout = (pathState == 0) ? intakeTimeoutSec : returnTimeoutSec;

        String reason = legDoneReason(target, timeout);
        if (reason == null) return;

        lastEndReason = reason;
        if (pathState == 0) {
            pathState = 1;
            follower.followPath(returnPath, false);
            legTimer.resetTimer();
        } else {
            pathState = 2;
            follower.breakFollowing(); // zero drive output - full stop
            intake.intakeMotor.setPower(0);
            transfer.transferMotor.setPower(0);
        }
    }

    /** Null while still driving the current leg; else a short string saying why it finished. */
    private String legDoneReason(Pose target, double timeout) {
        if (!follower.isBusy()) return "isBusy() false";

        Pose now = follower.getPose();
        double dist = Math.hypot(now.getX() - target.getX(), now.getY() - target.getY());
        double speed = follower.getVelocity().getMagnitude();
        if (dist <= END_DISTANCE_TOLERANCE_IN && speed <= END_VELOCITY_TOLERANCE) {
            return String.format("at endpoint (%.1f in, %.1f in/s)", dist, speed);
        }

        if (legTimer.getElapsedTimeSeconds() > timeout) {
            return String.format("timeout %.1fs", timeout);
        }
        return null;
    }

    // ===== planning (only ever runs while idle or right after DONE) =====

    private void rebuildIfConfigChanged() {
        double sig = configSignature();
        if (sig != lastConfigSignature) {
            startPose = FieldVisualizer.getRobotPose();
            follower.setStartingPose(startPose);
            rebuildPlan(startPose);
            lastConfigSignature = sig;
        }
    }

    private double configSignature() {
        double sig = 17;
        for (Ball b : FieldVisualizer.getBalls()) sig = sig * 31 + b.x + b.y;
        for (Obstacle o : FieldVisualizer.getObstacles()) sig = sig * 31 + o.x + o.y + o.radius;
        sig = sig * 31 + FieldVisualizer.robotX + FieldVisualizer.robotY
                + FieldVisualizer.robotHeadingDeg + FieldVisualizer.CLEARANCE_IN;
        return sig;
    }

    private void rebuildPlan(Pose from) {
        planReady = false;
        List<Obstacle> obstacles = FieldVisualizer.getObstacles();
        double clearance = FieldVisualizer.CLEARANCE_IN;
        Ball[] balls = FieldVisualizer.getBalls().toArray(new Ball[0]);

        // RouteOptimizer decides ball VISIT ORDER (and, as a side effect,
        // obstacle-avoiding waypoints for every leg - reused below by
        // buildIntakeCurve/buildReturnPath whenever the smooth curve/line
        // they'd otherwise drive clips an obstacle).
        RouteOptimizer.Route route =
                RouteOptimizer.findOptimalRoute(from, balls, from, obstacles, clearance);

        if (route == null) {
            lastRoute = null;
            intakePath = null;
            returnPath = null;
            intakeSamples = null;
            returnSamples = null;
            return;
        }

        lastRoute = route;
        buildIntakeCurve(from, route, obstacles, clearance);
        buildReturnPath(from, route, obstacles, clearance);
        planReady = (intakePath != null && returnPath != null);
    }

    /**
     * PATH 1, normal case: a curve through only the ESSENTIAL stops (see
     * essentialIntakeStops) - up to a cubic BezierCurve [start, c1, c2, end]
     * when all 3 balls are essential, collapsing to a simpler curve/line
     * when the wide intake sweeps some of them up for free. `end` is always
     * the last ball in visit order (touched exactly, since it IS the
     * curve's endpoint). When 2 interior balls ARE essential, c1/c2 are
     * solved so the curve passes exactly through both at t = INTAKE_T1/T2 -
     * see solveInteriorControls. Heading stays tangent to the curve
     * throughout.
     *
     * If that curve would actually clip an obstacle, it's discarded in
     * favour of ONE BIGGER SPLINE - a BezierCurve.through(...) solved so it
     * passes exactly through start, all 3 balls, and the detour points
     * RouteOptimizer already planned around the obstacle for this exact
     * ball order (route.legs[0..2]: Robot->ball0->ball1->ball2). This is
     * still a single smooth curve (heading stays tangent throughout), just
     * with more control points bending it around the obstacle. (Obstacle
     * avoidance always targets all 3 balls explicitly, regardless of the
     * intake-width shortcut above - safety takes priority over the extra
     * few inches saved by skipping a ball.)
     *
     * Only if that bigger spline still clips something (a tight obstacle
     * cluster the detour points can't fully solve around) do we fall back
     * to a guaranteed-safe multi-segment polyline through the same
     * waypoints - see buildAvoidanceChain.
     */
    private void buildIntakeCurve(Pose start, RouteOptimizer.Route route,
                                  List<Obstacle> obstacles, double clearance) {
        Ball[] order = route.order;
        Pose end = order[2].toPose();
        List<Pose> essentialStops = essentialIntakeStops(start, order);
        intakeEssentialStopCount = essentialStops.size();

        BezierCurve directCurve = buildThroughCurve(start, essentialStops);
        intakeRerouted = firstCollision(directCurve, obstacles, clearance) != null;
        intakeHardFallback = false;

        if (!intakeRerouted) {
            intakeEndPose = end;
            intakeEndHeading = directCurve.getEndTangent().getTheta();

            intakePath = follower.pathBuilder()
                    .addPath(directCurve)
                    .setTangentHeadingInterpolation()
                    .build();

            intakeSamples = new ArrayList<>();
            sampleCurveInto(intakeSamples, directCurve);
        } else {
            List<Pose> waypoints = new ArrayList<>();
            waypoints.add(start);
            for (int leg = 0; leg < 3; leg++) {
                List<Pose> legPts = route.legs.get(leg).waypoints;
                for (int j = 1; j < legPts.size(); j++) waypoints.add(legPts.get(j));
            }

            BezierCurve spline = BezierCurve.through(waypoints.toArray(new Pose[0]));
            intakeHardFallback = firstCollision(spline, obstacles, clearance) != null;

            if (!intakeHardFallback) {
                intakeEndPose = waypoints.get(waypoints.size() - 1);
                intakeEndHeading = spline.getEndTangent().getTheta();

                intakePath = follower.pathBuilder()
                        .addPath(spline)
                        .setTangentHeadingInterpolation()
                        .build();

                intakeSamples = new ArrayList<>();
                sampleCurveInto(intakeSamples, spline);
            } else {
                intakeEndPose = waypoints.get(waypoints.size() - 1);
                Pose last = waypoints.get(waypoints.size() - 1);
                Pose secondLast = waypoints.get(waypoints.size() - 2);
                intakeEndHeading = Math.atan2(last.getY() - secondLast.getY(), last.getX() - secondLast.getX());

                intakePath = buildAvoidanceChain(waypoints);
                intakeSamples = waypoints;
            }
        }

        double length = pathLength(intakeSamples);
        intakeTimeoutSec = Math.max(TIMEOUT_MIN_SEC, length / TIMEOUT_MIN_AVG_SPEED_IPS);
    }

    /**
     * Which balls the intake curve actually needs to touch, in visit order -
     * always ending at order[2] (the last ball is always essential, since
     * it's where the route ends). A ball is dropped from this list if
     * driving straight between two OTHER essential stops already sweeps it
     * into the intake (see coveredByIntake), since the full-width intake
     * collects it without the curve needing to bend there. E.g. two balls
     * sitting side-by-side within intake width of each other collapse to a
     * single stop, so the curve is a straight shot instead of a detour to
     * each ball's exact centre.
     *
     * Checked in order of biggest shortcut first: can we skip BOTH interior
     * balls (drive straight start -> last ball)? Then can we skip just the
     * first, or just the second? Only visits all 3 explicitly if none of
     * those shortcuts hold.
     */
    private static List<Pose> essentialIntakeStops(Pose start, Ball[] order) {
        Pose o0 = order[0].toPose();
        Pose o1 = order[1].toPose();
        Pose o2 = order[2].toPose();
        double intakeWidth = FieldVisualizer.INTAKE_WIDTH_IN;

        if (coveredByIntake(start, o2, o0, intakeWidth) && coveredByIntake(start, o2, o1, intakeWidth)) {
            return List.of(o2);
        }
        if (coveredByIntake(start, o1, o0, intakeWidth)) {
            return List.of(o1, o2);
        }
        if (coveredByIntake(o0, o2, o1, intakeWidth)) {
            return List.of(o0, o2);
        }
        return List.of(o0, o1, o2);
    }

    /**
     * True if driving straight from a to b would sweep {@code ball} into the
     * intake: its perpendicular distance from the segment is within half the
     * intake width, AND its projection actually lands on the segment (not
     * off one end, where the intake wouldn't reach it yet/anymore).
     */
    private static boolean coveredByIntake(Pose a, Pose b, Pose ball, double intakeWidth) {
        double dx = b.getX() - a.getX();
        double dy = b.getY() - a.getY();
        double lenSq = dx * dx + dy * dy;
        if (lenSq < 1e-9) return false;

        double t = ((ball.getX() - a.getX()) * dx + (ball.getY() - a.getY()) * dy) / lenSq;
        if (t < 0 || t > 1) return false;

        double projX = a.getX() + t * dx;
        double projY = a.getY() + t * dy;
        return Math.hypot(ball.getX() - projX, ball.getY() - projY) <= intakeWidth / 2.0;
    }

    /**
     * Builds the smallest curve that passes exactly through start and every
     * essential stop: a straight BezierLine for 1 stop, an exact-through
     * quadratic for 2, or the solved cubic (through both interior balls at
     * INTAKE_T1/T2) for all 3.
     */
    private BezierCurve buildThroughCurve(Pose start, List<Pose> essentialStops) {
        if (essentialStops.size() == 1) {
            return new BezierLine(start, essentialStops.get(0));
        }
        if (essentialStops.size() == 2) {
            return BezierCurve.through(start, essentialStops.get(0), essentialStops.get(1));
        }
        Pose ballA = essentialStops.get(0);
        Pose ballB = essentialStops.get(1);
        Pose end = essentialStops.get(2);
        Pose[] controls = solveInteriorControls(start, end, ballA, ballB, INTAKE_T1, INTAKE_T2);
        return new BezierCurve(start, controls[0], controls[1], end);
    }

    /**
     * Solves for the 2 middle control points of a cubic Bezier so it passes
     * exactly through ballA/ballB at the given t values, with fixed start
     * (t=0) and end (t=1). Two independent 2x2 linear solves (X and Y),
     * using the cubic Bernstein basis:
     *   C(t) = (1-t)^3 P0 + 3(1-t)^2 t P1 + 3(1-t) t^2 P2 + t^3 P3
     */
    private static Pose[] solveInteriorControls(Pose start, Pose end, Pose ballA, Pose ballB,
                                                double t1, double t2) {
        double omt1 = 1 - t1, omt2 = 1 - t2;
        double a1 = 3 * omt1 * omt1 * t1, b1 = 3 * omt1 * t1 * t1; // coeffs of P1, P2 at t1
        double a2 = 3 * omt2 * omt2 * t2, b2 = 3 * omt2 * t2 * t2; // coeffs of P1, P2 at t2

        double rhs1x = ballA.getX() - omt1 * omt1 * omt1 * start.getX() - t1 * t1 * t1 * end.getX();
        double rhs1y = ballA.getY() - omt1 * omt1 * omt1 * start.getY() - t1 * t1 * t1 * end.getY();
        double rhs2x = ballB.getX() - omt2 * omt2 * omt2 * start.getX() - t2 * t2 * t2 * end.getX();
        double rhs2y = ballB.getY() - omt2 * omt2 * omt2 * start.getY() - t2 * t2 * t2 * end.getY();

        double det = a1 * b2 - a2 * b1;
        // det shouldn't be ~0 for distinct t1/t2 in (0,1); fall back to the
        // raw ball positions if it somehow is, rather than dividing by ~0.
        if (Math.abs(det) < 1e-9) {
            return new Pose[]{ballA, ballB};
        }

        double p1x = (rhs1x * b2 - rhs2x * b1) / det;
        double p1y = (rhs1y * b2 - rhs2y * b1) / det;
        double p2x = (a1 * rhs2x - a2 * rhs1x) / det;
        double p2y = (a1 * rhs2y - a2 * rhs1y) / det;

        return new Pose[]{new Pose(p1x, p1y, 0), new Pose(p2x, p2y, 0)};
    }

    /**
     * PATH 2, normal case: a straight line from the intake curve's end back
     * to the original start pose. Heading turns (linearly) from wherever the
     * robot actually ends up facing after path 1 (intakeEndHeading) back to
     * the ORIGINAL start heading, so the robot returns to the exact position
     * AND heading it started at.
     *
     * If that line would clip an obstacle, it's replaced with ONE BIGGER
     * SPLINE - a BezierCurve.through(...) bent through the detour points
     * RouteOptimizer already planned for this leg (route.legs[3]: last
     * ball -> return) - still one smooth curve, still landing on the
     * original start heading (linear heading interpolation spans the whole
     * curve regardless of its shape).
     *
     * Only if that spline still clips something does this fall back to a
     * guaranteed-safe multi-segment polyline through the same waypoints.
     */
    private void buildReturnPath(Pose originalStart, RouteOptimizer.Route route,
                                 List<Obstacle> obstacles, double clearance) {
        Pose[] linePoints = {intakeEndPose, originalStart};
        returnRerouted = firstCollision(linePoints, obstacles, clearance) != null;
        returnHardFallback = false;

        if (!returnRerouted) {
            returnPath = follower.pathBuilder()
                    .addPath(new BezierLine(intakeEndPose, originalStart))
                    .setLinearHeadingInterpolation(intakeEndHeading, originalStart.getHeading())
                    .build();

            returnSamples = new ArrayList<>();
            sampleCurveInto(returnSamples, linePoints);
        } else {
            List<Pose> waypoints = new ArrayList<>();
            waypoints.add(intakeEndPose);
            List<Pose> legPts = route.legs.get(3).waypoints;
            for (int j = 1; j < legPts.size(); j++) waypoints.add(legPts.get(j));

            BezierCurve spline = BezierCurve.through(waypoints.toArray(new Pose[0]));
            returnHardFallback = firstCollision(spline, obstacles, clearance) != null;

            if (!returnHardFallback) {
                returnPath = follower.pathBuilder()
                        .addPath(spline)
                        .setLinearHeadingInterpolation(intakeEndHeading, originalStart.getHeading())
                        .build();

                returnSamples = new ArrayList<>();
                sampleCurveInto(returnSamples, spline);
            } else {
                returnPath = buildAvoidanceChainToHeading(waypoints, originalStart.getHeading());
                returnSamples = waypoints;
            }
        }

        double length = pathLength(returnSamples);
        returnTimeoutSec = Math.max(TIMEOUT_MIN_SEC, length / TIMEOUT_MIN_AVG_SPEED_IPS);
    }

    // ===== obstacle-avoidance chain builders =====

    /** Chains BezierLines through every waypoint, heading tangent to each segment throughout. */
    private PathChain buildAvoidanceChain(List<Pose> waypoints) {
        PathBuilder builder = follower.pathBuilder();
        for (int i = 0; i < waypoints.size() - 1; i++) {
            builder.addPath(new BezierLine(waypoints.get(i), waypoints.get(i + 1)))
                    .setTangentHeadingInterpolation();
        }
        return builder.build();
    }

    /** Same as {@link #buildAvoidanceChain}, but the final segment turns (linearly) into {@code finalHeading}. */
    private PathChain buildAvoidanceChainToHeading(List<Pose> waypoints, double finalHeading) {
        PathBuilder builder = follower.pathBuilder();
        for (int i = 0; i < waypoints.size() - 1; i++) {
            Pose a = waypoints.get(i);
            Pose b = waypoints.get(i + 1);
            builder.addPath(new BezierLine(a, b));
            if (i == waypoints.size() - 2) {
                double fromHeading = Math.atan2(b.getY() - a.getY(), b.getX() - a.getX());
                builder.setLinearHeadingInterpolation(fromHeading, finalHeading);
            } else {
                builder.setTangentHeadingInterpolation();
            }
        }
        return builder.build();
    }

    // ===== shared curve/geometry helpers =====

    /** First obstacle the actual BezierCurve (sampled via its own getPose) passes within `clearance` of, or null. */
    private static Obstacle firstCollision(BezierCurve curve, List<Obstacle> obstacles, double clearance) {
        for (int s = 0; s <= CURVE_CHECK_SAMPLES; s++) {
            Pose p = curve.getPose((double) s / CURVE_CHECK_SAMPLES);
            for (Obstacle o : obstacles) {
                if (Math.hypot(p.getX() - o.x, p.getY() - o.y) <= o.radius + clearance) return o;
            }
        }
        return null;
    }

    /** First obstacle the sampled curve/line passes within `clearance` of, or null. */
    private static Obstacle firstCollision(Pose[] points, List<Obstacle> obstacles, double clearance) {
        double[] px = new double[points.length];
        double[] py = new double[points.length];
        for (int i = 0; i < points.length; i++) { px[i] = points[i].getX(); py[i] = points[i].getY(); }

        for (int s = 0; s <= CURVE_CHECK_SAMPLES; s++) {
            double t = (double) s / CURVE_CHECK_SAMPLES;
            double bx = bezierPoint(px, t), by = bezierPoint(py, t);
            for (Obstacle o : obstacles) {
                if (Math.hypot(bx - o.x, by - o.y) <= o.radius + clearance) return o;
            }
        }
        return null;
    }

    private static double bezierPoint(double[] pts, double t) {
        double[] tmp = pts.clone();
        for (int level = tmp.length - 1; level > 0; level--) {
            for (int i = 0; i < level; i++) tmp[i] = (1 - t) * tmp[i] + t * tmp[i + 1];
        }
        return tmp[0];
    }

    /** Dense samples along the actual driven BezierCurve (via its own getPose), for drawing/length. */
    private static void sampleCurveInto(List<Pose> out, BezierCurve curve) {
        int samples = 60;
        for (int s = 0; s <= samples; s++) {
            out.add(curve.getPose((double) s / samples));
        }
    }

    private static void sampleCurveInto(List<Pose> out, Pose[] curve) {
        double[] px = new double[curve.length];
        double[] py = new double[curve.length];
        for (int i = 0; i < curve.length; i++) { px[i] = curve[i].getX(); py[i] = curve[i].getY(); }

        int samples = 60;
        out.add(curve[0]);
        for (int s = 1; s <= samples; s++) {
            double t = (double) s / samples;
            out.add(new Pose(bezierPoint(px, t), bezierPoint(py, t), 0));
        }
    }

    private static double pathLength(List<Pose> samples) {
        double length = 0;
        for (int i = 0; i < samples.size() - 1; i++) {
            Pose a = samples.get(i), b = samples.get(i + 1);
            length += Math.hypot(b.getX() - a.getX(), b.getY() - a.getY());
        }
        return length;
    }

    // ===== Dashboard / telemetry =====

    private void drawDashboard(Pose displayPose) {
        TelemetryPacket packet = new TelemetryPacket();
        boolean showSeedRobot = isIdle();
        FieldVisualizer.draw(packet, showSeedRobot);

        // Both paths, plus a marker at path 1's endpoint (the last ball).
        FieldVisualizer.drawPlannedPath(packet.fieldOverlay(), intakeSamples,
                intakeEndPose == null ? null : List.of(intakeEndPose));
        FieldVisualizer.drawPlannedPath(packet.fieldOverlay(), returnSamples, null);

        if (!showSeedRobot) {
            FieldVisualizer.drawLiveRobotOnCanvas(packet.fieldOverlay(), displayPose);
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    private void showTelemetry(Pose pose) {
        String state = isIdle() ? (planReady ? "IDLE (paths ready)" : "IDLE (no valid path)")
                : pathState == 0 ? "INTAKE CURVE"
                : pathState == 1 ? "RETURNING"
                : "DONE";

        telemetry.addData("State", state);
        telemetry.addData("X", String.format("%.1f in", pose.getX()));
        telemetry.addData("Y", String.format("%.1f in", pose.getY()));
        telemetry.addData("Heading", String.format("%.1f deg", Math.toDegrees(pose.getHeading())));

        if (lastRoute != null) {
            telemetry.addLine("--- Ball Order ---");
            for (int i = 0; i < lastRoute.order.length; i++) {
                telemetry.addData("  Ball " + (i + 1), lastRoute.order[i].toString());
            }
            if (!lastEndReason.isEmpty()) {
                telemetry.addData("Last leg ended by", lastEndReason);
            }
            if (intakeEssentialStopCount < 3) {
                telemetry.addLine("Intake shortcut: only " + intakeEssentialStopCount
                        + "/3 balls needed as explicit stops (intake width sweeps the rest).");
            }
            if (intakeRerouted) {
                telemetry.addLine(intakeHardFallback
                        ? "!! Intake path hard-fallback to straight-line legs (obstacle cluster too tight for a spline)."
                        : "~ Intake path bent into a bigger spline around an obstacle.");
            }
            if (returnRerouted) {
                telemetry.addLine(returnHardFallback
                        ? "!! Return path hard-fallback to straight-line legs (obstacle cluster too tight for a spline)."
                        : "~ Return path bent into a bigger spline around an obstacle.");
            }
        }

        if (isIdle()) {
            telemetry.addLine(planReady ? ">> Set run=1 to drive intake curve, then return."
                    : ">> No valid path - adjust obstacles/balls.");
        } else if (isDone()) {
            telemetry.addLine(">> Stopped. Drop run to 0 then back to 1 to run again.");
        }
        telemetry.update();
    }
}