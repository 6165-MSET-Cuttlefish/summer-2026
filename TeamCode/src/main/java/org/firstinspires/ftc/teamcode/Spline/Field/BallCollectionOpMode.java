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

import org.firstinspires.ftc.teamcode.Spline.Pedro.PedroSetup;

import java.util.ArrayList;
import java.util.List;

/**
 * BallCollectionOpMode — pre-built PathChains, zero planning after start.
 * -----------------------------------------------------------------------------
 * LIFECYCLE (this is the key structural guarantee):
 *
 *   WHILE IDLE  — the route is planned and ALL PathChains are FULLY BUILT
 *                 ahead of time, from the current Dashboard positions. The
 *                 finished spline is drawn on the Field view so you can see
 *                 exactly what will run BEFORE pressing A. Rebuilding only
 *                 happens when a Dashboard value actually changes (robot,
 *                 ball, or obstacle moved) — not every loop.
 *
 *   PRESS A     — does NOTHING except follower.followPath(legChains.get(0)).
 *                 No planning, no searching, no path construction. The
 *                 chains being followed are the exact frozen objects built
 *                 while idle.
 *
 *   WHILE       — the only per-loop calls are follower.update() and the
 *   RUNNING       pathState FSM that starts the NEXT pre-built chain when
 *                 the current one finishes (!isBusy(), standard Pedro auto
 *                 style). The paths are never modified, replanned, or
 *                 regenerated after A. Guaranteed.
 *
 * PATH STRUCTURE — exactly the Pedro auto pattern:
 *   Each leg is ONE path in ONE PathChain:
 *     BezierCurve( startPoint, controlPoint(s)..., endPoint )
 *   or BezierLine( startPoint, endPoint ) when the leg is a clear straight
 *   shot. The control points are the obstacle detour points found by the
 *   visibility-graph planner AT BUILD TIME (while idle), then frozen.
 *     legChains[0]  Robot  → Ball A
 *     legChains[1]  Ball A → Ball B
 *     legChains[2]  Ball B → Ball C
 *     legChains[3]  Ball C → return (start pose)
 *   Ball order is chosen at build time over all 3! = 6 possibilities.
 *
 * DASHBOARD:
 *   IDLE:           BLUE robot square (drag it — the route rebuilds and the
 *                   blue spline preview updates), spline + anchor circles.
 *   RUNNING / DONE: GREEN live robot square only; the same frozen spline
 *                   stays drawn so you can watch the robot trace it.
 *
 * STOP GUARANTEE:
 *   Per-leg completion = !isBusy() OR at-endpoint-and-stopped OR per-leg
 *   timeout. After the last leg: follower.breakFollowing() (all drive
 *   output zeroed) and update() stops being called. Dead stop.
 */
@Config
@TeleOp(name = "Ball Collection", group = "Spline")
public class BallCollectionOpMode extends OpMode {

    // ── Completion tuning (dashboard-editable) ────────────────────────────────
    public static double LEG_TIMEOUT_MIN_SEC        = 4.0;
    public static double TIMEOUT_MIN_AVG_SPEED_IPS  = 10.0;
    public static double END_DISTANCE_TOLERANCE_IN  = 2.0;
    public static double END_VELOCITY_TOLERANCE     = 1.0;

    // ── Spline safety tuning ──────────────────────────────────────────────────
    public static int    CURVE_CHECK_SAMPLES     = 60;
    public static int    CURVE_REPAIR_ITERATIONS = 10;
    public static double CURVE_REPAIR_STEP_IN    = 3.0;

    /** Legs shorter than this are skipped as already-complete (avoids NaN). */
    public static double MIN_LEG_LENGTH_IN = 1.0;

    // ── Pedro Pathing ─────────────────────────────────────────────────────────
    private Follower follower;
    private final Timer pathTimer = new Timer();

    // ── FSM: -1 idle · 0..n-1 following legChains[pathState] · n done ────────
    private int pathState = -1;

    // Pre-built while IDLE; frozen the moment A is pressed.
    private final List<PathChain> legChains   = new ArrayList<>();
    private final List<Pose>      legEndPoses = new ArrayList<>();
    private final List<Double>    legTimeouts = new ArrayList<>();

    private Pose startPose;
    private boolean planReady = false;

    // Change detection: rebuild only when Dashboard values actually change.
    private double lastConfigSignature = Double.NaN;

    // Telemetry / drawing
    private RouteOptimizer.Route lastRoute;
    private List<Pose> splineSamples;
    private List<Pose> legAnchors;
    private double lastPlanningTimeMs;
    private String lastLegEndReason = "";
    private int legsFallenBackToLines = 0;

    private boolean prevA = false;

    private boolean isIdle()    { return pathState == -1; }
    private boolean isDone()    { return !legChains.isEmpty() && pathState >= legChains.size(); }
    private boolean isRunning() { return pathState >= 0 && pathState < legChains.size(); }

    // =========================================================================
    //  init / start
    // =========================================================================
    @Override
    public void init() {
        follower = PedroSetup.createFollower(hardwareMap);
        startPose = FieldVisualizer.getRobotPose();
        follower.setStartingPose(startPose);

        // Build the full route + PathChains right now, before A is ever
        // pressed. init_loop/loop keep it in sync with Dashboard edits.
        rebuildPlan(startPose);

        telemetry.addLine("Path pre-built. Drag things on the Dashboard to update it,");
        telemetry.addLine("then press A on gamepad 1 to run the pre-built path.");
        telemetry.update();
    }

    /** Keep the pre-built plan synced with Dashboard edits during INIT too. */
    @Override
    public void init_loop() {
        rebuildIfConfigChanged();
        drawDashboard(startPose);
    }

    /** Re-apply seed pose after the Pinpoint's ~0.25s calibration window. */
    @Override
    public void start() {
        follower.setStartingPose(startPose);
    }

    // =========================================================================
    //  loop
    // =========================================================================
    @Override
    public void loop() {

        // While idle, keep the pre-built plan in sync with Dashboard edits.
        // While running/done: NEVER rebuild — the chains are frozen.
        if (isIdle()) {
            rebuildIfConfigChanged();
        }

        boolean currA = gamepad1.a;
        if (currA && !prevA) {
            startPrebuiltPath();
        }
        prevA = currA;

        if (isRunning()) {
            follower.update();
            autonomousPathUpdate();
        }

        Pose displayPose = isIdle() ? startPose : follower.getPose();
        drawDashboard(displayPose);
        driverHubTelemetry(displayPose);
    }

    // =========================================================================
    //  startPrebuiltPath — the A press. Follows the already-built chain.
    //  NO planning happens here.
    // =========================================================================
    private void startPrebuiltPath() {
        if (isIdle()) {
            if (!planReady) return; // nothing valid to run
            // The seed pose the plan was built from is the robot's position.
            follower.setStartingPose(startPose);
        } else if (isDone()) {
            // Re-running after a finished route: rebuild ONCE from wherever
            // the robot actually ended up, then run those new chains. This
            // is the only rebuild outside IDLE, and it happens strictly
            // before following starts — never mid-drive.
            startPose = follower.getPose();
            rebuildPlan(startPose);
            if (!planReady) return;
        } else {
            return; // already running — ignore A
        }

        pathState = 0;
        follower.followPath(legChains.get(0), false);
        pathTimer.resetTimer();
        lastLegEndReason = "";
    }

    // =========================================================================
    //  autonomousPathUpdate — standard Pedro FSM over the pre-built chains.
    // =========================================================================
    private void autonomousPathUpdate() {
        String done = legComplete();
        if (done == null) return;

        lastLegEndReason = done;
        pathState++;

        if (pathState < legChains.size()) {
            follower.followPath(legChains.get(pathState), false);
            pathTimer.resetTimer();
        } else {
            follower.breakFollowing(); // zero all drive output — dead stop
        }
    }

    /** Reason string if the CURRENT leg is complete, else null. */
    private String legComplete() {
        if (!follower.isBusy()) {
            return "isBusy() false";
        }

        Pose end = legEndPoses.get(pathState);
        Pose now = follower.getPose();
        double distToEnd = Math.hypot(now.getX() - end.getX(), now.getY() - end.getY());
        double speed = follower.getVelocity().getMagnitude();
        if (distToEnd <= END_DISTANCE_TOLERANCE_IN && speed <= END_VELOCITY_TOLERANCE) {
            return String.format("at endpoint (%.1f in, %.1f in/s)", distToEnd, speed);
        }

        double timeout = legTimeouts.get(pathState);
        if (pathTimer.getElapsedTimeSeconds() > timeout) {
            return String.format("LEG TIMEOUT %.1fs", timeout);
        }

        return null;
    }

    // =========================================================================
    //  Change detection — rebuild the plan only when a Dashboard value that
    //  affects it actually changes. Cheap signature over every input.
    // =========================================================================
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
        for (Ball b : FieldVisualizer.getBalls()) {
            sig = sig * 31 + b.x;
            sig = sig * 31 + b.y;
        }
        for (Obstacle o : FieldVisualizer.getObstacles()) {
            sig = sig * 31 + o.x;
            sig = sig * 31 + o.y;
            sig = sig * 31 + o.radius;
        }
        sig = sig * 31 + FieldVisualizer.robotX;
        sig = sig * 31 + FieldVisualizer.robotY;
        sig = sig * 31 + FieldVisualizer.robotHeadingDeg;
        sig = sig * 31 + FieldVisualizer.CLEARANCE_IN;
        return sig;
    }

    // =========================================================================
    //  rebuildPlan — plan the route AND fully build every PathChain, from
    //  the given start pose. Only ever called while NOT driving.
    // =========================================================================
    private void rebuildPlan(Pose from) {
        long t0 = System.nanoTime();
        planReady = false;

        List<Obstacle> obstacles = FieldVisualizer.getObstacles();
        double clearance = FieldVisualizer.CLEARANCE_IN;
        Ball[] balls = FieldVisualizer.getBalls().toArray(new Ball[0]);

        RouteOptimizer.Route route =
                RouteOptimizer.findOptimalRoute(from, balls, from, obstacles, clearance);

        if (route == null) {
            lastRoute = null;
            splineSamples = null;
            legAnchors = null;
            legChains.clear();
            lastPlanningTimeMs = (System.nanoTime() - t0) / 1_000_000.0;
            return;
        }
        lastRoute = route;

        buildPaths(from, route, obstacles, clearance);
        planReady = !legChains.isEmpty();

        lastPlanningTimeMs = (System.nanoTime() - t0) / 1_000_000.0;
    }

    // =========================================================================
    //  buildPaths — one PathChain per leg; each chain is a single path with
    //  a start point, an end point, and the detour control points between.
    // =========================================================================
    private void buildPaths(Pose robotNow, RouteOptimizer.Route route,
                            List<Obstacle> obstacles, double clearance) {
        legChains.clear();
        legEndPoses.clear();
        legTimeouts.clear();
        legsFallenBackToLines = 0;

        splineSamples = new ArrayList<>();
        legAnchors = new ArrayList<>();
        splineSamples.add(robotNow);

        Pose legStart = robotNow;

        for (VisibilityGraphPlanner.PlanResult leg : route.legs) {
            List<Pose> wp = leg.waypoints;
            Pose legEnd = wp.get(wp.size() - 1);

            // Skip degenerate legs (coincident balls) — a zero-length
            // BezierLine(p, p) NaNs Pedro's parameterization.
            if (Math.hypot(legEnd.getX() - legStart.getX(),
                    legEnd.getY() - legStart.getY()) < MIN_LEG_LENGTH_IN) {
                continue;
            }

            double startHeading = legStart.getHeading();
            double endHeading   = legEnd.getHeading();

            PathBuilder builder = follower.pathBuilder();

            if (wp.size() <= 2) {
                // Start point + end point, no control points needed.
                builder.addPath(new BezierLine(legStart, legEnd))
                        .setLinearHeadingInterpolation(startHeading, endHeading);
                splineSamples.add(legEnd);
            } else {
                // Start point + control points + end point, one BezierCurve.
                List<Pose> controls = new ArrayList<>(wp.subList(1, wp.size() - 1));
                boolean safe = repairCurve(legStart, controls, legEnd, obstacles, clearance);

                if (safe) {
                    Pose[] curvePoses = new Pose[controls.size() + 2];
                    curvePoses[0] = legStart;
                    for (int i = 0; i < controls.size(); i++) curvePoses[i + 1] = controls.get(i);
                    curvePoses[curvePoses.length - 1] = legEnd;

                    builder.addPath(new BezierCurve(curvePoses))
                            .setLinearHeadingInterpolation(startHeading, endHeading);
                    sampleCurveInto(splineSamples, curvePoses);
                } else {
                    // Unrepairable curve — known-safe polyline fallback for
                    // this one leg (still a single pre-built PathChain).
                    legsFallenBackToLines++;
                    Pose prev = legStart;
                    for (int i = 1; i < wp.size(); i++) {
                        Pose next = wp.get(i);
                        builder.addPath(new BezierLine(prev, next))
                                .setLinearHeadingInterpolation(prev.getHeading(), next.getHeading());
                        splineSamples.add(next);
                        prev = next;
                    }
                }
            }

            legChains.add(builder.build());
            legEndPoses.add(legEnd);
            legTimeouts.add(Math.max(LEG_TIMEOUT_MIN_SEC,
                    leg.length / TIMEOUT_MIN_AVG_SPEED_IPS));
            legAnchors.add(legEnd);

            legStart = legEnd;
        }
    }

    // =========================================================================
    //  Curve safety (runs at BUILD time only, never while driving)
    // =========================================================================
    private boolean repairCurve(Pose start, List<Pose> controls, Pose end,
                                List<Obstacle> obstacles, double clearance) {
        for (int iter = 0; iter <= CURVE_REPAIR_ITERATIONS; iter++) {
            Obstacle clipped = firstCurveCollision(start, controls, end, obstacles, clearance);
            if (clipped == null) return true;
            if (iter == CURVE_REPAIR_ITERATIONS) break;

            for (int i = 0; i < controls.size(); i++) {
                Pose c = controls.get(i);
                double dx = c.getX() - clipped.x;
                double dy = c.getY() - clipped.y;
                double d = Math.hypot(dx, dy);
                double ux, uy;
                if (d < 1e-6) { ux = 1; uy = 0; } else { ux = dx / d; uy = dy / d; }
                double nx = clamp(c.getX() + ux * CURVE_REPAIR_STEP_IN, 0, 144);
                double ny = clamp(c.getY() + uy * CURVE_REPAIR_STEP_IN, 0, 144);
                controls.set(i, new Pose(nx, ny, c.getHeading()));
            }
        }
        return false;
    }

    private Obstacle firstCurveCollision(Pose start, List<Pose> controls, Pose end,
                                         List<Obstacle> obstacles, double clearance) {
        int nPts = controls.size() + 2;
        double[] px = new double[nPts];
        double[] py = new double[nPts];
        px[0] = start.getX();  py[0] = start.getY();
        for (int i = 0; i < controls.size(); i++) {
            px[i + 1] = controls.get(i).getX();
            py[i + 1] = controls.get(i).getY();
        }
        px[nPts - 1] = end.getX();  py[nPts - 1] = end.getY();

        for (int s = 0; s <= CURVE_CHECK_SAMPLES; s++) {
            double t = (double) s / CURVE_CHECK_SAMPLES;
            double bx = bezierPoint(px, t);
            double by = bezierPoint(py, t);
            for (Obstacle o : obstacles) {
                double d = Math.hypot(bx - o.x, by - o.y);
                if (d <= o.radius + clearance) return o;
            }
        }
        return null;
    }

    private static double bezierPoint(double[] pts, double t) {
        double[] tmp = pts.clone();
        for (int level = tmp.length - 1; level > 0; level--) {
            for (int i = 0; i < level; i++) {
                tmp[i] = (1 - t) * tmp[i] + t * tmp[i + 1];
            }
        }
        return tmp[0];
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void sampleCurveInto(List<Pose> out, Pose[] curvePoses) {
        double[] px = new double[curvePoses.length];
        double[] py = new double[curvePoses.length];
        for (int i = 0; i < curvePoses.length; i++) {
            px[i] = curvePoses[i].getX();
            py[i] = curvePoses[i].getY();
        }
        int samples = 20;
        for (int s = 1; s <= samples; s++) {
            double t = (double) s / samples;
            out.add(new Pose(bezierPoint(px, t), bezierPoint(py, t), 0));
        }
    }

    // =========================================================================
    //  Dashboard + Driver Hub output
    // =========================================================================
    private void drawDashboard(Pose displayPose) {
        TelemetryPacket packet = new TelemetryPacket();
        boolean showSeedRobot = isIdle();
        FieldVisualizer.draw(packet, showSeedRobot);
        FieldVisualizer.drawPlannedPath(packet.fieldOverlay(), splineSamples, legAnchors);
        if (!showSeedRobot) {
            FieldVisualizer.drawLiveRobotOnCanvas(packet.fieldOverlay(), displayPose);
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    private void driverHubTelemetry(Pose displayPose) {
        String stateLabel = isIdle() ? (planReady ? "IDLE (path pre-built)" : "IDLE (no valid path)")
                : isDone() ? "DONE"
                : ("LEG " + (pathState + 1) + "/" + legChains.size());
        telemetry.addData("State",   stateLabel);
        telemetry.addData("X",       String.format("%.1f in", displayPose.getX()));
        telemetry.addData("Y",       String.format("%.1f in", displayPose.getY()));
        telemetry.addData("Heading", String.format("%.1f deg", Math.toDegrees(displayPose.getHeading())));
        if (lastRoute != null) {
            telemetry.addLine("--- Pre-built Collection Order ---");
            for (int i = 0; i < lastRoute.order.length; i++) {
                telemetry.addData("  Pick " + (i + 1), lastRoute.order[i].toString());
            }
            telemetry.addData("Total planned distance", String.format("%.1f in", lastRoute.totalLength));
            telemetry.addData("Build time", String.format("%.1f ms", lastPlanningTimeMs));
            if (!lastLegEndReason.isEmpty()) {
                telemetry.addData("Last leg ended by", lastLegEndReason);
            }
            if (legsFallenBackToLines > 0) {
                telemetry.addData("Legs using line fallback", legsFallenBackToLines);
            }
        }
        if (isIdle()) {
            telemetry.addLine(planReady
                    ? ">> Path is pre-built (blue spline). Press A to run it."
                    : ">> No valid route for this layout — adjust obstacles/balls.");
        } else if (isDone()) {
            telemetry.addLine(">> STOPPED. Press A to rebuild from here and run again.");
        }
        telemetry.update();
    }
}