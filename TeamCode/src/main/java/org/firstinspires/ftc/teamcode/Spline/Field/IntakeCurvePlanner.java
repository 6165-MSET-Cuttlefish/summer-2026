package org.firstinspires.ftc.teamcode.Spline.Field;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import java.util.List;

/**
 * Shared logic for the intake curve that BallCollectionOpMode actually
 * drives - the curve through only the ESSENTIAL ball stops (see
 * essentialStops), solved with chord-length parameterization so interior
 * balls sit at t-values proportional to their actual spacing instead of
 * fixed thirds.
 *
 * Used by both BallCollectionOpMode (to build the real driven path) and
 * RouteOptimizer (to score candidate ball visit orders by the length of the
 * curve that would actually be driven, instead of the straight-line/
 * visibility-graph ball-to-ball distance).
 */
public class IntakeCurvePlanner {

    private static final int SAMPLES = 60;

    private IntakeCurvePlanner() {
        // static-only utility class
    }

    /**
     * Which balls the intake curve actually needs to touch, in visit order -
     * always ending at order[2] (the last ball is always essential, since
     * it's where the route ends). A ball is dropped from this list if
     * driving straight between two OTHER essential stops already sweeps it
     * into the intake (see coveredByIntake), since the full-width intake
     * collects it without the curve needing to bend there.
     *
     * Checked in order of biggest shortcut first: can we skip BOTH interior
     * balls (drive straight start -> last ball)? Then can we skip just the
     * first, or just the second? Only visits all 3 explicitly if none of
     * those shortcuts hold.
     */
    public static List<Pose> essentialStops(Pose start, Ball[] order) {
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
     * quadratic for 2, or a solved cubic (through both interior balls) for
     * all 3.
     *
     * For 3 stops, the two interior balls are pinned at t-values
     * proportional to their actual chord-length spacing
     * (start->ballA->ballB->end) rather than fixed thirds, so unevenly
     * spaced balls don't force the solved control points into a wider,
     * slower loop than the layout actually needs.
     */
    public static BezierCurve buildThroughCurve(Pose start, List<Pose> essentialStops) {
        if (essentialStops.size() == 1) {
            return new BezierLine(start, essentialStops.get(0));
        }
        if (essentialStops.size() == 2) {
            return BezierCurve.through(start, essentialStops.get(0), essentialStops.get(1));
        }

        Pose ballA = essentialStops.get(0);
        Pose ballB = essentialStops.get(1);
        Pose end = essentialStops.get(2);

        double d1 = dist(start, ballA);
        double d2 = dist(ballA, ballB);
        double d3 = dist(ballB, end);
        double total = d1 + d2 + d3;
        double t1 = total > 1e-9 ? d1 / total : 1.0 / 3.0;
        double t2 = total > 1e-9 ? (d1 + d2) / total : 2.0 / 3.0;
        // Keep both t-values well inside (0,1) and apart from each other so
        // solveInteriorControls's 2x2 solve doesn't blow up when a ball sits
        // almost on top of start/end/the other ball.
        t1 = clamp(t1, 0.15, 0.75);
        t2 = clamp(t2, t1 + 0.10, 0.90);

        Pose[] controls = solveInteriorControls(start, end, ballA, ballB, t1, t2);
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

    /** Total arc length of the curve, approximated by sampling it SAMPLES times. */
    public static double curveLength(BezierCurve curve) {
        double length = 0;
        Pose prev = curve.getPose(0);
        for (int s = 1; s <= SAMPLES; s++) {
            Pose p = curve.getPose((double) s / SAMPLES);
            length += dist(prev, p);
            prev = p;
        }
        return length;
    }

    /** True if the curve passes within {@code clearance} of any obstacle's edge, anywhere along it. */
    public static boolean collides(BezierCurve curve, List<Obstacle> obstacles, double clearance) {
        for (int s = 0; s <= SAMPLES; s++) {
            Pose p = curve.getPose((double) s / SAMPLES);
            for (Obstacle o : obstacles) {
                if (Math.hypot(p.getX() - o.x, p.getY() - o.y) <= o.radius + clearance) return true;
            }
        }
        return false;
    }

    private static double dist(Pose a, Pose b) {
        return Math.hypot(b.getX() - a.getX(), b.getY() - a.getY());
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
