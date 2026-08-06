package org.firstinspires.ftc.teamcode.Spline.Field;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Finds the best ORDER to visit the balls — not just the best path between
 * two fixed points, which is VisibilityGraphPlanner's job.
 *
 * With only 3 balls there are 3! = 6 possible visit orders, so this
 * brute-forces all of them and keeps whichever full route
 * (Robot -> ball -> ball -> ball -> Robot) has the shortest estimated DRIVE
 * length.
 *
 * Each order is scored per leg, using whichever length BallCollectionOpMode
 * would actually drive for that leg:
 *   - Intake leg (Robot -> ball -> ball -> ball): the essential-stop curve
 *     from IntakeCurvePlanner (same shortcut logic BallCollectionOpMode
 *     uses - a ball doesn't need an explicit curve stop if the wide intake
 *     already sweeps it up in passing), UNLESS that curve clips an
 *     obstacle, in which case BallCollectionOpMode falls back to the
 *     bigger obstacle-avoiding spline, so the VisibilityGraphPlanner sum
 *     (over all 3 balls) is used instead.
 *   - Return leg (last ball -> Robot): straight-line distance, unless that
 *     line is obstacle-blocked, in which case the VisibilityGraphPlanner
 *     leg length is used instead.
 * This keeps order-selection matched to the path that's actually driven,
 * rather than always assuming all 3 balls are visited as explicit points.
 *
 * The VisibilityGraphPlanner legs are still planned (and kept in the
 * returned Route) regardless, since BallCollectionOpMode needs their
 * detour waypoints whenever the direct curve/line does clip an obstacle.
 *
 * This is the "1-2-4-3 vs 3-2-1-4" ordering question from the project plan.
 * For 3 balls, exhaustive search is trivial (6 orders x 4 legs = 24 plans,
 * each over a graph of ~80 nodes — well under a millisecond total) and it
 * guarantees the true optimum, unlike a nearest-neighbour greedy pick, which
 * can lock in a bad first choice and end up with a visibly longer route.
 */
public class RouteOptimizer {

    /** A fully-planned route: the chosen ball order plus one PlanResult per leg. */
    public static class Route {
        /** Balls in the order they'll be visited. */
        public final Ball[] order;
        /** One planned leg per hop: Robot->order[0], order[0]->order[1], ..., order[last]->return. */
        public final List<VisibilityGraphPlanner.PlanResult> legs;
        /** Estimated total drive length across the whole route, inches (see class header). */
        public final double totalLength;

        Route(Ball[] order, List<VisibilityGraphPlanner.PlanResult> legs, double totalLength) {
            this.order = order;
            this.legs = legs;
            this.totalLength = totalLength;
        }
    }

    private RouteOptimizer() {
        // static-only utility class
    }

    /**
     * @param robotPose  robot's current pose (start of the route)
     * @param balls      balls to visit, in any order
     * @param returnPose pose to return to after the last ball (e.g. the
     *                   robot's original starting pose)
     * @return the lowest-total-distance Route, or null if no visit order is
     *         fully reachable (shouldn't happen on a normal field — it would
     *         mean some ball or the return point is fully walled off)
     */
    public static Route findOptimalRoute(Pose robotPose, Ball[] balls, Pose returnPose,
                                         List<Obstacle> obstacles, double clearance) {
        List<Ball> ballList = new ArrayList<>();
        Collections.addAll(ballList, balls);

        Ball[] bestOrder = null;
        List<VisibilityGraphPlanner.PlanResult> bestLegs = null;
        double bestLength = Double.MAX_VALUE;

        for (Ball[] order : permutations(ballList)) {
            List<VisibilityGraphPlanner.PlanResult> legs = new ArrayList<>();
            Pose from = robotPose;
            boolean feasible = true;

            for (Ball b : order) {
                VisibilityGraphPlanner.PlanResult leg =
                        VisibilityGraphPlanner.planPath(from, b.toPose(), obstacles, clearance);
                if (leg == null) {
                    feasible = false;
                    break;
                }
                legs.add(leg);
                from = leg.waypoints.get(leg.waypoints.size() - 1); // arrival pose (with heading)
            }
            if (!feasible) continue;

            VisibilityGraphPlanner.PlanResult lastLeg =
                    VisibilityGraphPlanner.planPath(from, returnPose, obstacles, clearance);
            if (lastLeg == null) continue;
            legs.add(lastLeg);

            double total = estimateDriveLength(robotPose, order, returnPose, obstacles, clearance, legs);

            if (total < bestLength) {
                bestLength = total;
                bestOrder = order;
                bestLegs = legs;
            }
        }

        if (bestOrder == null) return null;
        return new Route(bestOrder, bestLegs, bestLength);
    }

    /**
     * Estimates the length BallCollectionOpMode would actually drive for
     * this order: the essential-stop intake curve (or, if that curve clips
     * an obstacle, the VisibilityGraphPlanner sum over all 3 balls that
     * BallCollectionOpMode falls back to) plus the straight return distance
     * (or, if that line is blocked, the VisibilityGraphPlanner return leg).
     *
     * {@code legs} is Robot->ball0, ball0->ball1, ball1->ball2, ball2->return
     * - already planned by the caller for obstacle detection/detour
     * waypoints, and reused here as the fallback lengths.
     */
    private static double estimateDriveLength(Pose robotPose, Ball[] order, Pose returnPose,
                                              List<Obstacle> obstacles, double clearance,
                                              List<VisibilityGraphPlanner.PlanResult> legs) {
        List<Pose> essentialStops = IntakeCurvePlanner.essentialStops(robotPose, order);
        BezierCurve intakeCurve = IntakeCurvePlanner.buildThroughCurve(robotPose, essentialStops);

        double intakeCost = IntakeCurvePlanner.collides(intakeCurve, obstacles, clearance)
                ? legs.get(0).length + legs.get(1).length + legs.get(2).length
                : IntakeCurvePlanner.curveLength(intakeCurve);

        Pose intakeEnd = essentialStops.get(essentialStops.size() - 1); // always order[2]
        double returnCost = Obstacle.anyBlocks(obstacles,
                intakeEnd.getX(), intakeEnd.getY(), returnPose.getX(), returnPose.getY(), clearance)
                ? legs.get(3).length
                : Math.hypot(returnPose.getX() - intakeEnd.getX(), returnPose.getY() - intakeEnd.getY());

        return intakeCost + returnCost;
    }

    private static List<Ball[]> permutations(List<Ball> balls) {
        List<Ball[]> result = new ArrayList<>();
        permuteHelper(new ArrayList<>(balls), 0, result);
        return result;
    }

    private static void permuteHelper(List<Ball> balls, int k, List<Ball[]> result) {
        if (k == balls.size()) {
            result.add(balls.toArray(new Ball[0]));
            return;
        }
        for (int i = k; i < balls.size(); i++) {
            Collections.swap(balls, k, i);
            permuteHelper(balls, k + 1, result);
            Collections.swap(balls, k, i);
        }
    }
}