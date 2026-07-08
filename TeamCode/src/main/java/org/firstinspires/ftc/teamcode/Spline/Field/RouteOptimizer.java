package org.firstinspires.ftc.teamcode.Spline.Field;

import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Finds the best ORDER to visit the balls — not just the best path between
 * two fixed points, which is VisibilityGraphPlanner's job.
 *
 * With only 3 balls there are 3! = 6 possible visit orders, so this
 * brute-forces all of them, plans an obstacle-avoiding
 * VisibilityGraphPlanner route for every leg of every order, and keeps
 * whichever full route (Robot -> ball -> ball -> ball -> Robot) has the
 * shortest total obstacle-avoiding distance.
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
        /** Total obstacle-avoiding distance across every leg, inches. */
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
            double total = 0;
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
                total += leg.length;
                from = leg.waypoints.get(leg.waypoints.size() - 1); // arrival pose (with heading)
            }
            if (!feasible) continue;

            VisibilityGraphPlanner.PlanResult lastLeg =
                    VisibilityGraphPlanner.planPath(from, returnPose, obstacles, clearance);
            if (lastLeg == null) continue;
            legs.add(lastLeg);
            total += lastLeg.length;

            if (total < bestLength) {
                bestLength = total;
                bestOrder = order;
                bestLegs = legs;
            }
        }

        if (bestOrder == null) return null;
        return new Route(bestOrder, bestLegs, bestLength);
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