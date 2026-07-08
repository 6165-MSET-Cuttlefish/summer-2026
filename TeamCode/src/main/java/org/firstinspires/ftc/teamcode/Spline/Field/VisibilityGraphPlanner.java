package org.firstinspires.ftc.teamcode.Spline.Field;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.PriorityQueue;

/**
 * Obstacle-avoiding path planner.
 *
 * ALGORITHM
 * ---------
 * Classic visibility graph + Dijkstra, adapted for circular obstacles:
 *   1. Each obstacle is "inflated" by {@code clearance} (robot half-width) so
 *      a point is safe to drive through as long as it stays outside
 *      (obstacle.radius + clearance) of every obstacle's centre.
 *   2. We sample POINTS_PER_OBSTACLE points around the inflated boundary of
 *      each obstacle (plus a small extra buffer so the straight-line chord
 *      between two adjacent samples doesn't dip back inside the inflated
 *      circle — see BOUNDARY_MARGIN_IN). These samples, plus the start and
 *      goal points, are the nodes of the visibility graph.
 *   3. An edge connects two nodes if the straight segment between them does
 *      not pass through any obstacle's inflated circle (reuses
 *      Obstacle.anyBlocks(), the same check FieldVisualizer already uses to
 *      colour legs red/green).
 *   4. Dijkstra finds the shortest node-to-node path from start to goal.
 *
 * This isn't a "true" tangent-line visibility graph (which would compute
 * exact bitangent lines between circles), but with enough sample points it
 * gets close enough for an FTC field with a handful of obstacles, runs in
 * well under a millisecond, and is simple to reason about and tune from the
 * Dashboard.
 *
 * COORDINATES: Pedro Pathing convention — inches, [0, 144], (0,0) bottom-left.
 */
@Config
public class VisibilityGraphPlanner {

    /** How many points to sample around each obstacle's inflated boundary. */
    public static int POINTS_PER_OBSTACLE = 16;

    /**
     * Extra buffer (inches) added on top of (radius + clearance) when placing
     * boundary sample points. Needed because the straight-line chord between
     * two adjacent samples dips slightly inside the true circle (arc vs.
     * chord) — this margin keeps that chord outside the blocked radius.
     */
    public static double BOUNDARY_MARGIN_IN = 1.5;

    private VisibilityGraphPlanner() {
        // static-only utility class
    }

    /** Result of a single-leg plan: the waypoint sequence and its total length. */
    public static class PlanResult {
        public final List<Pose> waypoints;
        public final double length;

        PlanResult(List<Pose> waypoints, double length) {
            this.waypoints = waypoints;
            this.length = length;
        }
    }

    /**
     * Plans an obstacle-avoiding path from {@code start} to {@code goal}.
     *
     * @return a PlanResult whose waypoints list starts with {@code start}
     *         (heading preserved from the passed-in Pose) and ends at
     *         {@code goal}'s (x, y). Every point's heading (including the
     *         last one) points along the segment it's arriving on, so the
     *         list can be fed directly into consecutive BezierLine segments
     *         with matching heading interpolation. Returns null only if no
     *         line-of-sight route exists at all (shouldn't normally happen
     *         since obstacles are inflated, not solid, and the field is
     *         finite).
     */
    public static PlanResult planPath(Pose start, Pose goal, List<Obstacle> obstacles, double clearance) {
        List<double[]> nodes = new ArrayList<>(); // each entry: {x, y}
        final int START = 0;
        final int GOAL = 1;
        nodes.add(new double[]{start.getX(), start.getY()});
        nodes.add(new double[]{goal.getX(), goal.getY()});

        for (Obstacle o : obstacles) {
            double placementRadius = o.radius + clearance + BOUNDARY_MARGIN_IN;
            for (int i = 0; i < POINTS_PER_OBSTACLE; i++) {
                double angle = 2 * Math.PI * i / POINTS_PER_OBSTACLE;
                double px = o.x + placementRadius * Math.cos(angle);
                double py = o.y + placementRadius * Math.sin(angle);

                if (px < 0 || px > 144 || py < 0 || py > 144) continue; // stay on the field
                if (insideAnyOtherObstacle(px, py, obstacles, o, clearance)) continue;

                nodes.add(new double[]{px, py});
            }
        }

        int n = nodes.size();
        double[] dist = new double[n];
        int[] prev = new int[n];
        boolean[] visited = new boolean[n];
        Arrays.fill(dist, Double.MAX_VALUE);
        Arrays.fill(prev, -1);
        dist[START] = 0;

        PriorityQueue<Integer> pq = new PriorityQueue<>((a, b) -> Double.compare(dist[a], dist[b]));
        pq.add(START);

        while (!pq.isEmpty()) {
            int u = pq.poll();
            if (visited[u]) continue;
            visited[u] = true;
            if (u == GOAL) break;

            for (int v = 0; v < n; v++) {
                if (v == u || visited[v]) continue;
                if (Obstacle.anyBlocks(obstacles,
                        nodes.get(u)[0], nodes.get(u)[1],
                        nodes.get(v)[0], nodes.get(v)[1], clearance)) {
                    continue; // line of sight blocked by an obstacle
                }
                double w = Math.hypot(nodes.get(v)[0] - nodes.get(u)[0], nodes.get(v)[1] - nodes.get(u)[1]);
                double alt = dist[u] + w;
                if (alt < dist[v]) {
                    dist[v] = alt;
                    prev[v] = u;
                    pq.add(v);
                }
            }
        }

        if (dist[GOAL] == Double.MAX_VALUE) {
            return null; // no reachable line-of-sight route
        }

        // Reconstruct the node-index path, start -> goal.
        List<Integer> indexPath = new ArrayList<>();
        int cur = GOAL;
        while (cur != -1) {
            indexPath.add(0, cur);
            cur = prev[cur];
        }

        // Convert to Pose waypoints with sensible headings:
        //  - first point keeps the robot's actual current heading
        //  - every later point's heading points back along the segment it
        //    just arrived on, so the robot (and its intake) is already
        //    facing the right way as it reaches each waypoint/ball
        List<Pose> waypoints = new ArrayList<>();
        for (int i = 0; i < indexPath.size(); i++) {
            double[] p = nodes.get(indexPath.get(i));
            double heading;
            if (i == 0) {
                heading = start.getHeading();
            } else {
                double[] prevP = nodes.get(indexPath.get(i - 1));
                heading = Math.atan2(p[1] - prevP[1], p[0] - prevP[0]);
            }
            waypoints.add(new Pose(p[0], p[1], heading));
        }

        return new PlanResult(waypoints, dist[GOAL]);
    }

    private static boolean insideAnyOtherObstacle(double px, double py, List<Obstacle> obstacles,
                                                  Obstacle self, double clearance) {
        for (Obstacle other : obstacles) {
            if (other == self) continue;
            double d = Math.hypot(px - other.x, py - other.y);
            if (d <= other.radius + clearance) return true;
        }
        return false;
    }
}