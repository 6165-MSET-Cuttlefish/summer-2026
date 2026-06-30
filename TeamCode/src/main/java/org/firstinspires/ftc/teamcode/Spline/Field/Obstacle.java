package org.firstinspires.ftc.teamcode.Spline.Field;

import com.pedropathing.geometry.Pose;

import java.util.List;

/**
 * Simple data holder representing a circular obstacle on the field.
 *
 * COORDINATES: same Pedro Pathing convention as Ball — inches, range
 * [0, 144] on both axes, (0, 0) at the bottom-left corner of the field.
 *
 * A circle (rather than a rectangle) keeps later path-planning math simple:
 * point-to-circle distance checks, inflating the radius by the robot's own
 * radius for clearance, line-segment/circle intersection for "does this path
 * segment clip an obstacle", etc.
 */
public class Obstacle {
    public double x;
    public double y;
    public double radius;

    public Obstacle() {
        this(0, 0, 6.0);
    }

    public Obstacle(double x, double y, double radius) {
        this.x = x;
        this.y = y;
        this.radius = radius;
    }

    /** Returns true if the given point lies inside this obstacle (no clearance). */
    public boolean contains(double px, double py) {
        double dx = px - x;
        double dy = py - y;
        return (dx * dx + dy * dy) <= (radius * radius);
    }

    /**
     * Returns true if the straight-line path from (x1,y1) to (x2,y2) comes
     * within {@code clearance} inches of this obstacle's edge.
     *
     * This models the robot as a point but inflates the obstacle by
     * {@code clearance}, which is equivalent to sweeping a circle of radius
     * {@code clearance} along the segment. Set clearance to half the robot's
     * widest dimension (e.g. ROBOT_SIZE_IN / 2.0) so the robot body is
     * accounted for even when only the centre-line is checked.
     *
     * Technique: project the obstacle centre onto the segment, clamp to
     * [0,1], and compare the squared distance to (radius + clearance)².
     */
    public boolean intersectsSegment(double x1, double y1, double x2, double y2,
                                     double clearance) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double lenSq = dx * dx + dy * dy;

        // t is the parameter of the closest point on the segment to this obstacle
        double t = (lenSq == 0) ? 0
                : ((x - x1) * dx + (y - y1) * dy) / lenSq;
        t = Math.max(0, Math.min(1, t));

        double closestX = x1 + t * dx;
        double closestY = y1 + t * dy;

        double distSq = (closestX - x) * (closestX - x)
                + (closestY - y) * (closestY - y);
        double minDist = radius + clearance;
        return distSq <= minDist * minDist;
    }

    /**
     * Convenience: returns true if ANY obstacle in the list blocks the segment
     * from (x1,y1) to (x2,y2) given the robot clearance radius.
     *
     * @param obstacles  list of obstacles to check (e.g. FieldVisualizer.getObstacles())
     * @param clearance  robot half-width in inches (use ROBOT_SIZE_IN / 2.0)
     */
    public static boolean anyBlocks(List<Obstacle> obstacles,
                                    double x1, double y1,
                                    double x2, double y2,
                                    double clearance) {
        for (Obstacle o : obstacles) {
            if (o.intersectsSegment(x1, y1, x2, y2, clearance)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Returns the first obstacle that blocks the segment, or null if the path
     * is clear. Useful for logging which specific obstacle is in the way.
     */
    public static Obstacle firstBlocking(List<Obstacle> obstacles,
                                         double x1, double y1,
                                         double x2, double y2,
                                         double clearance) {
        for (Obstacle o : obstacles) {
            if (o.intersectsSegment(x1, y1, x2, y2, clearance)) {
                return o;
            }
        }
        return null;
    }

    public Pose toPose() {
        return new Pose(x, y, 0);
    }

    @Override
    public String toString() {
        return String.format("Obstacle(%.1f, %.1f, r=%.1f)", x, y, radius);
    }
}