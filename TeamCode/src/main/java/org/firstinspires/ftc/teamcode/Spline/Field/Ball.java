package org.firstinspires.ftc.teamcode.Spline.Field;

import com.pedropathing.geometry.Pose;

/**
 * Simple data holder representing a ball (game piece) on the field.
 *
 * COORDINATES: matches Pedro Pathing's convention — inches, range [0, 144]
 * on both axes, with (0, 0) at the bottom-left corner of the field. This is
 * the same frame your Pedro Poses already use, so these values can be fed
 * straight into path building.
 */
public class Ball {
    public double x;
    public double y;
    public double radius; // for drawing + simple distance/avoidance checks

    public Ball() {
        this(0, 0, 2.0);
    }

    public Ball(double x, double y) {
        this(x, y, 2.0);
    }

    public Ball(double x, double y, double radius) {
        this.x = x;
        this.y = y;
        this.radius = radius;
    }

    /** Converts this ball's position into a Pedro Pose (heading defaults to 0). */
    public Pose toPose() {
        return toPose(0);
    }

    public Pose toPose(double headingRadians) {
        return new Pose(x, y, headingRadians);
    }

    public double distanceTo(Ball other) {
        double dx = other.x - x;
        double dy = other.y - y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    @Override
    public String toString() {
        return String.format("Ball(%.1f, %.1f)", x, y);
    }
}