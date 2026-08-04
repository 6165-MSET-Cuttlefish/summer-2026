package org.firstinspires.ftc.teamcode.architecture.auto;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;

@Config
public class FieldVisualization {
    public static final double ROBOT_RADIUS = 7.5;

    public static final String COLOR_ROBOT = "#FFFFFF";
    public static final String COLOR_PATH = "#ea8743";
    public static final String COLOR_CURRENT_PATH = "#00ff26";
    public static final String COLOR_HISTORY = "#ff0015";
    public static final String COLOR_HEADING = "#00e5ff";

    public static int headingTickCount = 8;
    public static double headingTickLength = 5.0;
    /** Perpendicular nudge off the path, so a tick stays visible where planned heading ∥ path tangent. */
    public static double headingTickOffset = 1.5;

    private FieldVisualization() {}

    public static Pose toField(Pose pedroPose) {
        return pedroPose.getAsCoordinateSystem(FTCCoordinates.INSTANCE);
    }

    /** Mirrors FTCCoordinates.convertFromPedro without allocating a Pose. */
    public static double[] toField(double x, double y) {
        double transX = x - 72.0;
        double transY = y - 72.0;
        return new double[]{ transY, -transX };
    }

    public static void drawRobot(Canvas canvas, Pose pose) {
        Pose canvasPose = toField(pose);
        double cx = canvasPose.getX();
        double cy = canvasPose.getY();

        canvas.setStroke(COLOR_ROBOT);
        canvas.strokeCircle(cx, cy, ROBOT_RADIUS);

        Vector heading = canvasPose.getHeadingAsUnitVector();
        canvas.strokeLine(cx, cy,
                cx + heading.getXComponent() * ROBOT_RADIUS,
                cy + heading.getYComponent() * ROBOT_RADIUS);
    }

    /** One polyline op per curve instead of one strokeLine per segment — same picture, far smaller packet. */
    private static void strokePedroPolyline(Canvas canvas, double[] pedroX, double[] pedroY) {
        int n = Math.min(pedroX.length, pedroY.length);
        if (n < 2) return;
        double[] fx = new double[n];
        double[] fy = new double[n];
        for (int i = 0; i < n; i++) {
            double[] p = toField(pedroX[i], pedroY[i]);
            fx[i] = p[0];
            fy[i] = p[1];
        }
        canvas.strokePolyline(fx, fy);
    }

    public static void drawPath(Canvas canvas, Path path, String color) {
        canvas.setStroke(color);
        double[][] points = path.getPanelsDrawingPoints(); // points[0] = x‑array, points[1] = y‑array
        strokePedroPolyline(canvas, points[0], points[1]);
    }

    public static void drawPath(Canvas canvas, PathChain pathChain, String color) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(canvas, pathChain.getPath(i), color);
        }
    }

    /**
     * Ticks showing the heading the follower is <em>supposed</em> to hold along the path, to compare against
     * the robot marker's own heading line. Each tick is nudged perpendicular to the path so it stays legible
     * where the planned heading runs parallel to the path instead of hiding inside the path line.
     */
    public static void drawPlannedHeading(Canvas canvas, Path path) {
        canvas.setStroke(COLOR_HEADING);
        int ticks = Math.max(1, headingTickCount);
        for (int i = 0; i <= ticks; i++) {
            double t = (double) i / ticks;
            // getPose(t) carries the interpolated heading goal; toField rotates position AND heading.
            Pose planned = toField(path.getPose(t));
            Vector direction = planned.getHeadingAsUnitVector();

            // Path normal in canvas space, from two nearby samples — avoids re-deriving the frame rotation.
            Pose before = toField(path.getPoint(Math.max(0.0, t - 0.01)));
            Pose after = toField(path.getPoint(Math.min(1.0, t + 0.01)));
            double nx = -(after.getY() - before.getY());
            double ny = after.getX() - before.getX();
            double norm = Math.hypot(nx, ny);
            double offsetX = norm > 1e-9 ? nx / norm * headingTickOffset : 0.0;
            double offsetY = norm > 1e-9 ? ny / norm * headingTickOffset : 0.0;

            double x = planned.getX() + offsetX;
            double y = planned.getY() + offsetY;
            canvas.strokeLine(x, y,
                    x + direction.getXComponent() * headingTickLength,
                    y + direction.getYComponent() * headingTickLength);
        }
    }

    public static void drawPlannedHeading(Canvas canvas, PathChain pathChain) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPlannedHeading(canvas, pathChain.getPath(i));
        }
    }

    public static void drawPoseHistory(Canvas canvas, PoseHistory poseHistory) {
        canvas.setStroke(COLOR_HISTORY);
        strokePedroPolyline(canvas,
                poseHistory.getXPositionsArray(), poseHistory.getYPositionsArray());
    }
}
