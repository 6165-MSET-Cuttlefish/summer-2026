package org.firstinspires.ftc.teamcode.architecture.auto;

import static org.firstinspires.ftc.teamcode.core.Robot.robot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;

public class FieldVisualization {
    public static final double ROBOT_RADIUS = 7.5;

    public static final String COLOR_ROBOT = "#FFFFFF";
    public static final String COLOR_PATH = "#ea8743";
    public static final String COLOR_CURRENT_PATH = "#00ff26";
    public static final String COLOR_HISTORY = "#ff0015";

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

    public static Canvas getCanvas() {
        return robot.packet.fieldOverlay();
    }

    public static void drawRobot(Pose pose) {
        Canvas canvas = robot.packet.fieldOverlay();

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

    public static void drawPath(Path path, String color) {
        Canvas canvas = robot.packet.fieldOverlay();
        canvas.setStroke(color);

        try {
            double[][] points = path.getPanelsDrawingPoints(); // points[0] = x‑array, points[1] = y‑array
            for (int i = 0; i < points[0].length - 1; i++) {
                double[] p1 = toField(points[0][i], points[1][i]);
                double[] p2 = toField(points[0][i + 1], points[1][i + 1]);
                canvas.strokeLine(p1[0], p1[1], p2[0], p2[1]);
            }
        } catch (Exception e) {
            System.err.println("FieldVisualization: drawPath failed: " + e.getMessage());
        }
    }

    public static void drawPath(PathChain pathChain, String color) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), color);
        }
    }

    public static void drawPoseHistory(PoseHistory poseHistory) {
        Canvas canvas = robot.packet.fieldOverlay();
        canvas.setStroke(COLOR_HISTORY);

        double[] x = poseHistory.getXPositionsArray();
        double[] y = poseHistory.getYPositionsArray();

        int length = Math.min(x.length, y.length);
        for (int i = 0; i < length - 1; i++) {
            double[] p1 = toField(x[i], y[i]);
            double[] p2 = toField(x[i + 1], y[i + 1]);
            canvas.strokeLine(p1[0], p1[1], p2[0], p2[1]);
        }
    }
}
