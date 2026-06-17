package org.firstinspires.ftc.teamcode.OpenCVPipelines.HomographyCalculationPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class HomographyCalculationPipeline extends OpenCvPipeline {

    // Inner corners (NOT squares)
    private static final int GRID_COLS = 6;
    private static final int GRID_ROWS = 9;
    private static final int EXPECTED_CORNERS = GRID_COLS * GRID_ROWS;

    private static final float SQUARE_SIZE_INCHES = 1.0f;
    private static final float OUTPUT_SCALE_PX = 50.0f;
    private static final float MARGIN_PX = 250.0f;

    private static final int DETECTION_FRAME_INTERVAL = 3;
    private static final int FRAMES_TO_CONFIRM = 5;

    private final Telemetry telemetry;

    private final Mat gray = new Mat();
    private Mat homography = null;

    private int confirmCount = 0;
    private boolean locked = false;
    private int frameCount = 0;

    private final MatOfPoint2f dstCorners;

    private final int[] gridLineXCoords = new int[GRID_COLS + 1];
    private final int[] gridLineYCoords = new int[GRID_ROWS + 1];

    public HomographyCalculationPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;

        // Destination grid points
        List<Point> dstList = new ArrayList<>();
        for (int row = 0; row < GRID_ROWS; row++) {
            for (int col = 0; col < GRID_COLS; col++) {
                dstList.add(new Point(
                        MARGIN_PX + col * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX,
                        MARGIN_PX + row * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX
                ));
            }
        }
        dstCorners = new MatOfPoint2f();
        dstCorners.fromList(dstList);

        // Grid overlay lines
        for (int col = 0; col <= GRID_COLS; col++) {
            gridLineXCoords[col] = (int)(MARGIN_PX + col * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX);
        }
        for (int row = 0; row <= GRID_ROWS; row++) {
            gridLineYCoords[row] = (int)(MARGIN_PX + row * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX);
        }
    }

    public Mat getHomography() {
        return homography;
    }

    public boolean isLocked() {
        return locked;
    }

    @Override
    public Mat processFrame(Mat input) {

        if (locked) {
            return warpAndAnnotate(input);
        }

        frameCount++;

        if (frameCount % DETECTION_FRAME_INTERVAL != 0) {
            telemetry.addLine("Searching (frame skip)...");
            telemetry.addData("Confirmed", confirmCount + " / " + FRAMES_TO_CONFIRM);
            telemetry.update();
            return input;
        }

        // Convert to grayscale
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

        // Detect chessboard corners
        MatOfPoint2f imageCorners = new MatOfPoint2f();

        boolean found = Calib3d.findChessboardCorners(
                gray,
                new Size(GRID_COLS, GRID_ROWS),
                imageCorners,
                Calib3d.CALIB_CB_ADAPTIVE_THRESH +
                        Calib3d.CALIB_CB_NORMALIZE_IMAGE +
                        Calib3d.CALIB_CB_FAST_CHECK
        );

        if (!found || imageCorners.rows() != EXPECTED_CORNERS) {
            telemetry.addLine("Chessboard NOT found");
            telemetry.addData("Tip", "Ensure full board is visible and flat");
            telemetry.addData("Found", imageCorners.rows() + "/" + EXPECTED_CORNERS);
            telemetry.update();
            return input;
        }

        // Subpixel refinement
        Imgproc.cornerSubPix(
                gray,
                imageCorners,
                new Size(5, 5),
                new Size(-1, -1),
                new TermCriteria(
                        TermCriteria.EPS + TermCriteria.MAX_ITER,
                        30,
                        0.01
                )
        );

        // Compute homography
        Mat h = Calib3d.findHomography(imageCorners, dstCorners, Calib3d.RANSAC, 5.0);

        if (h == null || h.empty()) {
            telemetry.addLine("Homography failed");
            telemetry.update();
            return input;
        }

        homography = h;
        confirmCount++;

        if (confirmCount >= FRAMES_TO_CONFIRM) {
            locked = true;
        }

        // Telemetry output
        telemetry.addLine(locked ? "LOCKED" : "Confirming...");
        for (int r = 0; r < 3; r++) {
            telemetry.addData("Row " + r,
                    String.format("{ %.6f, %.6f, %.6f }",
                            h.get(r,0)[0],
                            h.get(r,1)[0],
                            h.get(r,2)[0]
                    )
            );
        }
        telemetry.update();

        // Draw detected corners
        Calib3d.drawChessboardCorners(
                input,
                new Size(GRID_COLS, GRID_ROWS),
                imageCorners,
                true
        );

        return input;
    }

    private Mat warpAndAnnotate(Mat input) {

        int width = (int)(GRID_COLS * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX + 2 * MARGIN_PX);
        int height = (int)(GRID_ROWS * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX + 2 * MARGIN_PX);

        Size warpSize = new Size(width, height);

        Mat warped = new Mat();
        Imgproc.warpPerspective(input, warped, homography, warpSize);

        Scalar green = new Scalar(0,255,0);

        int yStart = gridLineYCoords[0];
        int yEnd   = gridLineYCoords[GRID_ROWS];
        int xStart = gridLineXCoords[0];
        int xEnd   = gridLineXCoords[GRID_COLS];

        for (int x : gridLineXCoords) {
            Imgproc.line(warped, new Point(x,yStart), new Point(x,yEnd), green,1);
        }

        for (int y : gridLineYCoords) {
            Imgproc.line(warped, new Point(xStart,y), new Point(xEnd,y), green,1);
        }

        telemetry.addLine("Calibration Locked");
        telemetry.addLine("COPY HOMOGRAPHY:");
        telemetry.addLine(getHomographyAsString());
        telemetry.update();

        return warped;
    }
    public String getHomographyAsString() {
        if (homography == null || homography.empty()) {
            return "Homography not available";
        }

        StringBuilder sb = new StringBuilder();
        sb.append("double[][] H = {\n");

        for (int r = 0; r < 3; r++) {
            sb.append("    { ");
            for (int c = 0; c < 3; c++) {
                sb.append(String.format("%.10f", homography.get(r, c)[0]));
                if (c < 2) sb.append(", ");
            }
            sb.append(" }");
            if (r < 2) sb.append(",");
            sb.append("\n");
        }

        sb.append("};");
        return sb.toString();
    }
}