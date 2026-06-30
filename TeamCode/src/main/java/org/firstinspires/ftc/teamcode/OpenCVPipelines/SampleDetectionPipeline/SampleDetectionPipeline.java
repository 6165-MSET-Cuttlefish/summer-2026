package org.firstinspires.ftc.teamcode.OpenCVPipelines.SampleDetectionPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * ARCHITECTURE NOTE (refactor):
 *
 * The homography is now used ONLY to transform individual detected ball
 * points into field-coordinate inches. The full camera frame is never
 * warped. This preserves the camera's full field of view and detects balls
 * at any distance, instead of being limited to whatever area the warped
 * top-down view used to cover.
 *
 * Trade-off this introduces: in the old warped-image approach, a ball's
 * apparent pixel size was constant no matter how far away it was (that's
 * the whole point of a top-down projection). In the raw camera frame, a
 * ball's apparent pixel size shrinks with distance — there is no single
 * "expected ball radius in pixels" anymore. All shape/size validation below
 * has been redesigned to be SCALE-INVARIANT (relative roundness ratios and
 * frame-area fractions) rather than relying on a fixed expected pixel size,
 * so detection still works whether a ball is close to the camera or far away.
 */
public class SampleDetectionPipeline extends OpenCvPipeline {

    // -------------------------------------------------------------------------
    // MODE SWITCH: set to true to skip live homography calibration and use the
    // hardcoded H_ARRAY below instead.
    // -------------------------------------------------------------------------
    private static final boolean USE_PREDETERMINED_HOMOGRAPHY = true;

    // -------------------------------------------------------------------------
    // DISPLAY MODE — both modes now draw on the ORIGINAL (unwarped) camera
    // frame, preserving full field of view.
    //
    //   MASK    — binary detection mask (white = yellow detected) at full
    //             camera resolution, with contour/center/contact overlays.
    //             Useful for tuning HSV thresholds and morphology.
    //
    //   OVERLAY — full-color camera image with the same overlays. Useful for
    //             verifying detections against the real scene.
    // -------------------------------------------------------------------------
    public enum DisplayMode { MASK, OVERLAY }
    private static final DisplayMode DISPLAY_MODE = DisplayMode.OVERLAY; // ← change here

    // -------------------------------------------------------------------------
    // Converted from HomographyCalculationPipeline output (pixel dst space) to
    // inch dst space via H_inch = S_inv * H_pixel, where
    // S_inv = [[1/25, 0, -10], [0, 1/25, -10], [0, 0, 1]].
    // Re-run live calibration (USE_PREDETERMINED_HOMOGRAPHY = false) for a
    // fresh result calibrated directly in inch space.
    // -------------------------------------------------------------------------
    private static final double[][] H_ARRAY = {
            { -3.5594949247e-01, -1.0612401847e-01,  1.2082718993e+02 },
            { -4.1371433084e-02, -7.8756315897e-01,  2.8349653964e+02 },
            { -2.8668090956e-04, -1.2403394999e-02,  1.0000000000e+00 }
    };

    // -------------------------------------------------------------------------
    // Detection downscale factor
    // -------------------------------------------------------------------------
    private static final double DETECTION_SCALE = 1.0;

    // -------------------------------------------------------------------------
    // Yellow ball HSV range.
    // -------------------------------------------------------------------------
    private static final Scalar YELLOW_LOW  = new Scalar(15, 100, 100);
    private static final Scalar YELLOW_HIGH = new Scalar(34, 255, 255);

    // -------------------------------------------------------------------------
    // Morphology kernels.
    // -------------------------------------------------------------------------
    private static final Mat CLOSE_KERNEL = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, new Size(5, 5));
    // Fills internal dimple-pattern holes within a SINGLE ball's blob. Kept
    // small on purpose: at 35x35 this was wide enough to also bridge the
    // (often very thin) dark gap between two balls that are touching or
    // nearly touching, fusing the whole cluster into one undetectable blob.
    private static final Mat FILL_KERNEL  = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, new Size(15, 15));
    // Extra hole-filling pass: dilate aggressively then erode back to recover
    // shape. Kept SMALL (smaller than CLOSE_KERNEL's effective close) on
    // purpose — this only needs to bridge a thin notch/gap within a single
    // ball's blob (glare, dimple pattern). If it's too large it will also
    // bridge the dark gap between two separate adjacent balls, fusing them
    // into one connected blob before watershed gets a chance to seed them.
    private static final Mat HOLE_KERNEL  = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, new Size(7, 7));

    // -------------------------------------------------------------------------
    // Local-maximum neighborhood for watershed seeds.
    // -------------------------------------------------------------------------
    private static final Mat LOCAL_MAX_KERNEL = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, new Size(15, 15));

    private static final double MIN_SEED_DEPTH_PX = 2.0;

    // -------------------------------------------------------------------------
    // Final per-region validation.
    //
    // CIRCULARITY / ROUNDNESS are intentionally lenient: a real ball whose
    // mask has a notch (glare, dimple pattern, or a neighboring ball's shadow
    // biting into the top of the blob) will score lower than a perfect circle
    // even after the HOLE_KERNEL closing pass above. Instead of requiring a
    // near-perfect circle, we accept anything "close enough" — a small notch
    // should not disqualify an otherwise round, correctly-sized blob.
    // -------------------------------------------------------------------------
    private static final double MIN_AREA_FRACTION = 0.00005;
    private static final double MAX_AREA_FRACTION = 0.15;
    private static final double MIN_CIRCULARITY = 0.25;
    private static final double MIN_ROUNDNESS_RATIO = 0.30;
    private static final double MIN_SEED_SEPARATION_PX = 12.0;

    // -------------------------------------------------------------------------
    // Homography calibration settings — GRID_COLS=9, GRID_ROWS=6 matches the
    // physical board (9 inner corners wide, 6 inner corners tall).
    // -------------------------------------------------------------------------
    private static final int   GRID_COLS        = 9;
    private static final int   GRID_ROWS        = 6;
    private static final int   EXPECTED_CORNERS = GRID_COLS * GRID_ROWS;
    private static final float SQUARE_SIZE_INCHES = 1.0f;
    private static final int   DETECTION_FRAME_INTERVAL = 3;
    private static final int   FRAMES_TO_CONFIRM = 5;

    // -------------------------------------------------------------------------
    // Pipeline state
    // -------------------------------------------------------------------------
    private enum Phase { CALIBRATING, DETECTING }

    private Phase phase;
    private Mat   homography   = null;
    private int   confirmCount = 0;
    private int   frameCount   = 0;

    private final Mat gray         = new Mat();
    private final Mat small        = new Mat();
    private final Mat hsv          = new Mat();
    private final Mat yellowMask   = new Mat();
    private final Mat cleanMask    = new Mat();
    private final Mat filledMask   = new Mat();
    private final Mat holeFilled   = new Mat();
    private final Mat distMat      = new Mat();
    private final Mat displayImage = new Mat();

    private final MatOfPoint2f dstCorners;

    private final Telemetry telemetry;

    private static class BallResult {
        MatOfPoint contourSmall;
        Point centerSmall;
        Point contactSmall;
        Point fieldPoint;
    }

    // =========================================================================
    // Constructor
    // =========================================================================
    public SampleDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;

        if (USE_PREDETERMINED_HOMOGRAPHY) {
            homography = buildHomographyFromArray(H_ARRAY);
            phase = Phase.DETECTING;
        } else {
            phase = Phase.CALIBRATING;
        }

        dstCorners = buildCalibrationDstCorners();
    }

    // =========================================================================
    // Public accessors
    // =========================================================================

    public Mat getHomography()    { return homography; }
    public boolean isCalibrated() { return phase == Phase.DETECTING; }

    public String getHomographyAsString() {
        if (homography == null || homography.empty()) return "Homography not available";
        StringBuilder sb = new StringBuilder("double[][] H_ARRAY = {\n");
        for (int r = 0; r < 3; r++) {
            sb.append("    { ");
            for (int c = 0; c < 3; c++) {
                sb.append(String.format("%.10e", homography.get(r, c)[0]));
                if (c < 2) sb.append(", ");
            }
            sb.append(" }");
            if (r < 2) sb.append(",");
            sb.append("\n");
        }
        sb.append("};");
        return sb.toString();
    }

    // =========================================================================
    // Main pipeline entry point
    // =========================================================================
    @Override
    public Mat processFrame(Mat input) {
        switch (phase) {
            case CALIBRATING: return runCalibrationFrame(input);
            case DETECTING:   return runDetectionFrame(input);
            default:          return input;
        }
    }

    // =========================================================================
    // PHASE 1 — Homography calibration
    // =========================================================================

    private Mat runCalibrationFrame(Mat input) {
        frameCount++;

        if (frameCount % DETECTION_FRAME_INTERVAL != 0) {
            telemetry.addLine("[Calibrating] Searching for chessboard...");
            telemetry.addData("Confirmed", confirmCount + " / " + FRAMES_TO_CONFIRM);
            telemetry.update();
            return input;
        }

        MatOfPoint2f imageCorners = detectChessboardCorners(input);

        if (imageCorners == null) {
            confirmCount = 0;
            telemetry.addLine("[Calibrating] Chessboard NOT found");
            telemetry.addData("Tip", "Ensure full board is visible and flat");
            telemetry.update();
            return input;
        }

        Mat h = computeHomography(imageCorners, dstCorners);

        if (h == null) {
            confirmCount = 0;
            telemetry.addLine("[Calibrating] Homography computation failed");
            telemetry.update();
            return input;
        }

        homography = h;
        confirmCount++;

        if (confirmCount >= FRAMES_TO_CONFIRM) {
            phase = Phase.DETECTING;
        }

        telemetry.addLine(phase == Phase.DETECTING
                ? "[LOCKED] Switching to detection..."
                : "[Calibrating] Confirming...");
        telemetry.addLine("--- Homography (image px -> field inches) ---");
        telemetry.addLine(getHomographyAsString());
        telemetry.update();

        Calib3d.drawChessboardCorners(input, new Size(GRID_COLS, GRID_ROWS), imageCorners, true);
        return input;
    }

    // =========================================================================
    // PHASE 2 — Ball detection
    // =========================================================================

    private Mat runDetectionFrame(Mat input) {

        Size smallSize = new Size(input.cols() * DETECTION_SCALE, input.rows() * DETECTION_SCALE);
        Imgproc.resize(input, small, smallSize, 0, 0, Imgproc.INTER_AREA);

        Imgproc.cvtColor(small, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, YELLOW_LOW, YELLOW_HIGH, yellowMask);

        Imgproc.morphologyEx(yellowMask, cleanMask,  Imgproc.MORPH_CLOSE, CLOSE_KERNEL);
        Imgproc.morphologyEx(cleanMask,  filledMask, Imgproc.MORPH_CLOSE, FILL_KERNEL);

        // Extra hole-filling pass: notches/gaps at the top of a ball (e.g. from
        // glare or the dimple pattern breaking up the yellow detection) leave
        // a non-convex bite in the blob. A small close (dilate -> erode) seals
        // those notches without bridging the gap to a neighboring ball, which
        // would otherwise fuse separate balls into one undetectable blob.
        Imgproc.morphologyEx(filledMask, holeFilled, Imgproc.MORPH_CLOSE, HOLE_KERNEL);

        input.copyTo(displayImage);

        List<BallResult> results = new ArrayList<>();

        if (Core.countNonZero(holeFilled) > 0) {

            Imgproc.distanceTransform(holeFilled, distMat, Imgproc.DIST_L2, 3);

            Mat dilated = new Mat();
            Imgproc.dilate(distMat, dilated, LOCAL_MAX_KERNEL);
            Mat isPeak = new Mat();
            Core.compare(distMat, dilated, isPeak, Core.CMP_EQ);

            Mat deepEnough = new Mat();
            Core.compare(distMat, new Scalar(MIN_SEED_DEPTH_PX), deepEnough, Core.CMP_GE);
            Core.bitwise_and(isPeak, deepEnough, isPeak);
            dilated.release();
            deepEnough.release();

            Mat seedMask = new Mat();
            isPeak.convertTo(seedMask, CvType.CV_8U);
            isPeak.release();

            Mat ccLabels    = new Mat();
            Mat ccStats     = new Mat();
            Mat ccCentroids = new Mat();
            int numLabels = Imgproc.connectedComponentsWithStats(seedMask, ccLabels, ccStats,
                    ccCentroids, 8, CvType.CV_32S);

            List<double[]> seedCenters = new ArrayList<>();
            for (int lbl = 1; lbl < numLabels; lbl++) {
                double cx = ccCentroids.get(lbl, 0)[0];
                double cy = ccCentroids.get(lbl, 1)[0];
                boolean merged = false;
                for (double[] existing : seedCenters) {
                    double dx = cx - existing[0], dy = cy - existing[1];
                    if (Math.sqrt(dx * dx + dy * dy) < MIN_SEED_SEPARATION_PX) {
                        merged = true;
                        break;
                    }
                }
                if (!merged) seedCenters.add(new double[]{cx, cy, lbl});
            }

            if (!seedCenters.isEmpty()) {
                int rows = holeFilled.rows(), cols = holeFilled.cols();
                byte[] filledRow = new byte[cols];
                int[]  labelRow  = new int[cols];
                int[]  markerRow = new int[cols];
                Mat freshMarkers = new Mat(rows, cols, CvType.CV_32SC1);

                for (int row = 0; row < rows; row++) {
                    holeFilled.get(row, 0, filledRow);
                    ccLabels.get(row, 0, labelRow);
                    for (int col = 0; col < cols; col++) {
                        if (filledRow[col] == 0) {
                            markerRow[col] = 0;
                        } else if (labelRow[col] > 0) {
                            markerRow[col] = labelRow[col];
                        } else {
                            markerRow[col] = -1;
                        }
                    }
                    freshMarkers.put(row, 0, markerRow);
                }

                Mat wsSource = new Mat();
                Imgproc.cvtColor(small, wsSource, Imgproc.COLOR_RGB2BGR);
                Imgproc.watershed(wsSource, freshMarkers);
                wsSource.release();

                double frameArea = small.cols() * small.rows();
                double minArea = frameArea * MIN_AREA_FRACTION;
                double maxArea = frameArea * MAX_AREA_FRACTION;

                for (double[] seed : seedCenters) {
                    int label = (int) seed[2];

                    Mat regionMask = new Mat();
                    Core.compare(freshMarkers, new Scalar(label), regionMask, Core.CMP_EQ);
                    regionMask.convertTo(regionMask, CvType.CV_8U, 255);

                    List<MatOfPoint> regionContours = new ArrayList<>();
                    Imgproc.findContours(regionMask, regionContours, new Mat(),
                            Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                    if (regionContours.isEmpty()) { regionMask.release(); continue; }

                    MatOfPoint contour = regionContours.get(0);
                    double area = Imgproc.contourArea(contour);
                    for (MatOfPoint c : regionContours) {
                        double a = Imgproc.contourArea(c);
                        if (a > area) { area = a; contour = c; }
                    }

                    if (area < minArea || area > maxArea) { regionMask.release(); continue; }

                    MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                    double perimeter = Imgproc.arcLength(contour2f, true);
                    double circularity = perimeter > 0
                            ? (4 * Math.PI * area) / (perimeter * perimeter)
                            : 0;
                    if (circularity < MIN_CIRCULARITY) { regionMask.release(); continue; }

                    Mat regionDist = new Mat(distMat.size(), distMat.type(), Scalar.all(0));
                    distMat.copyTo(regionDist, regionMask);
                    double peak = Core.minMaxLoc(regionDist).maxVal;
                    regionDist.release();
                    regionMask.release();

                    double impliedRadius = Math.sqrt(area / Math.PI);
                    double roundness = impliedRadius > 0 ? peak / impliedRadius : 0;
                    if (roundness < MIN_ROUNDNESS_RATIO) continue;

                    BallResult result = new BallResult();
                    result.contourSmall = contour;

                    org.opencv.imgproc.Moments m = Imgproc.moments(contour);
                    result.centerSmall = new Point(
                            m.m00 != 0 ? m.m10 / m.m00 : seed[0],
                            m.m00 != 0 ? m.m01 / m.m00 : seed[1]);

                    Point[] contourPoints = contour.toArray();
                    double contactX = contourPoints[0].x;
                    double contactY = contourPoints[0].y;
                    for (Point p : contourPoints) {
                        if (p.y > contactY) {
                            contactY = p.y;
                            contactX = p.x;
                        }
                    }
                    result.contactSmall = new Point(contactX, contactY);

                    double fullResX = contactX / DETECTION_SCALE;
                    double fullResY = contactY / DETECTION_SCALE;
                    result.fieldPoint = transformPointToField(fullResX, fullResY);

                    results.add(result);
                }

                freshMarkers.release();
            }

            seedMask.release();
            ccLabels.release();
            ccStats.release();
            ccCentroids.release();
        }

        if (DISPLAY_MODE == DisplayMode.MASK) {
            Mat upscaledMask = new Mat();
            Imgproc.resize(holeFilled, upscaledMask, input.size(), 0, 0, Imgproc.INTER_NEAREST);
            Imgproc.cvtColor(upscaledMask, displayImage, Imgproc.COLOR_GRAY2RGB);
            upscaledMask.release();
        }

        for (BallResult r : results) {
            drawBallOverlay(displayImage, r);
        }

        drawOriginCrosshair(displayImage);

        telemetry.addLine("[Detecting Yellow Balls]");
        telemetry.addData("Balls Detected", results.size());
        for (int i = 0; i < results.size(); i++) {
            Point fp = results.get(i).fieldPoint;
            telemetry.addLine("--- Ball " + i + " ---");
            telemetry.addData("  Field X (in)", String.format("%.2f", fp.x));
            telemetry.addData("  Field Y (in)", String.format("%.2f", fp.y));
        }
        telemetry.update();

        return displayImage;
    }

    private void drawBallOverlay(Mat displayImage, BallResult r) {
        double scaleUp = 1.0 / DETECTION_SCALE;

        Point[] smallPts = r.contourSmall.toArray();
        Point[] fullPts  = new Point[smallPts.length];
        for (int i = 0; i < smallPts.length; i++) {
            fullPts[i] = new Point(smallPts[i].x * scaleUp, smallPts[i].y * scaleUp);
        }
        MatOfPoint fullContour = new MatOfPoint(fullPts);
        List<MatOfPoint> singleContourList = new ArrayList<>();
        singleContourList.add(fullContour);
        Imgproc.drawContours(displayImage, singleContourList, -1, new Scalar(0, 255, 0), 2);

        Point centerFull  = new Point(r.centerSmall.x  * scaleUp, r.centerSmall.y  * scaleUp);
        Point contactFull = new Point(r.contactSmall.x * scaleUp, r.contactSmall.y * scaleUp);

        Imgproc.circle(displayImage, centerFull,  5, new Scalar(255, 255, 0), -1);
        Imgproc.circle(displayImage, contactFull, 5, new Scalar(0,   0, 255), -1);

        String label = String.format("(%.1f, %.1f)in", r.fieldPoint.x, r.fieldPoint.y);
        Imgproc.putText(displayImage, label,
                new Point(contactFull.x + 8, contactFull.y),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
    }

    private Point transformPointToField(double x, double y) {
        MatOfPoint2f src = new MatOfPoint2f(new Point(x, y));
        MatOfPoint2f dst = new MatOfPoint2f();
        Core.perspectiveTransform(src, dst, homography);
        return dst.toArray()[0];
    }

    /**
     * Inverse of transformPointToField: maps a field-space point (inches) back
     * into image pixel coordinates, using the inverse homography. Used to draw
     * the field-origin crosshair at the correct spot in the camera view.
     */
    private Point transformFieldToPoint(double fieldX, double fieldY) {
        Mat inverseHomography = homography.inv();
        MatOfPoint2f src = new MatOfPoint2f(new Point(fieldX, fieldY));
        MatOfPoint2f dst = new MatOfPoint2f();
        Core.perspectiveTransform(src, dst, inverseHomography);
        inverseHomography.release();
        return dst.toArray()[0];
    }

    /**
     * Draws a small crosshair at the field origin (0,0 inches), projected
     * into image pixel space via the inverse homography. Helpful for sanity
     * checking that the calibration's origin lines up with where you expect
     * it on the physical field.
     */
    private void drawOriginCrosshair(Mat displayImage) {
        if (homography == null || homography.empty()) return;

        Point originPixel = transformFieldToPoint(0.0, 0.0);
        // originPixel is in full-resolution pixel space already (no
        // DETECTION_SCALE division needed — the homography maps directly
        // between full-res camera pixels and field inches).

        int size = 10;
        Scalar color = new Scalar(255, 0, 255); // magenta — distinct from ball overlay colors

        Imgproc.line(displayImage,
                new Point(originPixel.x - size, originPixel.y),
                new Point(originPixel.x + size, originPixel.y),
                color, 2);
        Imgproc.line(displayImage,
                new Point(originPixel.x, originPixel.y - size),
                new Point(originPixel.x, originPixel.y + size),
                color, 2);
        Imgproc.circle(displayImage, originPixel, 3, color, -1);
        Imgproc.putText(displayImage, "(0,0)",
                new Point(originPixel.x + size + 4, originPixel.y),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }

    // =========================================================================
    // Homography calibration helper methods
    // =========================================================================

    private MatOfPoint2f detectChessboardCorners(Mat input) {
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

        MatOfPoint2f imageCorners = new MatOfPoint2f();
        boolean found = Calib3d.findChessboardCorners(
                gray,
                new Size(GRID_COLS, GRID_ROWS),
                imageCorners,
                Calib3d.CALIB_CB_ADAPTIVE_THRESH |
                        Calib3d.CALIB_CB_NORMALIZE_IMAGE |
                        Calib3d.CALIB_CB_FAST_CHECK
        );

        if (!found || imageCorners.rows() != EXPECTED_CORNERS) return null;

        Imgproc.cornerSubPix(
                gray, imageCorners,
                new Size(5, 5), new Size(-1, -1),
                new TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 30, 0.01)
        );

        return imageCorners;
    }

    private static Mat computeHomography(MatOfPoint2f srcCorners, MatOfPoint2f dstCorners) {
        Mat h = Calib3d.findHomography(srcCorners, dstCorners, Calib3d.RANSAC, 5.0);
        return (h == null || h.empty()) ? null : h;
    }

    /**
     * Destination corners in real-world INCHES: corner(col, row) = (col, row).
     * The resulting homography maps image pixels directly to field inches.
     */
    private static MatOfPoint2f buildCalibrationDstCorners() {
        List<Point> dstList = new ArrayList<>();
        for (int row = 0; row < GRID_ROWS; row++) {
            for (int col = 0; col < GRID_COLS; col++) {
                dstList.add(new Point(
                        col * SQUARE_SIZE_INCHES,
                        row * SQUARE_SIZE_INCHES
                ));
            }
        }
        MatOfPoint2f dst = new MatOfPoint2f();
        dst.fromList(dstList);
        return dst;
    }

    private static Mat buildHomographyFromArray(double[][] arr) {
        Mat h = new Mat(3, 3, CvType.CV_64F);
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                h.put(r, c, arr[r][c]);
        return h;
    }
}