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

public class SampleDetectionPipeline extends OpenCvPipeline {

    // -------------------------------------------------------------------------
    // MODE SWITCH: set to true to skip live homography calibration and use the
    // hardcoded H_ARRAY below instead.
    // -------------------------------------------------------------------------
    private static final boolean USE_PREDETERMINED_HOMOGRAPHY = false;

    // -------------------------------------------------------------------------
    // DISPLAY MODE
    //
    //   MASK   — black-and-white: white where yellow was detected, black elsewhere.
    //            Useful for tuning HSV thresholds and morphology.
    //
    //   OVERLAY — full-color top-down warped image with a filled semi-transparent
    //             highlight on each detected ball and a circle + center dot on top.
    //             Useful for verifying position accuracy in the real scene.
    // -------------------------------------------------------------------------
    public enum DisplayMode { MASK, OVERLAY }
    private static final DisplayMode DISPLAY_MODE = DisplayMode.OVERLAY; // ← change here

    // -------------------------------------------------------------------------
    // Predetermined homography (used when USE_PREDETERMINED_HOMOGRAPHY = true)
    // -------------------------------------------------------------------------
    private static final double[][] H_ARRAY = {
            { 1.45538818e+00,  4.74651118e-01, -1.39014816e+02 },
            {-1.91056302e-02,  2.35917771e+00, -1.78695993e+02 },
            { 1.00860604e-04,  1.45801043e-03,  1.00000000e+00 }
    };

    // -------------------------------------------------------------------------
    // Pixel → inches conversion.
    // Since calibration places inner corners OUTPUT_SCALE_PX pixels apart and
    // each chessboard square is SQUARE_SIZE_INCHES inches, the ratio is exact.
    // -------------------------------------------------------------------------
    private static final float  OUTPUT_SCALE_PX    = 50.0f;
    private static final float  SQUARE_SIZE_INCHES = 1.0f;
    private static final double PIXELS_TO_INCHES   = SQUARE_SIZE_INCHES / OUTPUT_SCALE_PX; // 0.02

    // -------------------------------------------------------------------------
    // Coordinate origin in the full-res warped (top-down) image, in pixels.
    //
    // X increases to the RIGHT,  Y increases DOWNWARD (standard image convention).
    // Reported positions are (ball_center - origin) * PIXELS_TO_INCHES, so:
    //   • positive X = ball is to the RIGHT  of the origin
    //   • positive Y = ball is FURTHER AWAY  (down in the top-down view)
    //   • negative X = ball is to the LEFT   of the origin
    //   • negative Y = ball is CLOSER        (up in the top-down view)
    //
    // Set ORIGIN_X / ORIGIN_Y to the warped-image pixel coordinates that
    // correspond to the robot's camera position on the ground.
    // If the camera is at the left edge center of the warped image, for example:
    //   ORIGIN_X = 0,  ORIGIN_Y = warpHeight / 2
    // -------------------------------------------------------------------------
    private static final double ORIGIN_X = 0.0; // TODO: pixel x of robot/camera in warped image
    private static final double ORIGIN_Y = 0.0; // TODO: pixel y of robot/camera in warped image

    // -------------------------------------------------------------------------
    // Detection downscale factor.
    // All heavy processing (HSV threshold, morphology, contour detection) runs
    // on an image scaled down by this factor, then results are scaled back up.
    // 0.25 = quarter resolution — 16x fewer pixels, kernels 4x smaller.
    // Raise toward 0.5 if ball detection becomes inaccurate at distance;
    // lower toward 0.2 if still too slow.
    // -------------------------------------------------------------------------
    private static final double DETECTION_SCALE = 0.25;

    // -------------------------------------------------------------------------
    // Yellow ball HSV range.
    // Saturation and value floors are kept low so that glare-blown pixels
    // (which appear near-white: low saturation, high value) are still caught.
    // The hue range stays tight to avoid false positives from other colors.
    // -------------------------------------------------------------------------
    private static final Scalar YELLOW_LOW  = new Scalar(20,  100,  100);
    private static final Scalar YELLOW_HIGH = new Scalar(30, 255, 255);

    // -------------------------------------------------------------------------
    // Ball geometry — all computed in downscaled pixel space.
    //
    //   Full-res ball radius = 1.5 in / 0.02 px-per-in = 75 px
    //   Scaled radius        = 75 * DETECTION_SCALE     = 18.75 px  (at 0.25)
    //   Expected area        = π * r²                   ≈ 1,105 px²
    // -------------------------------------------------------------------------
    private static final double BALL_RADIUS_PX    = (1.5 / PIXELS_TO_INCHES);         // 75 px full-res
    private static final double BALL_RADIUS_SCALED = BALL_RADIUS_PX * DETECTION_SCALE; // ~18.75 px

    // Minimum blob area to be considered at all (filters pure noise).
    // Set to 40% of one ball — allows heavily glare-cropped crescents through.
    private static final double MIN_BLOB_AREA = Math.PI * BALL_RADIUS_SCALED * BALL_RADIUS_SCALED * 0.40;

    // Distance-transform peak threshold.
    // Peaks above this fraction of the local maximum are treated as ball centers.
    // Lower = more sensitive (catches dim peaks); higher = stricter.
    private static final double DIST_PEAK_THRESHOLD = 0.45;

    // -------------------------------------------------------------------------
    // Morphology kernels — sized for the downscaled image.
    //
    // At DETECTION_SCALE=0.25 the ball is ~19px radius in the small image.
    // CLOSE_KERNEL (5px): bridges small internal gaps / noise pixels.
    // FILL_KERNEL (19px): ~= ball radius; large enough to bridge the full
    //   glare hole across the center of the ball at this scale.
    // PEAK_DILATE_KERNEL: pre-allocated here instead of inside processFrame
    //   to avoid allocating a new Mat on every detection frame.
    // -------------------------------------------------------------------------
    private static final Mat CLOSE_KERNEL       = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, new Size(5, 5));
    private static final Mat FILL_KERNEL        = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, new Size(19, 19));
    private static final Mat PEAK_DILATE_KERNEL = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, new Size(3, 3));

    // -------------------------------------------------------------------------
    // Homography calibration settings
    // -------------------------------------------------------------------------
    private static final int GRID_COLS             = 9;
    private static final int GRID_ROWS             = 6;
    private static final int EXPECTED_CORNERS      = GRID_COLS * GRID_ROWS;
    private static final float MARGIN_PX           = 250.0f;
    private static final int DETECTION_FRAME_INTERVAL = 3;
    private static final int FRAMES_TO_CONFIRM     = 5;

    // -------------------------------------------------------------------------
    // Pipeline state
    // -------------------------------------------------------------------------
    private enum Phase { CALIBRATING, DETECTING }

    private Phase phase;
    private Mat   homography      = null; // full-res homography
    private Mat   homographySmall = null; // homography pre-scaled for direct small warp
    private int   confirmCount    = 0;
    private int   frameCount      = 0;
    private Size  warpSize;

    // Pre-allocated Mats — reused every frame to avoid GC pressure
    private final Mat warped       = new Mat(); // full-res top-down view (OVERLAY display)
    private final Mat warpedSmall  = new Mat(); // warped directly to small size (processing)
    private final Mat hsv          = new Mat();
    private final Mat yellowMask   = new Mat();
    private final Mat cleanMask    = new Mat();
    private final Mat filledMask   = new Mat(); // after hole-filling for glare
    private final Mat distMat      = new Mat(); // distance transform output (float)
    private final Mat distNorm     = new Mat(); // normalized 0–1 distance transform
    private final Mat peaks        = new Mat(); // thresholded peaks = ball centers
    private final Mat highlight    = new Mat(); // colored highlight layer for OVERLAY mode
    private final Mat contourImage = new Mat();

    // Warp output size at DETECTION_SCALE — computed once after calibration locks
    private Size smallWarpSize = null;

    // Calibration Mats
    private final Mat          gray       = new Mat();
    private final MatOfPoint2f dstCorners;

    private final Telemetry telemetry;

    // =========================================================================
    // Constructor
    // =========================================================================
    public SampleDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;

        if (USE_PREDETERMINED_HOMOGRAPHY) {
            homography      = buildHomographyFromArray(H_ARRAY);
            warpSize        = new Size(640, 480); // adjust to match your camera resolution
            smallWarpSize   = new Size(640 * DETECTION_SCALE, 480 * DETECTION_SCALE);
            homographySmall = buildSmallHomography(homography, DETECTION_SCALE);
            phase           = Phase.DETECTING;
        } else {
            phase = Phase.CALIBRATING;
        }

        dstCorners = buildCalibrationDstCorners();
    }

    // =========================================================================
    // Public accessors
    // =========================================================================

    public Mat     getHomography() { return homography; }
    public boolean isCalibrated()  { return phase == Phase.DETECTING; }

    /** Returns the homography as a copy-pasteable Java array string. */
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

        homography      = h;
        homographySmall = buildSmallHomography(h, DETECTION_SCALE);
        confirmCount++;

        if (confirmCount >= FRAMES_TO_CONFIRM) {
            lockCalibration();
        }

        telemetry.addLine(phase == Phase.DETECTING
                ? "[LOCKED] Switching to detection..."
                : "[Calibrating] Confirming...");
        reportHomographyToTelemetry(homography);
        telemetry.update();

        Calib3d.drawChessboardCorners(input, new Size(GRID_COLS, GRID_ROWS), imageCorners, true);
        return input;
    }

    private void lockCalibration() {
        int width  = (int)(GRID_COLS * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX + 2 * MARGIN_PX);
        int height = (int)(GRID_ROWS * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX + 2 * MARGIN_PX);
        warpSize        = new Size(width, height);
        smallWarpSize   = new Size(width * DETECTION_SCALE, height * DETECTION_SCALE);
        homographySmall = buildSmallHomography(homography, DETECTION_SCALE);
        phase           = Phase.DETECTING;
    }

    // =========================================================================
    // PHASE 2 — Yellow ball detection
    // =========================================================================

    private Mat runDetectionFrame(Mat input) {

        // 1. Warp perspective directly to small size.
        //    Combining warp + resize into a single warpPerspective call avoids
        //    allocating and processing a full-res intermediate image.
        //    The homography is pre-scaled by DETECTION_SCALE to map straight to
        //    the small output size.
        Size procSize = smallWarpSize != null ? smallWarpSize
                : new Size(input.cols() * DETECTION_SCALE, input.rows() * DETECTION_SCALE);
        Imgproc.warpPerspective(input, warpedSmall, homographySmall, procSize);

        // 2. HSV threshold on the small warped image
        Imgproc.cvtColor(warpedSmall, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, YELLOW_LOW, YELLOW_HIGH, yellowMask);

        // 3. Two-pass morphological closing to bridge glare holes.
        //    Pass 1 (CLOSE_KERNEL, 5px): removes noise, bridges tiny gaps.
        //    Pass 2 (FILL_KERNEL, 19px ≈ ball radius): fills the central glare hole.
        Imgproc.morphologyEx(yellowMask, cleanMask,  Imgproc.MORPH_CLOSE, CLOSE_KERNEL);
        Imgproc.morphologyEx(cleanMask,  filledMask, Imgproc.MORPH_CLOSE, FILL_KERNEL);

        // 4. Early exit: if there are no white pixels at all, skip the expensive
        //    distance transform and return a black frame immediately.
        if (Core.countNonZero(filledMask) == 0) {
            filledMask.copyTo(contourImage); // already black
            telemetry.addLine("[Detecting Yellow Balls]");
            telemetry.addData("Balls Detected", 0);
            telemetry.update();
            return contourImage;
        }

        // 5. Distance transform on the filled mask.
        //    DIST_L2 with mask size 3 (instead of 5) is faster and accurate
        //    enough at this resolution.
        Imgproc.distanceTransform(filledMask, distMat, Imgproc.DIST_L2, 3);

        // Normalize using the precomputed global max from minMaxLoc — one JNI
        // call instead of traversing all pixels twice (normalize does two passes).
        Core.MinMaxLocResult mmr = Core.minMaxLoc(distMat);
        if (mmr.maxVal == 0) {
            filledMask.copyTo(contourImage);
            telemetry.addLine("[Detecting Yellow Balls]");
            telemetry.addData("Balls Detected", 0);
            telemetry.update();
            return contourImage;
        }
        // Scale distMat → distNorm in-place via convertTo (single pass, no copy)
        distMat.convertTo(distNorm, CvType.CV_32F, 1.0 / mmr.maxVal);

        // Threshold and convert to 8U in one convertTo call
        Imgproc.threshold(distNorm, peaks, DIST_PEAK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        peaks.convertTo(peaks, CvType.CV_8U);

        // Dilate peaks using the pre-allocated kernel (no per-frame allocation)
        Imgproc.dilate(peaks, peaks, PEAK_DILATE_KERNEL);

        // 6. Find peak contours and compute centroids via moments()
        //    Imgproc.moments() is a single native call vs. a Java loop over points.
        List<MatOfPoint> peakContours = new ArrayList<>();
        Imgproc.findContours(peaks, peakContours, new Mat(),
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<float[]> balls = new ArrayList<>(peakContours.size());
        for (MatOfPoint pc : peakContours) {
            org.opencv.imgproc.Moments m = Imgproc.moments(pc);
            if (m.m00 == 0) continue;
            double cx = m.m10 / m.m00;
            double cy = m.m01 / m.m00;

            // Sample distance value at centroid using minMaxLoc on a 1px ROI —
            // avoids the slow mat.get() JNI allocation path entirely.
            int ix = (int) Math.min(Math.max(cx, 0), distNorm.cols() - 1);
            int iy = (int) Math.min(Math.max(cy, 0), distNorm.rows() - 1);
            double distVal = distNorm.get(iy, ix)[0];

            // Require centroid to be well inside a real blob (not a noise speck)
            if (distVal < 0.30) continue;

            // Scale back to full-res and use known physical radius
            balls.add(new float[]{
                    (float)(cx / DETECTION_SCALE),
                    (float)(cy / DETECTION_SCALE),
                    (float) BALL_RADIUS_PX
            });
        }

        // 7. Display — two modes controlled by DISPLAY_MODE at the top of the file.
        Size fullSize = warpSize != null ? warpSize : input.size();

        if (DISPLAY_MODE == DisplayMode.MASK) {
            // ── MASK mode ──────────────────────────────────────────────────────
            // White = yellow pixels that passed the HSV filter.
            // Black = everything else.
            // Circle overlays show where balls were found.
            Imgproc.resize(cleanMask, yellowMask, fullSize, 0, 0, Imgproc.INTER_NEAREST);
            Imgproc.cvtColor(yellowMask, contourImage, Imgproc.COLOR_GRAY2RGB);

            for (float[] ball : balls) {
                Imgproc.circle(contourImage,
                        new Point(ball[0], ball[1]), (int) ball[2],
                        new Scalar(0, 255, 0), 2);           // green ring
                Imgproc.circle(contourImage,
                        new Point(ball[0], ball[1]), 4,
                        new Scalar(0, 0, 255), -1);           // red center dot
            }

        } else {
            // ── OVERLAY mode ───────────────────────────────────────────────────
            // Full-color top-down warped image. Each detected ball gets:
            //   • a filled semi-transparent yellow highlight over its area
            //   • a green bounding circle outline
            //   • a red center dot
            // Only warps full-res when there are balls (saves time on empty frames).
            if (!balls.isEmpty()) {
                Imgproc.warpPerspective(input, warped, homography, fullSize);

                // Semi-transparent yellow fill: blend a solid-color circle into warped
                warped.copyTo(contourImage);
                highlight.create(contourImage.size(), contourImage.type());
                highlight.setTo(new Scalar(0, 0, 0));
                for (float[] ball : balls) {
                    Imgproc.circle(highlight,
                            new Point(ball[0], ball[1]), (int) ball[2],
                            new Scalar(0, 215, 255), -1);    // filled yellow circle
                }
                // Alpha blend: contourImage = 0.7*warped + 0.3*highlight
                Core.addWeighted(contourImage, 0.7, highlight, 0.3, 0, contourImage);

                // Draw circle outline and center dot on top
                for (float[] ball : balls) {
                    Imgproc.circle(contourImage,
                            new Point(ball[0], ball[1]), (int) ball[2],
                            new Scalar(0, 255, 0), 2);       // green ring
                    Imgproc.circle(contourImage,
                            new Point(ball[0], ball[1]), 4,
                            new Scalar(0, 0, 255), -1);      // red center dot
                }
            } else {
                // No balls — just show the warped image with no overlay
                Imgproc.warpPerspective(input, contourImage, homography, fullSize);
            }
        }

        // 8. Telemetry
        telemetry.addLine("[Detecting Yellow Balls]");
        telemetry.addData("Balls Detected", balls.size());
        for (int i = 0; i < balls.size(); i++) {
            float[] ball = balls.get(i);
            double xInches      = (ball[0] - ORIGIN_X) * PIXELS_TO_INCHES;
            double yInches      = (ball[1] - ORIGIN_Y) * PIXELS_TO_INCHES;
            double radiusInches = ball[2] * PIXELS_TO_INCHES;

            telemetry.addLine("--- Ball " + i + " ---");
            telemetry.addData("  Position X (in)",  String.format("%.2f", xInches));
            telemetry.addData("  Position Y (in)",  String.format("%.2f", yInches));
            telemetry.addData("  Radius (in)",      String.format("%.2f", radiusInches));
        }
        telemetry.update();

        return contourImage;
    }

    // =========================================================================
    // Homography helper methods
    // =========================================================================

    /**
     * Detects inner chessboard corners in the given frame with sub-pixel refinement.
     * Returns null if detection fails or corner count is wrong.
     */
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

    /**
     * Computes a homography mapping srcCorners → dstCorners using RANSAC.
     * Returns null if OpenCV returns an empty result.
     */
    private static Mat computeHomography(MatOfPoint2f srcCorners, MatOfPoint2f dstCorners) {
        Mat h = Calib3d.findHomography(srcCorners, dstCorners, Calib3d.RANSAC, 5.0);
        return (h == null || h.empty()) ? null : h;
    }

    /**
     * Builds the destination corner grid used during calibration.
     * Inner corners only, spaced SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX pixels apart.
     */
    private static MatOfPoint2f buildCalibrationDstCorners() {
        List<Point> dstList = new ArrayList<>();
        for (int row = 0; row < GRID_ROWS; row++) {
            for (int col = 0; col < GRID_COLS; col++) {
                dstList.add(new Point(
                        MARGIN_PX + col * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX,
                        MARGIN_PX + row * SQUARE_SIZE_INCHES * OUTPUT_SCALE_PX
                ));
            }
        }
        MatOfPoint2f dst = new MatOfPoint2f();
        dst.fromList(dstList);
        return dst;
    }

    /**
     * Scales a homography so it maps directly to an image downscaled by `scale`.
     * Multiplying the output columns by `scale` achieves this in one matrix op.
     * This lets warpPerspective output the small image in a single call,
     * avoiding a separate resize step entirely.
     */
    private static Mat buildSmallHomography(Mat h, double scale) {
        // Scale matrix: S = diag(scale, scale, 1)
        // smallH = S * H
        Mat smallH = h.clone();
        for (int c = 0; c < 3; c++) {
            smallH.put(0, c, h.get(0, c)[0] * scale);
            smallH.put(1, c, h.get(1, c)[0] * scale);
            // row 2 (homogeneous) is unchanged
        }
        return smallH;
    }

    /**
     * Builds a 3×3 CV_64F Mat from a raw 3×3 double array.
     * Used to load the predetermined H_ARRAY.
     */
    private static Mat buildHomographyFromArray(double[][] arr) {
        Mat h = new Mat(3, 3, CvType.CV_64F);
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                h.put(r, c, arr[r][c]);
        return h;
    }

    /**
     * Writes the three rows of a homography matrix to telemetry.
     */
    private void reportHomographyToTelemetry(Mat h) {
        telemetry.addLine("--- Homography Matrix ---");
        for (int r = 0; r < 3; r++) {
            telemetry.addData("Row " + r,
                    String.format("{ %.6e, %.6e, %.6e }",
                            h.get(r, 0)[0],
                            h.get(r, 1)[0],
                            h.get(r, 2)[0]));
        }
        telemetry.addLine("Copy-paste string:");
        telemetry.addLine(getHomographyAsString());
    }
}