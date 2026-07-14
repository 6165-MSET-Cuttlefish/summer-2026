package org.firstinspires.ftc.teamcode.OpenCVPipelines.SampleDetectionPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
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
    // DISPLAY MODE — all modes draw on the ORIGINAL (unwarped) camera frame,
    // preserving full field of view.
    //
    //   MASK    — binary detection mask (white = yellow detected) at full
    //             camera resolution, with contour/center/contact overlays.
    //             Useful for tuning HSV thresholds and morphology.
    //
    //   OVERLAY — full-color camera image with contour outline + center dot +
    //             contact dot overlays. Useful for verifying detections
    //             against the real scene.
    //
    //   BOX     — full-color camera image with a clean axis-aligned bounding
    //             box around each ball and a solid label plate above the box
    //             showing the ball number and its calculated field
    //             coordinates. Best for a clean, presentation-style view.
    // -------------------------------------------------------------------------
    public enum DisplayMode { MASK, OVERLAY, BOX }
    private static final DisplayMode DISPLAY_MODE = DisplayMode.BOX; // ← change here

    // -------------------------------------------------------------------------
    // QUICK FIX APPLIED: every reported distance was exactly 2x too large.
    // The matrix below is the original H_ARRAY pre-multiplied by the scale
    // matrix diag(0.5, 0.5, 1) — i.e. S * H, where S halves only the output
    // (x, y) rows and leaves the bottom projective row untouched. This is the
    // mathematically correct way to halve every output distance; naively
    // dividing all 9 raw entries by 2 would also incorrectly scale the
    // perspective terms in the bottom row and break the transform.
    //
    // This patches the symptom, not the root cause. The original matrix was
    // itself a manual pixel-to-inch conversion of an older pipeline's output
    // (see the derivation note that used to be here) — that conversion is
    // where the 2x almost certainly crept in. If distances drift off again
    // after any recalibration, redo this scale correction, or better, run
    // live calibration once (USE_PREDETERMINED_HOMOGRAPHY = false) to get a
    // matrix computed directly by this pipeline with no manual conversion
    // step at all.
    // -------------------------------------------------------------------------
    private static final double[][] H_ARRAY = {
            { -1.7797474624e-01, -5.3062009235e-02,  6.0413594965e+01 },
            { -2.0685716542e-02, -3.9378157948e-01,  1.4174826982e+02 },
            { -2.8668090956e-04, -1.2403394999e-02,  1.0000000000e+00 }
    };

    // -------------------------------------------------------------------------
    // Detection downscale factor.
    //
    // PERFORMANCE: this is the single biggest lever in the whole pipeline.
    // HSV conversion, all three morphology passes, the distance transform,
    // and the per-pixel Java loop that builds the watershed markers mat all
    // scale with the NUMBER OF PIXELS processed, which scales with the SQUARE
    // of this factor (half the linear scale = quarter the pixels = ~4x faster
    // on every one of those steps). At 1.0 (full 640x480 camera resolution)
    // this is by far the largest contributor to a ~1000ms frame time.
    //
    // 0.5 processes ~4x fewer pixels than 1.0 and is a safe starting point —
    // ball detection accuracy is largely unaffected because the shape/size
    // checks later in the pipeline are scale-invariant (fractions of frame
    // area, not fixed pixel counts). Detected points are scaled back up to
    // full resolution before being transformed by the homography, so
    // reported field coordinates are not affected by this value at all.
    //
    // Lower further (e.g. 0.35) for more speed if 0.5 isn't enough; raise
    // toward 1.0 only if small/distant balls stop being detected reliably.
    // -------------------------------------------------------------------------
    private static final double DETECTION_SCALE = 0.5;

    // -------------------------------------------------------------------------
    // Yellow ball HSV range.
    // -------------------------------------------------------------------------
    private static final Scalar YELLOW_LOW  = new Scalar(15, 120, 120);
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
    // -------------------------------------------------------------------------
    // Physical size of ONE chessboard square, in inches. This is the single
    // source of truth for the scale of every distance the pipeline reports —
    // it directly defines what "1 unit" means in the calibration destination
    // grid (see buildCalibrationDstCorners() below), so an error here produces
    // a uniform, constant-factor error in every single reported coordinate.
    //
    // If every reported distance is consistently OFF BY A FIXED MULTIPLE
    // (e.g. exactly 2x too large, or exactly 2x too small), the almost
    // certain cause is this constant not matching your PHYSICAL PRINTED
    // board — not a bug in the transform math. Measure an actual square
    // on the printed board with a ruler (not the pattern's nominal/intended
    // size, since printers/PDF scaling can shrink or enlarge it) and set
    // this to that measured value.
    //
    //   distances doubled  -> this constant is HALF of the true square size
    //                         -> multiply it by 2
    //   distances halved   -> this constant is DOUBLE the true square size
    //                         -> divide it by 2
    //
    // IMPORTANT: after changing this value you MUST re-run live calibration
    // (USE_PREDETERMINED_HOMOGRAPHY = false) and re-copy the resulting
    // H_ARRAY from telemetry. Simply editing this constant does NOT retroactively
    // fix a homography matrix that was already computed/copied using the old
    // value — the old H_ARRAY has the wrong scale baked into it permanently.
    // -------------------------------------------------------------------------
    private static final float SQUARE_SIZE_INCHES = 1.0f; // TODO: verify against your physical board
    private static final int   DETECTION_FRAME_INTERVAL = 3;
    private static final int   FRAMES_TO_CONFIRM = 5;

    // -------------------------------------------------------------------------
    // Pipeline state
    // -------------------------------------------------------------------------
    private enum Phase { CALIBRATING, DETECTING }

    // volatile: written on the camera thread (processFrame), read on the OpMode thread (getHomography /
    // isCalibrated / getHomographyAsString).
    private volatile Phase phase;
    private volatile Mat   homography   = null;
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
    // Reused findContours hierarchy output — was a per-seed `new Mat()` that leaked every frame.
    private final Mat contourHierarchy = new Mat();

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
            imageCorners.release();
            confirmCount = 0;
            telemetry.addLine("[Calibrating] Homography computation failed");
            telemetry.update();
            return input;
        }

        // Release the homography we're superseding so calibration frames don't leak one each.
        Mat previousHomography = homography;
        homography = h;
        if (previousHomography != null && previousHomography != h) previousHomography.release();
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
        imageCorners.release();
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

        // PERFORMANCE: only copy the full-resolution input when it will
        // actually be used. MASK mode overwrites displayImage entirely a few
        // lines below, so copying `input` into it here was wasted work.
        if (DISPLAY_MODE != DisplayMode.MASK) {
            input.copyTo(displayImage);
        }

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

                // PERFORMANCE: process each ball within a small ROI (region of
                // interest) around its seed instead of operating on the whole
                // frame. Previously, Core.compare/copyTo/minMaxLoc all ran on
                // full-size Mats once PER BALL — with N balls that's N passes
                // over the entire frame. A ROI sized generously around each
                // seed (using MAX_AREA_FRACTION as an upper bound on ball
                // size) cuts that down to a small crop per ball, independent
                // of how many balls are in the frame.
                int maxBallDim = (int) Math.ceil(Math.sqrt(maxArea)) * 2;
                int roiPad = Math.max(8, maxBallDim);

                for (double[] seed : seedCenters) {
                    int label = (int) seed[2];
                    int seedCx = (int) seed[0];
                    int seedCy = (int) seed[1];

                    int roiX = Math.max(0, seedCx - roiPad);
                    int roiY = Math.max(0, seedCy - roiPad);
                    int roiW = Math.min(freshMarkers.cols() - roiX, roiPad * 2);
                    int roiH = Math.min(freshMarkers.rows() - roiY, roiPad * 2);
                    if (roiW <= 0 || roiH <= 0) continue;
                    Rect roi = new Rect(roiX, roiY, roiW, roiH);

                    // Per-seed native Mats. This loop runs once per candidate ball on every processed
                    // frame, so anything not released here accumulates on the Control Hub's native
                    // heap and crashes the process within a match — release them all in finally so no
                    // early-out (`continue`) can leak. (offsetContour is kept in the BallResult for
                    // the later draw pass and released after that; see below.)
                    Mat markersRoi = freshMarkers.submat(roi);
                    Mat regionMask = new Mat();
                    List<MatOfPoint> regionContours = new ArrayList<>();
                    MatOfPoint2f contour2f = null;
                    Mat distRoi = null;
                    Mat regionDist = null;
                    try {
                        Core.compare(markersRoi, new Scalar(label), regionMask, Core.CMP_EQ);
                        regionMask.convertTo(regionMask, CvType.CV_8U, 255);

                        Imgproc.findContours(regionMask, regionContours, contourHierarchy,
                                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                        if (regionContours.isEmpty()) continue;

                        MatOfPoint contour = regionContours.get(0);
                        double area = Imgproc.contourArea(contour);
                        for (MatOfPoint c : regionContours) {
                            double a = Imgproc.contourArea(c);
                            if (a > area) { area = a; contour = c; }
                        }

                        if (area < minArea || area > maxArea) continue;

                        contour2f = new MatOfPoint2f(contour.toArray());
                        double perimeter = Imgproc.arcLength(contour2f, true);
                        double circularity = perimeter > 0
                                ? (4 * Math.PI * area) / (perimeter * perimeter)
                                : 0;
                        if (circularity < MIN_CIRCULARITY) continue;

                        // Distance-transform peak, also restricted to the same ROI
                        distRoi = distMat.submat(roi);
                        regionDist = new Mat(distRoi.size(), distRoi.type(), Scalar.all(0));
                        distRoi.copyTo(regionDist, regionMask);
                        double peak = Core.minMaxLoc(regionDist).maxVal;

                        double impliedRadius = Math.sqrt(area / Math.PI);
                        double roundness = impliedRadius > 0 ? peak / impliedRadius : 0;
                        if (roundness < MIN_ROUNDNESS_RATIO) continue;

                        BallResult result = new BallResult();

                        // The contour was found within a cropped ROI submat, so its
                        // point coordinates are ROI-local. Offset every point by the
                        // ROI's top-left corner to get back to full-small-image
                        // coordinates before storing/using them anywhere downstream.
                        Point[] roiLocalPts = contour.toArray();
                        Point[] offsetPts = new Point[roiLocalPts.length];
                        for (int i = 0; i < roiLocalPts.length; i++) {
                            offsetPts[i] = new Point(roiLocalPts[i].x + roi.x, roiLocalPts[i].y + roi.y);
                        }
                        MatOfPoint offsetContour = new MatOfPoint(offsetPts);
                        result.contourSmall = offsetContour;

                        org.opencv.imgproc.Moments m = Imgproc.moments(offsetContour);
                        result.centerSmall = new Point(
                                m.m00 != 0 ? m.m10 / m.m00 : seed[0],
                                m.m00 != 0 ? m.m01 / m.m00 : seed[1]);

                        double contactX = offsetPts[0].x;
                        double contactY = offsetPts[0].y;
                        for (Point p : offsetPts) {
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
                    } finally {
                        markersRoi.release();
                        regionMask.release();
                        if (contour2f != null) contour2f.release();
                        if (distRoi != null) distRoi.release();
                        if (regionDist != null) regionDist.release();
                        for (MatOfPoint c : regionContours) c.release();
                    }
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

        if (DISPLAY_MODE == DisplayMode.BOX) {
            for (int i = 0; i < results.size(); i++) {
                drawBallBox(displayImage, results.get(i), i);
            }
        } else {
            for (BallResult r : results) {
                drawBallOverlay(displayImage, r);
            }
        }

        // Free the per-ball contour Mats now drawing is done (kept out of the per-seed finally
        // specifically so this draw pass could use them).
        for (int i = 0; i < results.size(); i++) {
            if (results.get(i).contourSmall != null) results.get(i).contourSmall.release();
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
        fullContour.release();

        Point centerFull  = new Point(r.centerSmall.x  * scaleUp, r.centerSmall.y  * scaleUp);
        Point contactFull = new Point(r.contactSmall.x * scaleUp, r.contactSmall.y * scaleUp);

        Imgproc.circle(displayImage, centerFull,  5, new Scalar(255, 255, 0), -1);
        Imgproc.circle(displayImage, contactFull, 5, new Scalar(0,   0, 255), -1);

        String label = String.format("(%.1f, %.1f)in", r.fieldPoint.x, r.fieldPoint.y);
        Imgproc.putText(displayImage, label,
                new Point(contactFull.x + 8, contactFull.y),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
    }

    /**
     * BOX display mode: draws a clean axis-aligned bounding rectangle around
     * the ball (derived from its contour, scaled to full resolution) plus a
     * solid label plate directly above the box showing the ball's index and
     * its calculated field coordinates. The plate is sized to the text it
     * contains and clamped to stay within the frame so it's always readable.
     */
    private void drawBallBox(Mat displayImage, BallResult r, int ballIndex) {
        double scaleUp = 1.0 / DETECTION_SCALE;

        // Scale the contour to full resolution, then take its bounding rect.
        Point[] smallPts = r.contourSmall.toArray();
        Point[] fullPts  = new Point[smallPts.length];
        for (int i = 0; i < smallPts.length; i++) {
            fullPts[i] = new Point(smallPts[i].x * scaleUp, smallPts[i].y * scaleUp);
        }
        MatOfPoint boxContour = new MatOfPoint(fullPts);
        Rect box = Imgproc.boundingRect(boxContour);
        boxContour.release();

        Scalar boxColor  = new Scalar(0, 255, 0);   // green
        Scalar textColor = new Scalar(255, 255, 255); // white

        // Bounding box
        Imgproc.rectangle(displayImage,
                new Point(box.x, box.y),
                new Point(box.x + box.width, box.y + box.height),
                boxColor, 2);

        // Label text: ball number + field coordinates
        String label = String.format("#%d (%.1f, %.1f)in", ballIndex, r.fieldPoint.x, r.fieldPoint.y);

        int fontFace = Imgproc.FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.5;
        int thickness = 1;
        int[] baseline = new int[1];
        Size textSize = Imgproc.getTextSize(label, fontFace, fontScale, thickness, baseline);

        int padding = 4;
        int plateWidth  = (int) textSize.width  + padding * 2;
        int plateHeight = (int) textSize.height + baseline[0] + padding * 2;

        // Place the plate directly above the box, matching the box width if
        // the box is wider than the text needs, otherwise sized to the text.
        int plateDrawWidth = Math.max(plateWidth, box.width);
        int plateX = box.x;
        int plateY = box.y - plateHeight;

        // Clamp so the plate never draws off the top or side edges of the frame
        if (plateY < 0) plateY = box.y + box.height + 2; // fall back to below the box
        if (plateX + plateDrawWidth > displayImage.cols()) {
            plateX = displayImage.cols() - plateDrawWidth;
        }
        if (plateX < 0) plateX = 0;

        // Solid background plate for legible text over any background
        Imgproc.rectangle(displayImage,
                new Point(plateX, plateY),
                new Point(plateX + plateDrawWidth, plateY + plateHeight),
                boxColor, -1);

        // Text, vertically centered in the plate, left-aligned with padding
        Point textOrigin = new Point(
                plateX + padding,
                plateY + plateHeight - padding - baseline[0]);
        Imgproc.putText(displayImage, label, textOrigin,
                fontFace, fontScale, textColor, thickness);
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