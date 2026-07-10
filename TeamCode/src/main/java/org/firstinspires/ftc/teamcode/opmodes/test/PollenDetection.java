package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpenCVPipelines.PollenDetectionPipeline.PollenDetectionPipeline;
import org.firstinspires.ftc.teamcode.OpenCVPipelines.WebcamControls;
import org.firstinspires.ftc.teamcode.OpenCVPipelines.WebcamSession;

/**
 * Runs the Pollen detection pipeline on the Control Hub — the on-robot equivalent of selecting the
 * pipeline in EOCV-Sim. Opens the webcam (MJPEG), streams the pipeline output to FtcDashboard, and
 * applies the live {@link WebcamControls} exposure/gain/WB sliders.
 *
 * <p>Everything is tunable from FtcDashboard while it runs:
 * <ul>
 *   <li>camera — {@code WebcamControls.*} (exposure / gain / white balance, manual on/off)</li>
 *   <li>detection — {@code PollenDetectionPipeline.Tuning.*} (HSV gate, minPeakDist, and the
 *       MASK/OVERLAY {@code displayMode} dropdown)</li>
 * </ul>
 *
 * <p>The pipeline owns telemetry (it prints calibration status / detections each frame), so this
 * OpMode's loop only pumps the camera controls.
 */
@TeleOp(name = "Pollen Detection", group = "test")
@Config
public class PollenDetection extends LinearOpMode {

    // Matches the webcam name in camera.xml / decode.xml (the Arducam UC-852 / OV9782).
    private static final String WEBCAM_NAME = "nerdDetector";

    private WebcamSession session;

    @Override
    public void runOpMode() {
        session = new WebcamSession(hardwareMap, telemetry, WEBCAM_NAME,
                new PollenDetectionPipeline(telemetry));

        // Pipeline drives telemetry from the camera thread; here we only pump camera controls.
        while (opModeInInit()) pump();
        while (opModeIsActive()) pump();

        session.close();
    }

    private void pump() {
        session.update();
        sleep(20);
    }
}
