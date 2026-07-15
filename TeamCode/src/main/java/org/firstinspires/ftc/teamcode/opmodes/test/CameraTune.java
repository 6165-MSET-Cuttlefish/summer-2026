package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpenCVPipelines.WebcamControls;
import org.firstinspires.ftc.teamcode.OpenCVPipelines.WebcamSession;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * Camera-tuning OpMode for the Control Hub. Opens the webcam, streams the raw feed to FtcDashboard,
 * and applies the live {@link WebcamControls} exposure / gain / white-balance sliders — the fix for
 * the washed-out / white Pollen. Open FtcDashboard, watch the camera, and edit the WebcamControls.*
 * fields in the config; the stream updates immediately (works in init too).
 *
 * <p>Run on the robot — EOCV-Sim never opens an OpenCvWebcam, so the controls only take effect here.
 */
@TeleOp(name = "Camera Tune", group = "test")
public class CameraTune extends LinearOpMode {

    // Matches the webcam name in camera.xml / decode.xml (the Arducam UC-852 / OV9782).
    private static final String WEBCAM_NAME = "nerdDetector";

    private WebcamSession session;

    @Override
    public void runOpMode() {
        session = new WebcamSession(hardwareMap, telemetry, WEBCAM_NAME, new PassThrough());

        while (opModeInInit()) pump();   // tune before pressing play — the stream is already live
        while (opModeIsActive()) pump();

        session.close();
    }

    private void pump() {
        session.update();
        telemetry.addLine("Edit WebcamControls.* in the FtcDashboard config.");
        telemetry.addData("Mode", WebcamControls.manual ? "MANUAL" : "AUTO");
        telemetry.addData("Exposure (ms)", WebcamControls.exposureMs);
        telemetry.addData("Gain", WebcamControls.gain);
        telemetry.addData("White balance (K)", WebcamControls.whiteBalanceK);
        telemetry.addData("Camera FPS", "%.1f", session.webcam().getFps());
        telemetry.update();
        sleep(20);
    }

    /** Pass the camera frame straight through — this OpMode is for judging exposure/color, not detection. */
    private static class PassThrough extends OpenCvPipeline {
        @Override public Mat processFrame(Mat input) {
            return input;
        }
    }
}
