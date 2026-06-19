package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCVPipelines.WebcamControls;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

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
    private static final int WIDTH = 640;
    private static final int HEIGHT = 480;
    private static final int DASHBOARD_FPS = 30;

    private OpenCvWebcam webcam;
    private WebcamControls controls;

    @Override
    public void runOpMode() {
        WebcamName name = hardwareMap.get(WebcamName.class, WEBCAM_NAME);
        int viewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(name, viewId);
        webcam.setPipeline(new PassThrough());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                // OV9782 only offers 640x480 in MJPEG, not the default uncompressed YUY2.
                webcam.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT,
                        OpenCvWebcam.StreamFormat.MJPEG);
                FtcDashboard.getInstance().startCameraStream(webcam, DASHBOARD_FPS);
                // Controls need an open, streaming camera, so build the helper here.
                controls = new WebcamControls(webcam);
            }

            @Override public void onError(int errorCode) {
                telemetry.addData("Camera open error", errorCode);
                telemetry.update();
            }
        });

        while (opModeInInit()) pump();   // tune before pressing play — the stream is already live
        while (opModeIsActive()) pump();

        FtcDashboard.getInstance().stopCameraStream();
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    private void pump() {
        if (controls != null) controls.update();
        telemetry.addLine("Edit WebcamControls.* in the FtcDashboard config.");
        telemetry.addData("Mode", WebcamControls.manual ? "MANUAL" : "AUTO");
        telemetry.addData("Exposure (ms)", WebcamControls.exposureMs);
        telemetry.addData("Gain", WebcamControls.gain);
        telemetry.addData("White balance (K)", WebcamControls.whiteBalanceK);
        if (webcam != null) telemetry.addData("Camera FPS", "%.1f", webcam.getFps());
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
