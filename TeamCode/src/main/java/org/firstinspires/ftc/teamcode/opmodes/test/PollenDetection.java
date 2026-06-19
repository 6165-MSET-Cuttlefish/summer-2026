package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCVPipelines.PollenDetectionPipeline.PollenDetectionPipeline;
import org.firstinspires.ftc.teamcode.OpenCVPipelines.WebcamControls;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

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
        webcam.setPipeline(new PollenDetectionPipeline(telemetry));

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                // OV9782 only offers 640x480 in MJPEG, not the default uncompressed YUY2.
                webcam.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT,
                        OpenCvWebcam.StreamFormat.MJPEG);
                FtcDashboard.getInstance().startCameraStream(webcam, DASHBOARD_FPS);
                controls = new WebcamControls(webcam);
            }

            @Override public void onError(int errorCode) {
                telemetry.addData("Camera open error", errorCode);
                telemetry.update();
            }
        });

        // Pipeline drives telemetry from the camera thread; here we only pump camera controls.
        while (opModeInInit()) pump();
        while (opModeIsActive()) pump();

        FtcDashboard.getInstance().stopCameraStream();
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    private void pump() {
        if (controls != null) controls.update();
        sleep(20);
    }
}
