package org.firstinspires.ftc.teamcode.Spline.Field;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Spline.Pedro.PedroSetup;

/**
 * Straight-line sanity test - isolates Pedro Pathing / drivetrain / localizer
 * from all the ball-collection planning logic.
 *
 * Drives a single 24-inch BezierLine straight ahead (no curve, no heading
 * change, no obstacle logic, no FSM beyond "drive it, then stop"). If this
 * doesn't move the robot, the problem is in PedroSetup / RobotHardwareConfig
 * / wiring - not in BallCollectionOpMode's path-building code.
 *
 * USAGE:
 *   1. Place the robot somewhere with ~30 in of clear space in front of it.
 *   2. Run this OpMode, press init, then start.
 *   3. Set run=1 on the Dashboard (Configuration tab -> StraightLineTest).
 *   4. Watch: does the robot physically move forward ~24 inches and stop?
 *      Telemetry below reports isBusy(), pose, and velocity every loop so
 *      you can see whether Pedro thinks it's moving even if it isn't.
 */
@Config
@TeleOp(name = "Straight Line Test", group = "Spline")
public class StraightLineTestOpMode extends OpMode {

    public static double TEST_DISTANCE_IN = 24.0; // how far straight ahead to drive
    public static double TIMEOUT_SEC = 6.0;
    public static int run = 0; // set to 1 to start

    private Follower follower;
    private PathChain testPath;
    private Pose startPose;
    private Pose endPose;

    private boolean running = false;
    private boolean done = false;
    private boolean prevRun = false;
    private double startTimeSec;

    @Override
    public void init() {
        follower = PedroSetup.createFollower(hardwareMap);

        // Fixed, known starting pose - not tied to FieldVisualizer/Dashboard
        // ball config, so nothing from the ball-collection code can affect
        // this test.
        startPose = new Pose(0, 0, 0);
        endPose = new Pose(TEST_DISTANCE_IN, 0, 0);
        follower.setStartingPose(startPose);

        testPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setTangentHeadingInterpolation()
                .build();

        telemetry.addLine("Straight line test ready.");
        telemetry.addLine("Set run=1 on Dashboard to drive " + TEST_DISTANCE_IN + " in forward.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setStartingPose(startPose); // re-seed after Pinpoint calibration
        prevRun = (run == 1);
    }

    @Override
    public void loop() {
        boolean startTriggered = (run == 1) && !prevRun;
        prevRun = (run == 1);

        if (startTriggered && !running && !done) {
            follower.followPath(testPath, false);
            running = true;
            startTimeSec = getRuntime();
        }

        if (running) {
            follower.update();

            boolean timedOut = (getRuntime() - startTimeSec) > TIMEOUT_SEC;
            if (!follower.isBusy() || timedOut) {
                follower.breakFollowing();
                running = false;
                done = true;
            }
        }

        Pose pose = follower.getPose();
        double speed = follower.getVelocity().getMagnitude();

        telemetry.addData("State", running ? "RUNNING" : done ? "DONE" : "IDLE (set run=1)");
        telemetry.addData("isBusy()", follower.isBusy());
        telemetry.addData("X", String.format("%.2f in", pose.getX()));
        telemetry.addData("Y", String.format("%.2f in", pose.getY()));
        telemetry.addData("Heading", String.format("%.1f deg", Math.toDegrees(pose.getHeading())));
        telemetry.addData("Speed", String.format("%.2f in/s", speed));
        telemetry.addData("Target X", String.format("%.2f in", endPose.getX()));
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#2962FF");
        packet.fieldOverlay().strokeLine(startPose.getX(), startPose.getY(), endPose.getX(), endPose.getY());
        packet.fieldOverlay().setFill("#4CAF50");
        packet.fieldOverlay().fillCircle(pose.getX(), pose.getY(), 3);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}