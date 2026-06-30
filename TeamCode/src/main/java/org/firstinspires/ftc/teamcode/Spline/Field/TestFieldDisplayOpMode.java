package org.firstinspires.ftc.teamcode.Spline.Field;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Spline.Field.FieldVisualizer;

// If you want to draw the robot/path on the same packet using Pedro's own
// Drawing class (the one in Tuning.java from the quickstart), uncomment:
// import com.pedropathing.follower.Follower;

/**
 * Minimal example showing how to push the FieldVisualizer state to FTC
 * Dashboard every loop. Run this OpMode, then open the Dashboard's
 * Configuration tab to drag ball/obstacle positions around and watch the
 * Field tab update live.
 *
 * Once you have a Follower set up, replace the body of loop() with the
 * single-packet pattern below so the robot/path and the balls/obstacles
 * show up together instead of overwriting each other:
 *
 *   TelemetryPacket packet = new TelemetryPacket();
 *   FieldVisualizer.draw(packet);
 *   Drawing.drawRobotOnCanvas(packet.fieldOverlay(), follower.getPose());
 *   FtcDashboard.getInstance().sendTelemetryPacket(packet);
 */
@TeleOp(name = "Test Field Display", group = "Test")
public class TestFieldDisplayOpMode extends OpMode {

    @Override
    public void init() {
        // nothing to set up; FieldVisualizer holds static state
    }

    @Override
    public void loop() {
        FieldVisualizer.update();

        telemetry.addData("Ball1", FieldVisualizer.ball1);
        telemetry.addData("Ball2", FieldVisualizer.ball2);
        telemetry.addData("Ball3", FieldVisualizer.ball3);
        telemetry.addData("Robot", "(%.1f, %.1f) @ %.0f deg",
                FieldVisualizer.robotX, FieldVisualizer.robotY, FieldVisualizer.robotHeadingDeg);
        telemetry.update();
    }
}