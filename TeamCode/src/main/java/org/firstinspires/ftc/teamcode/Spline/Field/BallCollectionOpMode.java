package org.firstinspires.ftc.teamcode.Spline.Field;

import static org.firstinspires.ftc.teamcode.Spline.Field.FieldVisualizerFlat.getRobotPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Spline.Pedro.PedroSetup;

/**
 * BallCollectionOpMode
 * ---------------------
 * Press gamepad1.a to start the automated ball-collection sequence.
 * The robot does NOT move until that button is pressed — follower.update()
 * is only called once a path is actually running.
 *
 * BUTTON BINDING
 *   gamepad1.a — start / restart the collection path from the robot's
 *                current position at the moment of the press.
 *
 * HEADING CONVENTION (Pedro Pathing 2.x)
 *   Radians, CCW positive, 0 = facing +X.
 *   The intake heading for each leg is atan2(dy, dx) pointing straight
 *   at the target, so the full 18-inch front face sweeps through the ball.
 *
 * PEDRO 2.x API NOTES
 *   - BezierLine takes Pose directly (no Point wrapper needed).
 *   - Drawing class is in the telemetry artifact and manages its own packet;
 *     we draw the live robot directly onto our Canvas instead via
 *     FieldVisualizer.drawLiveRobotOnCanvas().
 *
 * DASHBOARD
 *   One TelemetryPacket per loop: FieldVisualizer draws field objects and
 *   dashed path-status lines; drawLiveRobotOnCanvas adds the live robot
 *   circle on the same overlay; then one sendTelemetryPacket().
 */
@TeleOp(name = "Ball Collection", group = "Spline")
public class BallCollectionOpMode extends OpMode {

    // ── Pedro Pathing ─────────────────────────────────────────────────────────
    private Follower follower;

    // ── State machine ─────────────────────────────────────────────────────────
    //   IDLE    – OpMode is running but robot is stationary, waiting for A
    //   RUNNING – follower is actively executing the PathChain
    //   DONE    – PathChain finished; follower holds the end pose
    private enum State { IDLE, RUNNING, DONE }
    private State state = State.IDLE;

    private PathChain collectionChain;

    // Captured once in init(); used as the return waypoint at the end of the chain
    private Pose startPose;

    // ── Button-edge detection ─────────────────────────────────────────────────
    private boolean prevA = false;

    // =========================================================================
    //  init
    // =========================================================================
    @Override
    public void init() {
        follower = PedroSetup.createFollower(hardwareMap);

        // Seed the localizer from the Dashboard-editable robot position.
        // For a fixed competition starting pose replace with:
        //   startPose = new Pose(<x>, <y>, Math.toRadians(<deg>));
        startPose = FieldVisualizer.getRobotPose();
        follower.setStartingPose(startPose);

        // Do NOT call follower.update() here — robot must stay still until A is pressed.

        telemetry.addLine("Ready. Press A on gamepad 1 to start collection.");
        telemetry.update();
    }

    // =========================================================================
    //  loop
    // =========================================================================
    @Override
    public void loop() {

        // ── 1. Button-edge: A starts / restarts the path ──────────────────────
        boolean currA = gamepad1.a;
        if (currA && !prevA) {
            buildAndStartPath();
        }
        prevA = currA;

        // ── 2. Only drive the follower while a path is active ─────────────────
        //    Calling follower.update() in IDLE lets Pedro's position controller
        //    fight to hold its zero pose, which would move the robot immediately.
        if (state == State.RUNNING || state == State.DONE) {
            follower.update();
        }

        // ── 3. Detect path completion ─────────────────────────────────────────
        if (state == State.RUNNING && !follower.isBusy()) {
            state = State.DONE;
        }

        // ── 4. Sync FieldVisualizer robot marker with Pedro's live pose ────────
        Pose livePose = follower.getPose();
        FieldVisualizer.robotX          = livePose.getX();
        FieldVisualizer.robotY          = livePose.getY();
        FieldVisualizer.robotHeadingDeg = Math.toDegrees(livePose.getHeading());

        // ── 5. One unified Dashboard packet ───────────────────────────────────
        //    FieldVisualizer.draw() applies the canvas transform and draws all
        //    field objects. drawLiveRobotOnCanvas() then draws the Pedro robot
        //    circle on the same already-transformed overlay. One send at the end.
        TelemetryPacket packet = new TelemetryPacket();
        FieldVisualizer.draw(packet);
        FieldVisualizer.drawLiveRobotOnCanvas(packet.fieldOverlay(), livePose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        // ── 6. Driver Hub telemetry ───────────────────────────────────────────
        telemetry.addData("State",   state);
        telemetry.addData("X",       String.format("%.1f in", livePose.getX()));
        telemetry.addData("Y",       String.format("%.1f in", livePose.getY()));
        telemetry.addData("Heading", String.format("%.1f deg",
                Math.toDegrees(livePose.getHeading())));
        telemetry.addLine("--- Collection Order ---");
        Ball[] order = FieldVisualizer.getCollectionOrder();
        for (int i = 0; i < order.length; i++) {
            telemetry.addData("  Pick " + (i + 1), order[i].toString());
        }
        if (state == State.IDLE) {
            telemetry.addLine(">> Press A to begin.");
        } else if (state == State.DONE) {
            telemetry.addLine(">> Done! Press A to run again.");
        }
        telemetry.update();
    }

    // =========================================================================
    //  buildAndStartPath
    //  Route: Robot → Ball[0] → Ball[1] → Ball[2] → StartPose
    //  Pedro 2.x: BezierLine takes Pose directly, no Point wrapper.
    // =========================================================================
    private void buildAndStartPath() {
        Ball[] order    = FieldVisualizer.getCollectionOrder();
        Pose   robotNow = getRobotPose();

        double h0      = intakeHeading(robotNow.getX(), robotNow.getY(),
                order[0].x,      order[0].y);
        double h1      = intakeHeading(order[0].x, order[0].y,
                order[1].x, order[1].y);
        double h2      = intakeHeading(order[1].x, order[1].y,
                order[2].x, order[2].y);
        double hReturn = intakeHeading(order[2].x, order[2].y,
                startPose.getX(), startPose.getY());

        Pose ball0Pose  = new Pose(order[0].x, order[0].y, h0);
        Pose ball1Pose  = new Pose(order[1].x, order[1].y, h1);
        Pose ball2Pose  = new Pose(order[2].x, order[2].y, h2);
        Pose returnPose = new Pose(startPose.getX(), startPose.getY(), hReturn);

        // Pedro 2.x: BezierLine(Pose, Pose) — no Point wrapper required
        collectionChain = follower.pathBuilder()

                .addPath(new BezierLine(robotNow,  ball0Pose))
                .setLinearHeadingInterpolation(robotNow.getHeading(),  ball0Pose.getHeading())

                .addPath(new BezierLine(ball0Pose, ball1Pose))
                .setLinearHeadingInterpolation(ball0Pose.getHeading(), ball1Pose.getHeading())

                .addPath(new BezierLine(ball1Pose, ball2Pose))
                .setLinearHeadingInterpolation(ball1Pose.getHeading(), ball2Pose.getHeading())

                .addPath(new BezierLine(ball2Pose, returnPose))
                .setLinearHeadingInterpolation(ball2Pose.getHeading(), returnPose.getHeading())

                .build();

        follower.followPath(collectionChain, true);
        state = State.RUNNING;
    }

    // =========================================================================
    //  intakeHeading — atan2 angle pointing the robot front toward (toX, toY)
    // =========================================================================
    private static double intakeHeading(double fromX, double fromY,
                                        double toX,   double toY) {
        return Math.atan2(toY - fromY, toX - fromX);
    }
}