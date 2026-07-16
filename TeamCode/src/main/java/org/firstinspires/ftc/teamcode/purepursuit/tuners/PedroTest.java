package org.firstinspires.ftc.teamcode.purepursuit.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Spline.Pedro.PedroSetup;
import org.firstinspires.ftc.teamcode.purepursuit.Robot;
import org.firstinspires.ftc.teamcode.purepursuit.math.Bezier;
import org.firstinspires.ftc.teamcode.purepursuit.math.BezierPath;
import org.firstinspires.ftc.teamcode.purepursuit.math.PurePursuit;


@TeleOp(group = "purepursuit")
@Config
public class PedroTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        Follower follower = PedroSetup.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int asdf = 0;

        PathChain spline = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(0,0),
                        new Pose(36, 0),
                        new Pose(0, 36),
                        new Pose(36, 36)))
                .setTangentHeadingInterpolation().build();

        PathChain line = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(0,0), new Pose(86, 0)))
                .setTangentHeadingInterpolation().build();
        PathChain line2 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(86,0), new Pose(0, 0)))
                .setTangentHeadingInterpolation().setReversed().build();


        PathChain spline2 = follower.pathBuilder().addPath(
                new BezierCurve(new Pose(36,36),
                        new Pose(0, 36),
                        new Pose(36, 0),
                        new Pose(0, 0)))
                .setTangentHeadingInterpolation().setReversed().build();

        PathChain l_spline = follower.pathBuilder().addPath(
                        new BezierCurve(new Pose(0,0),
                                new Pose(48, -6),
                                new Pose(60, 6),
                                new Pose(60, 40)))
                .setTangentHeadingInterpolation().build();

        PathChain l_spline2 = follower.pathBuilder().addPath(
                        new BezierCurve(new Pose(60,40),
                                new Pose(60, 6),
                                new Pose(48, -6),
                                new Pose(0, 0)))
                .setTangentHeadingInterpolation().setReversed().build();

        waitForStart();

//        while (opModeIsActive() && !isStopRequested()) {
//            follower.update();
//
//            switch (asdf) {
//                case 0: {
//                    if (!follower.isBusy()) {
//                        follower.followPath(l_spline);
//                        asdf = 1;
//                    }
//                    break;
//                }
//                case 1: {
//                    if (!follower.isBusy()) {
//                        follower.followPath(l_spline2);
//                        asdf = 0;
//                    }
//                    break;
//                }
//            }
//
//        }

        follower.followPath(l_spline);

        while (follower.isBusy()) {
            follower.update();
        }

        Pose finalPose = follower.getPose();
        telemetry.addData("FinalX", finalPose.getX());
        telemetry.addData("FinalY", finalPose.getY());
        telemetry.addData("FinalH", finalPose.getHeading());
        telemetry.update();

    }
}
