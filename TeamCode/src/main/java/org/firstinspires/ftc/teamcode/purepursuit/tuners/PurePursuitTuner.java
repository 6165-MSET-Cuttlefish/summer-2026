package org.firstinspires.ftc.teamcode.purepursuit.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.purepursuit.Robot;
import org.firstinspires.ftc.teamcode.purepursuit.math.Bezier;
import org.firstinspires.ftc.teamcode.purepursuit.math.BezierPath;
import org.firstinspires.ftc.teamcode.purepursuit.math.Pose;
import org.firstinspires.ftc.teamcode.purepursuit.math.PurePursuit;


@TeleOp(group = "purepursuit")
@Config
public class PurePursuitTuner extends LinearOpMode {
    public static double searchRad = 8;
    public static double maxPower = 0.65;
    public static double kSQx = 0.05, kSQy = 0.05, kF = 0.1;
    public static double hP = 1.5, hI = 0, hD = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        bot.setPose(new Pose(0, 0, Math.toRadians(0)));
        bot.purePursuit.tuning = false;

        bot.purePursuit.searchRad = searchRad;
        bot.purePursuit.maxPower = maxPower;
        bot.purePursuit.kSQx = kSQx;
        bot.purePursuit.kSQy = kSQy;
        bot.purePursuit.kF = kF;
        bot.purePursuit.hPID = new PIDCoefficients(hP, hI, hD);



        BezierPath curve = PurePursuit.pathBuilder
                .addBezier(PurePursuit.builder
                        .addControlPoint(0,0)
                        .addControlPoint(36, 0)
                        .build())
                .addBezier(PurePursuit.builder
                        .addControlPoint(36, 0)
                        .addControlPoint(52, 18)
                        .addControlPoint(36,36)
                        .build())
                .addBezier(PurePursuit.builder
                        .addControlPoint(36, 36)
                        .addControlPoint(0, 36)
                        .build()).build();

        Bezier spline = PurePursuit.builder
                .addControlPoint(0,0)
                .addControlPoint(36,0)
                .addControlPoint(0, 36)
                .addControlPoint(36,36)
                .build();

        Bezier line = PurePursuit.builder
                .addControlPoint(0, 0)
                .addControlPoint(86, 0)
                .build();

        Bezier l_spline = PurePursuit.builder
                .addControlPoint(0,0)
                .addControlPoint(48,-6)
                .addControlPoint(60, 6)
                .addControlPoint(60, 40)
                .build();
        waitForStart();

//        while (opModeIsActive() && !isStopRequested()) {
//            bot.purePursuit.searchRad = searchRad;
//            bot.purePursuit.maxPower = maxPower;
//            bot.purePursuit.kSQx = kSQx;
//            bot.purePursuit.kSQy = kSQy;
//            bot.purePursuit.kF = kF;
//            bot.purePursuit.hPID = new PIDCoefficients(hP, hI, hD);
//            Actions.runBlocking(bot.followPath(l_spline));
//            Actions.runBlocking(bot.followPathReversed(l_spline.reverse()));
//        }
        Actions.runBlocking(bot.followPath(l_spline));
        bot.purePursuit.mecDrive.setPowers(0, 0, 0, 0);
        Pose finalPose = bot.purePursuit.pose;
        telemetry.addData("FinalX", finalPose.x);
        telemetry.addData("FinalY", finalPose.y);
        telemetry.addData("FinalH", finalPose.h);
        telemetry.update();

    }
}
