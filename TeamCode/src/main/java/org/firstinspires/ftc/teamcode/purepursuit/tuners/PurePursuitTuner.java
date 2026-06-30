package org.firstinspires.ftc.teamcode.purepursuit.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.purepursuit.Robot;
import org.firstinspires.ftc.teamcode.purepursuit.math.Bezier;
import org.firstinspires.ftc.teamcode.purepursuit.math.BezierPath;
import org.firstinspires.ftc.teamcode.purepursuit.math.Pose;
import org.firstinspires.ftc.teamcode.purepursuit.math.PurePursuit;


@TeleOp
@Config
public class PurePursuitTuner extends LinearOpMode {
    public static double searchRad = 8;
    public static double maxPower = 0.4;
    public static double kSQx = 0.06, kSQy = 0.06, kF = 0.003;
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
        BezierPath curve2 = PurePursuit.pathBuilder
                .addBezier(PurePursuit.builder
                        .addControlPoint(0, 36)
                        .addControlPoint(36, 36)
                        .build())
                .addBezier(PurePursuit.builder
                        .addControlPoint(36,36)
                        .addControlPoint(52, 18)
                        .addControlPoint(36,0)
                        .build())
                .addBezier(PurePursuit.builder
                        .addControlPoint(36, 0)
                        .addControlPoint(0,0)
                        .build()).build();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bot.purePursuit.searchRad = searchRad;
            bot.purePursuit.maxPower = maxPower;
            bot.purePursuit.kSQx = kSQx;
            bot.purePursuit.kSQy = kSQy;
            bot.purePursuit.kF = kF;
            Actions.runBlocking(bot.followPath(curve));
            Actions.runBlocking(bot.followPathReversed(curve2));
        }
//        Actions.runBlocking(bot.followPathConstantHeading(curve, 0));

    }
}
