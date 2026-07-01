package org.firstinspires.ftc.teamcode.purepursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.purepursuit.Robot;
import org.firstinspires.ftc.teamcode.purepursuit.actions.ActionList;
import org.firstinspires.ftc.teamcode.purepursuit.math.Bezier;
import org.firstinspires.ftc.teamcode.purepursuit.math.BezierPath;
import org.firstinspires.ftc.teamcode.purepursuit.math.Pose;
import org.firstinspires.ftc.teamcode.purepursuit.math.PurePursuit;


@TeleOp(group = "purepursuit")
@Config
public class Blue18BallPath extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        bot.setPose(new Pose(-56, -48, Math.toRadians(232)));
        bot.purePursuit.tuning = false;
        ActionList a = new ActionList(bot);

        Bezier score1 = PurePursuit.builder
                .addControlPoint(-56,-48)
                .addControlPoint(-24,-24)
                .build();

        Bezier intake1 = PurePursuit.builder
                .addControlPoint(-24,-24)
                .addControlPoint(-3.6,-2.9)
                .addControlPoint(18.8, -9.8)
                .addControlPoint(12,-54)
                .build();
        Bezier score2 = PurePursuit.builder
                .addControlPoint(12,-54)
                .addControlPoint(-24,-24)
                .build();
        Bezier gate = PurePursuit.builder
                .addControlPoint(-24, -24)
                .addControlPoint(-1.2, -22.3)
                .addControlPoint(31, -44.8)
                .addControlPoint(10.5, -58)
                .build();
        Bezier intake2 = PurePursuit.builder
                .addControlPoint(-24,-24)
                .addControlPoint(19.8, 3.3)
                .addControlPoint(44, -30.4)
                .addControlPoint(36, -54)
                .build();
        Bezier score3 = PurePursuit.builder
                .addControlPoint(36,-54)
                .addControlPoint(-24, -24)
                .build();
        Bezier intake3 = PurePursuit.builder
                .addControlPoint(-24,-24)
                .addControlPoint(-3.5,-29)
                .addControlPoint(-12,-52)
                .build();
        Bezier score4 = PurePursuit.builder
                .addControlPoint(-12,-52)
                .addControlPoint(-24,-24)
                .build();


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

        Actions.runBlocking(a.combine(
                bot.followPathReversed(score1),
                bot.followPath(intake1),
                bot.followPathReversed(score2),
                bot.followPath(gate),
                bot.followPathReversed(gate.reverse()),
                bot.followPath(gate),
                bot.followPathReversed(gate.reverse()),
                bot.followPath(intake2),
                bot.followPathReversed(score3),
                bot.followPath(intake3),
                bot.followPathReversed(score4)
        ));

    }
}
