package org.firstinspires.ftc.teamcode.purepursuit.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.purepursuit.Robot;
import org.firstinspires.ftc.teamcode.purepursuit.math.Pose;
import org.firstinspires.ftc.teamcode.purepursuit.pid.SquIDController;

@Config
@TeleOp
public class SquidTuner extends LinearOpMode {
    public static double kSQx = 0.02;
    public static double kSQy = 0.02;
    public static double polynomial = 1.8;

    public static double hP = 0.6, hI = 0, hD = 0.05;

    public static double targetX = 0, targetY = 0, targetH = 0;
    public static boolean random = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        bot.setPose(new Pose());

        bot.purePursuit.tuning = true;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bot.purePursuit.kSQx = kSQx;
            bot.purePursuit.kSQy = kSQy;
            bot.purePursuit.hPID = new PIDCoefficients(hP, hI, hD);
            bot.purePursuit.singlePIDtoPoint(new Pose(targetX, targetY, targetH));
            SquIDController.polynomial = polynomial;

            if (random) {
                random = false;
                targetX = 48*Math.random() - 24;
                targetY = 48*Math.random() - 24;
                targetH = 2*Math.PI* Math.random();
            }

            telemetry.addData("TargetX", targetX);
            telemetry.addData("TargetY", targetY);
            telemetry.addData("TargetH", targetH);

            telemetry.addData("PosX", bot.purePursuit.pose.x);
            telemetry.addData("PosY", bot.purePursuit.pose.y);
            telemetry.addData("PosH", bot.purePursuit.pose.h);

            telemetry.update();
        }
    }
}
