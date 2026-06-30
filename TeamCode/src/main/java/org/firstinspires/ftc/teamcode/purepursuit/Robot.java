package org.firstinspires.ftc.teamcode.purepursuit;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.purepursuit.actions.LoopAction;
import org.firstinspires.ftc.teamcode.purepursuit.actions.PurePursuitAction;
import org.firstinspires.ftc.teamcode.purepursuit.actions.WaitUntilAction;
import org.firstinspires.ftc.teamcode.purepursuit.MecanumDrivePurePursuit;
import org.firstinspires.ftc.teamcode.purepursuit.math.Bezier;
import org.firstinspires.ftc.teamcode.purepursuit.math.BezierPath;
import org.firstinspires.ftc.teamcode.purepursuit.math.Maths;
import org.firstinspires.ftc.teamcode.purepursuit.math.Path;
import org.firstinspires.ftc.teamcode.purepursuit.math.Pose;
import org.firstinspires.ftc.teamcode.purepursuit.math.PurePursuit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class Robot {

    public MecanumDrivePurePursuit drive;
    public DcMotorEx leftIntake, rightIntake, intake, shooter, turret;
    public DcMotorEx transfer;
    public Servo hood, hardstop;
    public WebcamName camera;
    public AprilTagProcessor tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
    public List<AprilTagDetection> detections;
    public DistanceSensor distance, intakeDistance;
    public static Pose2d autoPose = new Pose2d(0, 0, 0);
    public static double autoTurretPos = 0;

    public PurePursuit purePursuit;

    public Telemetry telemetry;

    public Gamepad gamepad1, gamepad2;
    public VoltageSensor voltage;

    public static boolean shooting = false;

    public static double p = 0.0035, i = 0, d = 0, f = 0.000205;
    public static double lastP = p, lastI = i, lastD = d, lastF = f;

    public static double FILTER_CUTOFF = 1.5;   // Hz (adjustable in dashboard)
    private double filteredTicksPerSec = 0;
    public static double rawRpm = 0;
//    public static double tP = -0.001, tI = 0, tD = 0;

    public static double tP = 0.01, tI = 0, tD = 0;
    public static double hP = 0.04, hI = 0, hD = 0;
    public double turretPos = 0;

    public static double hoodMax = 0.82, hoodMin = 0.73;

    public static double turretOffset = 0;

    public static double leftOffset = -1.5, rightOffset = 1;
    public PIDController hPID = new PIDController(hP, hI, hD);
    public static boolean atagAlign = false;

    public static double ticksPerRev = 8192.0;

    public double lastTicks = 0, thisTicks = 0;
    public static boolean closeMode = false;

    public static double ballDist = 0;

    public static double targetVel = 0;
    public static double intakePower = 0;
    public static double transferPower = 0;
    public static double turretTarget = 0;

    public static double rpm = 0;

    public double lastTime = 0;

    public static boolean runScoringLoop = true;
    public static boolean runCheckLoop = false;

    public PIDFController shooterPID = new PIDFController(p, i, d, f);
    public PIDController turretPID = new PIDController(tP, tI, tD);

    public static double TAG_HEIGHT = 38.75-9.25;
    public static Vector2d RED_GOAL_TAG = new Vector2d(-58.27, 55.63);
    public static Vector2d BLUE_GOAL_TAG = new Vector2d(-58.27, -55.63);

    public static double ticksToDegrees = 90.0/500.0;
    public static int closeRPM = 2675;
    public static int farRPM = 3700;
    public static double HardstopOpen = 0.97;
    public static double HardstopClose = 0.8;

    public static int ballCount = 0;
    public static boolean ShouldTurn = false;

    public static double heading;

    public static double tps0 = 0;
    public static double tps1 = 0;
    public static double tps2 = 0;
    public double dt;
    public ElapsedTime shooterTimer = new ElapsedTime();

    // Sliding window for moving average
    private static final int VELOCITY_WINDOW_SIZE = 5;
    private final double[] velocityWindow = new double[VELOCITY_WINDOW_SIZE];
    private int velocityWindowIdx = 0;

    private static final int VELOCITY_HISTORY_SIZE = 5;
    private final double[] tickHistory = new double[VELOCITY_HISTORY_SIZE];
    private final double[] timeHistory = new double[VELOCITY_HISTORY_SIZE];
    private int historyIdx = 0;

    public static Vector2d blueGoalClose = new Vector2d(-67, -64);
    public static Vector2d blueGoalFar = new Vector2d(-86, -64);


    public static Vector2d redGoalClose = new Vector2d(-61, 64);
    public static Vector2d redGoalFar = new Vector2d(-65, 64);

    public ElapsedTime timer = new ElapsedTime();

    public Robot(HardwareMap hardwareMap) {
        targetVel = 0;
        intakePower = 0;
        transferPower = 0;
        ballDist = 0;
        ballCount = 0;
        turretTarget = 0;
        drive = new MecanumDrivePurePursuit(hardwareMap, new Pose2d(0, 0, 0));

        purePursuit = new PurePursuit(drive);

//        intake = hardwareMap.get(DcMotorEx.class, "intake");
//        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
//
//        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
//        turret = hardwareMap.get(DcMotorEx.class, "turret");

//        distance = hardwareMap.get(DistanceSensor.class, "distance");
        voltage = hardwareMap.voltageSensor.iterator().next();

//        hood = hardwareMap.get(Servo.class, "hood");
//        hardstop = hardwareMap.get(Servo.class, "hardstop");
//
//        camera = hardwareMap.get(WebcamName.class, "camera");


//        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
//        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//
//        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterPID.setPIDF(p, i, d, f);
        turretPID.setPID(tP, tI, tD);

        timer.reset();

//        lastTicks = shooter.getCurrentPosition();
//        turretPos = turret.getCurrentPosition();
//        hood.setPosition(0.17);

        dt = timer.seconds();
        shooterTimer.reset();

    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }


    public void setPose(Pose pose) {
        drive.localizer.setPose(pose.toPose2d());
        purePursuit.mecDrive = drive;
    }
    public Action followPath(Path path) {
        return new PurePursuitAction(purePursuit, path);
    }
    public Action followPathConstantHeading(Path path, double heading) {
        PurePursuitAction action = new PurePursuitAction(purePursuit, path);
        action.setConstantHeading(heading);
        return action;
    }
    public Action followPathReversed(Path path) {
        PurePursuitAction action = new PurePursuitAction(purePursuit, path);
        action.setReversed(true);
        return action;
    }


    public Action scoringLoop() {
        return new ScoringLoop();
    }
    public void setShooterVelocity(int vel) {
        targetVel = vel;
    }
    public double batteryScale(double num) {
        return (12.5/voltage.getVoltage()) * num;
    }
    public void scoringLoopTele() {

        // --- Read encoders ---
        thisTicks = shooter.getCurrentPosition();
        turretPos = turret.getCurrentPosition();

        // --- Update PID if changed ---
        if (p != lastP || i != lastI || d != lastD || f != lastF) {
            shooterPID.setPIDF(p, i, d, f);
            lastP = p;
            lastI = i;
            lastD = d;
            lastF = f;
        }

        // --- Time step (FIXED, no reset) ---
        double now = shooterTimer.seconds();
        dt = now - lastTime;
        lastTime = now;

        // Clamp dt to avoid spikes
        dt = Math.max(dt, 0.005);

        // --- Velocity estimate ---
        double rawTicksPerSec = (thisTicks - lastTicks) / dt;

        // --- Simple LPF (NO RC math) ---
        double alpha = 0.15; // tune 0.1–0.25
        filteredTicksPerSec += alpha * (rawTicksPerSec - filteredTicksPerSec);

        if (!closeMode) rpm = rawTicksPerSec * 60.0 / ticksPerRev;
        else rpm = filteredTicksPerSec * 60.0 / ticksPerRev;

        // --- Shooter PID ---
        double shooterPower = shooterPID.calculate(rpm, targetVel);

        // Clamp output
        shooterPower = Math.max(-1.0, Math.min(1.0, shooterPower));

        if (targetVel == 0) shooterPower = 0;

        shooter.setPower(shooterPower);

        // --- Other subsystems ---
        transfer.setPower(transferPower);
        intakePower(intakePower);

        autoTurretPos = turretPos;

        turretPID.setPID(tP, tI, tD);
        turret.setPower(Math.min(turretPID.calculate(turretPos, Maths.clamp(turretTarget, -610, 610)), 0.8));

        lastTicks = thisTicks;
    }


    public double regressShooter(double distanceToGoal) {
        return 0.637786*Math.pow(distanceToGoal, 2) + -90.66597*distanceToGoal + 5822.03677;
    }


    public class ScoringLoop implements Action {
        public double dt;
        public ScoringLoop() {
            timer.reset();
            dt = timer.seconds();
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            // --- Read encoders ---
            thisTicks = shooter.getCurrentPosition();
            turretPos = turret.getCurrentPosition();

            // --- Update PID if changed ---
            if (p != lastP || i != lastI || d != lastD || f != lastF) {
                shooterPID.setPIDF(p, i, d, f);
                lastP = p;
                lastI = i;
                lastD = d;
                lastF = f;
            }

            // --- Time step (FIXED, no reset) ---
            double now = shooterTimer.seconds();
            dt = now - lastTime;
            lastTime = now;

            // Clamp dt to avoid spikes
            dt = Math.max(dt, 0.005);

            // --- Velocity estimate ---
            double rawTicksPerSec = (thisTicks - lastTicks) / dt;

            // --- Simple LPF (NO RC math) ---
            double alpha = 0.15; // tune 0.1–0.25
            filteredTicksPerSec += alpha * (rawTicksPerSec - filteredTicksPerSec);

            rpm = filteredTicksPerSec * 60.0 / ticksPerRev;

            // --- Shooter PID ---
            double shooterPower = shooterPID.calculate(rpm, targetVel);

            // Clamp output
            shooterPower = Math.max(-1.0, Math.min(1.0, shooterPower));

            if (targetVel == 0) shooterPower = 0;

            shooter.setPower(shooterPower);

            // --- Other subsystems ---
            transfer.setPower(transferPower);
            intakePower(intakePower);

            turretPID.setPID(tP, tI, tD);
            turret.setPower(turretPID.calculate(turretPos, Maths.clamp(turretTarget, -610, 610)));

            lastTicks = thisTicks;

            telemetry.addData("Filtered Rpm", rpm);
            telemetry.addData("TargetVel", targetVel);
            telemetry.update();

            autoPose = drive.localizer.getPose();

            return Robot.runScoringLoop;
        }

    }
    public double regressF(double targetVel) {
        if (targetVel != 0) return (0.0530089/(targetVel)) + 0.000192333;
        else return 0;
    }
    public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
    public void transferPower(double transferPower) {
        transfer.setPower(transferPower);
        Robot.transferPower = transferPower;
    }

//    public Action autoCheckTransfer() {
//        return new LoopAction(() -> {
//            if (runCheckLoop) checkTransferTele();
//        });
//    }
//    public void checkTransferTele() {
//        double intakeDist = intakeDistance.getDistance(DistanceUnit.MM);
//        Robot.ballDist = distance.getDistance(DistanceUnit.MM);
//
//
//        if (Robot.ballDist > 0 && Robot.ballDist < 40 && timer.seconds() > 0.75) {
//            if (ballCount < 1) turnTransfer();
//
//            timer.reset();
//
//            ballCount++;
//
//            if (ballCount > 2 && intakeDist > 0 && intakeDist < 15) {
//                intakePower(0);
//            }
//
//        }
//    }
//    public class CheckTransfer implements Action {
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            double intakeDist = intakeDistance.getDistance(DistanceUnit.MM);
//
//            boolean shouldTurn = Robot.runCheckLoop;
//
//            if (Robot.ballDist > 0 && Robot.ballDist < 40 && timer.seconds() > 0.75) {
//                if (shouldTurn) turnTransfer();
//
//                timer.reset();
//
//                ballCount++;
//                if (ballCount >= 1) {
//                    shouldTurn = false;
//                }
//
//                if (ballCount > 2 && intakeDist > 0 && intakeDist < 15) {
//                    intakePower(0);
//                }
//
//                Robot.ShouldTurn = shouldTurn;
//            }
//            return Robot.runCheckLoop;
//        }
//    }

    public Action driveAction(Gamepad gamepad) {
        return new LoopAction(() -> arcadeDrive(gamepad));
    }
    public void setTurretPos(double pos) {
        turretTarget = pos;
    }
    public Action sleepWithPID(double dt) {
        return new RaceAction(new SleepAction(dt), scoringLoop());
    }
    public Action sleepWithPIDTeleop(double dt, Gamepad gamepad, Telemetry telemetry) {
        return new RaceAction(new SleepAction(dt), scoringLoop(), driveAction(gamepad), new LoopAction(() -> {
            telemetry.addData("Vel", getRpm());
            telemetry.addData("Target", Robot.targetVel);
            telemetry.update();
        }));
    }
    public Action waitUntilReady(Gamepad gamepad, Telemetry telemetry) {
        return new RaceAction(new WaitUntilAction(() -> Math.abs(getRpm() - targetVel) <30), scoringLoop(), driveAction(gamepad), new LoopAction(() -> {
            telemetry.addData("Vel", getRpm());
            telemetry.addData("Target", Robot.targetVel);
            telemetry.addData("inRange", Math.abs(getRpm() - targetVel) <30);
            telemetry.update();
        }));
    }
    public Action waitUntilReady() {
        return new RaceAction(new WaitUntilAction(() -> Math.abs(getRpm() - targetVel) < 30), scoringLoop());
    }
    public Action waitForIntake(Gamepad gamepad, Telemetry telemetry) {
        return new RaceAction(scoringLoop(), driveAction(gamepad), telemetryPacket -> {
            telemetry.addData("Vel", getRpm());
            telemetry.addData("Target", Robot.targetVel);
            telemetry.addData("inRange", Math.abs(getRpm() - targetVel) <100);
            telemetry.update();
            return Math.abs(getRpm() - targetVel) > 100;
        });
    }
    public Action waitForIntake(Telemetry telemetry) {
        return new RaceAction(scoringLoop(),telemetryPacket -> {
            telemetry.addData("Vel", getRpm());
            telemetry.addData("Target", Robot.targetVel);
            telemetry.addData("inRange", Math.abs(getRpm() - targetVel) <30);
            telemetry.update();
            return Math.abs(getRpm() - targetVel) > 15;});
    }
    /// New
    public void shootFull(Telemetry telemetry) {
        if (closeMode) {
            shooting = true;
            hardstop.setPosition(HardstopOpen);
            Actions.runBlocking(sleepWithPIDTeleop(0.5, gamepad1, telemetry));
            transferPower(1);
            intakePower(1);
            Actions.runBlocking(sleepWithPIDTeleop(1.5, gamepad1, telemetry));
            hardstop.setPosition(HardstopClose);
            transferPower(0);
            intakePower(0);
            shooting = false;
        }
        else {
            shooting = true;
            hardstop.setPosition(HardstopOpen);
            Actions.runBlocking(sleepWithPIDTeleop(0.5, gamepad1, telemetry));
            transferPower(0.8);
            intakePower(1);
            Actions.runBlocking(sleepWithPIDTeleop(1.5, gamepad1, telemetry));
            hardstop.setPosition(HardstopClose);
            transferPower(0);
            intakePower(0);
            shooting = false;
        }
    }


    /// New

    public void intakePower(double power) {
        intakePower = power;
//        intake.setPower(power);
        intake.setPower(power);
    }
    public Action waitForDriver(Gamepad gamepad1, Telemetry telemetry) {
        return new RaceAction(new WaitUntilAction(() -> gamepad1.bWasPressed() && Math.abs(getRpm() - Robot.targetVel) < 40 || gamepad2.yWasPressed()), scoringLoop(), driveAction(gamepad1), new LoopAction(() -> {
            telemetry.addData("Vel", getRpm());
            telemetry.addData("Target", Robot.targetVel);
            telemetry.addData("inRange", Math.abs(getRpm() - targetVel) <30);
            telemetry.update();
        }));
    }

    public void arcadeDrive(Gamepad gamepad1) {
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        heading = 0;
        if (!atagAlign) {
            double rx = -0.75 * gamepad1.right_stick_x;

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-y, x), rx));
        }
        if (atagAlign) {
            atagAlign(-y, x);
        }
//        if (gamepad1.dpad_up) {
//            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(1, 0),0));
//        }
//        if (gamepad1.dpad_down) {
//            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-1, 0),0));
//        }
//        if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed()) {
//            heading = drive.localizer.getPose().heading.toDouble();
//        }
//        if (gamepad1.dpad_left) {
//            double hPower = hPID.calculate(drive.localizer.getPose().heading.toDouble(), heading);
//            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 1),hPower));
//        }
//        if (gamepad1.dpad_right) {
//            double hPower = hPID.calculate(drive.localizer.getPose().heading.toDouble(), heading);
//            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, -1), hPower));
//        }
    }
    public double getRpm() {
        return rpm;
    }
    public void atagAlign(double x, double y) {
        hPID.setPID(hP, hI, hD);
        double hPower = 0;
        if (!tagProcessor.getDetections().isEmpty() && (tagProcessor.getDetections().get(0).id == 20)) {
            hPower = hPID.calculate(tagProcessor.getDetections().get(0).ftcPose.bearing, 0);
        }
        else if (!tagProcessor.getDetections().isEmpty() && tagProcessor.getDetections().get(0).id
                == 24) {
            hPower = hPID.calculate(tagProcessor.getDetections().get(0).ftcPose.bearing, 0);
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(x,y), -hPower));

        drive.updatePoseEstimate();
    }

    public void arcadeDriveWithSlowMode(Gamepad gamepad) {
        double y,x,rx;
        if (gamepad.right_trigger > 0) {
            y = 0.5*gamepad.left_stick_y;
            x = -0.5*gamepad.left_stick_x;
            rx = -0.5*gamepad.right_stick_x;
        }
        else {
            y = gamepad.left_stick_y;
            x = -gamepad.left_stick_x;
            rx = -0.75*gamepad.right_stick_x;
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(x,y), rx));
    }

    //TODO: AprilTag Code

    public Pose2d getBotPose(MecanumDrivePurePursuit drive) {
        detections = tagProcessor.getDetections();
        if (detections.isEmpty()) return drive.localizer.getPose();

        for (AprilTagDetection tag : detections) {
            if (tag.id == 20 || tag.id == 24) {
                double r = get2dRange(tag);

                double theta = Math.toRadians(tag.ftcPose.bearing); //TODO: check if radians
                double imuHeading = drive.localizer.getPose().heading.toDouble();

                Vector2d tagPos = (tag.id == 20) ? BLUE_GOAL_TAG : RED_GOAL_TAG;

                double dx = r * Math.cos(theta + imuHeading);
                double dy = r * Math.sin(theta + imuHeading);

                double camX = tagPos.x - dx;
                double camY = tagPos.y - dy;

                double cx = 0; // forward offset from robot center
                double cy = 0; // left offset from robot center

                double offsetX = cx * Math.cos(imuHeading) - cy * Math.sin(imuHeading);
                double offsetY = cx * Math.sin(imuHeading) + cy * Math.cos(imuHeading);

                double rx = camX - offsetX;
                double ry = camY - offsetY;

                return new Pose2d(rx, ry, imuHeading);
            }
        }
        return drive.localizer.getPose();
    }


    public double get2dRange(AprilTagDetection detection) {
        if (detection.metadata != null) {
            return Math.sqrt(Math.pow(detection.ftcPose.range, 2) - Math.pow(TAG_HEIGHT, 2));
        }
        return 0;
    }

}
