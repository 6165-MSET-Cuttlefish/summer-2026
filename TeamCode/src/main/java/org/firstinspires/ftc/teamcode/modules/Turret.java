package org.firstinspires.ftc.teamcode.modules;

import static org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization.toField;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.targetX;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.targetY;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.turretFieldX;
import static org.firstinspires.ftc.teamcode.decode.DecodeContext.turretFieldY;
import static org.firstinspires.ftc.teamcode.decode.DecodeRobot.robot;
import static org.firstinspires.ftc.teamcode.decode.DecodeRobot.turretTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;

/**
 * Two-servo aiming turret with odometry-based auto-aim. The Limelight vision methods
 * ({@link #snapshotAprilTagOffset()}, {@link #getRelocalizedRobotPoseFromLimelight()},
 * {@link #detectObelisk()}) are stubs that read no camera.
 */
@Config
public class Turret extends Module {

    public double turretManualOffset = 0;
    public double leftSideOffset = 0;
    public static double autoTurretOffset = 0;

    public static boolean isCloseTele = true;
    public static double teleTurretOffsetClose = 2;
    public static double teleTurretOffsetFar = -4;

    public double turretAprilTagOffset = 0;
    public static boolean isAuto = false;

    public static double turretX = -3.875;
    public static double turretY = -1.6;

    public static double TENSION_OFFSET = 0.0;

    private String relocalizationStatus = "NO_ATTEMPT";

    private final EnhancedServo turretServoFront;
    private final EnhancedServo turretServoBack;

    public double targetAngle = 0;
    public double rawTargetAngle = 0;
    private double previousTargetAngle = 0.0;

    public static double maxTurretAngle = 90.0;

    public double aheadTargetAngle = 0.0;
    public static double AHEAD_GAIN = 0;
    public static double MIN_DELTA_FOR_AHEAD = 0;

    public double targetServoPosition = 0.5;
    public double lastTargetServoPosition = 0.5;

    public double deltaRawTarget = 0;
    private double previousRawTargetAngle = 0.0;

    private boolean withinRange = true;

    public static double frontServoOffset = 0.008;
    public static double backServoOffset = 0.00;

    public double flightTime = 1;

    public boolean detectingObelisk = false;

    public enum TurretState implements State {
        CENTER(.5),
        RIGHT(0.995),
        LEFT(0.005),
        AUTOAIM(-1),
        HOLD(-1),
        OFF(-1),
        MANUAL(-1);

        TurretState(double value) {
            setValue(value);
        }
    }

    public Turret(HardwareMap hardwareMap) {
        super();
        setTelemetryEnabled(turretTelemetry.TOGGLE);

        double turretServoTol = 0.001;
        turretServoFront = new EnhancedServo(hardwareMap, "turretFront").withCachingTolerance(turretServoTol);
        turretServoBack  = new EnhancedServo(hardwareMap, "turretBack").withCachingTolerance(turretServoTol);

        turretServoFront.setPwmRange(new PwmControl.PwmRange(525, 2475));
        turretServoBack.setPwmRange(new PwmControl.PwmRange(525, 2475));

        turretServoFront.setDirection(Servo.Direction.FORWARD);
        turretServoBack.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    protected void initStates() {
        setStates(TurretState.AUTOAIM);
    }

    @Override
    protected void read() {
        turretManualOffset = isAuto ? autoTurretOffset : (isCloseTele ? teleTurretOffsetClose : teleTurretOffsetFar);

        updateTargetPosition();
    }

    /** No-op: Limelight is stubbed, so the AprilTag offset is left unchanged. */
    public void snapshotAprilTagOffset() {
    }

    @Override
    protected void write() {
        if (getState(TurretState.class) == TurretState.OFF) {
            return;
        }

        double frontPos = Math.max(0.0, Math.min(1.0, targetServoPosition - TENSION_OFFSET + frontServoOffset));
        double backPos  = Math.max(0.0, Math.min(1.0, targetServoPosition + TENSION_OFFSET + backServoOffset));

        if (!robot.endgame.disableServosForEndgame) {
            turretServoFront.setPosition(frontPos);
            turretServoBack.setPosition(backPos);
        } else {
            turretServoFront.setPwmDisable();
            turretServoBack.setPwmDisable();
        }
    }

    @Override
    protected void onTelemetry() {
        if (turretTelemetry.TOGGLE) {
            if (turretTelemetry.position) {
                logDashboard("Turret State", getState(TurretState.class));
                logDashboard("Raw Target Angle (deg)",           "%.1f", rawTargetAngle);
                logDashboard("Delta Raw Target (deg)",           "%.1f", deltaRawTarget);
                log("Target Angle (deg)",               "%.1f", targetAngle);
                logDashboard("Ahead Target Angle (deg)",         "%.1f", aheadTargetAngle);
                log("Turret Offset (deg)",              "%.1f", turretManualOffset);
                log("Turret AprilTag Offset (deg)",     "%.1f", turretAprilTagOffset);
                log("Relocalization status",            "%s",   relocalizationStatus);
            }
            logDashboard("Target Servo Position", "%.3f", targetServoPosition);
            if (turretTelemetry.servos) {
                logDashboard("Front Servo Position", "%.3f", turretServoFront.getPosition());
                logDashboard("Back Servo Position",  "%.3f", turretServoBack.getPosition());
            }
        }
    }

    private void updateTargetPosition() {
        if (getState(TurretState.class).equals(TurretState.AUTOAIM)) {
            double robotHeading = Math.toDegrees(robot.follower.getPose().getHeading());

            Pose turretDash = toField(new Pose(turretFieldX, turretFieldY));
            robot.packet.fieldOverlay().setStroke("#FFFFFF")
                    .fillCircle(turretDash.getX(), turretDash.getY(), 2);

            Pose targetDash = toField(new Pose(targetX, targetY));
            robot.packet.fieldOverlay()
                    .setStroke(Context.allianceColor == AllianceColor.BLUE ? "blue" : "red")
                    .fillCircle(targetDash.getX(), targetDash.getY(), 2);

            double absoluteAngle =
                    Math.toDegrees(Math.atan2(targetY - turretFieldY, targetX - turretFieldX));

            rawTargetAngle = normalizeAngle(absoluteAngle - robotHeading);

            deltaRawTarget = calculateShortestError(rawTargetAngle, previousRawTargetAngle);
            previousRawTargetAngle = rawTargetAngle;

            targetAngle = rawTargetAngle + turretManualOffset;

            if (targetAngle < 90) {
                targetAngle += leftSideOffset;
            }

            double signedAngle = targetAngle > 180 ? targetAngle - 360 : targetAngle;
            withinRange = Math.abs(signedAngle) <= maxTurretAngle;

            double angleDelta = calculateShortestError(targetAngle, previousTargetAngle);
            aheadTargetAngle = (Math.abs(angleDelta) > MIN_DELTA_FOR_AHEAD)
                    ? targetAngle + Math.signum(angleDelta) * AHEAD_GAIN
                    : targetAngle;

            previousTargetAngle = targetAngle;
            targetServoPosition = angleToServoPosition(aheadTargetAngle);

        } else if (getState(TurretState.class).equals(TurretState.HOLD)) {
            targetServoPosition = lastTargetServoPosition;
        } else {
            targetServoPosition = getState(TurretState.class).getValue();
        }

        double minServo = Math.min(TurretState.LEFT.getValue(), TurretState.RIGHT.getValue());
        double maxServo = Math.max(TurretState.LEFT.getValue(), TurretState.RIGHT.getValue());
        if (targetServoPosition > maxServo) {
            targetServoPosition = maxServo;
            turretAprilTagOffset = 0;
        } else if (targetServoPosition < minServo) {
            targetServoPosition = minServo;
            turretAprilTagOffset = 0;
        }

        lastTargetServoPosition = targetServoPosition;
    }

    /** Always null: Limelight relocalization is stubbed, so the odometry pose is left as-is. */
    public Pose getRelocalizedRobotPoseFromLimelight() {
        relocalizationStatus = "LIMELIGHT_STUBBED";
        return null;
    }

    public String getRelocalizationStatus() {
        return relocalizationStatus;
    }

    public boolean isWithinRange() {
        return withinRange;
    }

    public void lock(Pose target) {
        double heading = target.getHeading();

        double lockTurretFieldX = target.getX() + turretX * Math.cos(heading) - turretY * Math.sin(heading);
        double lockTurretFieldY = target.getY() + turretX * Math.sin(heading) + turretY * Math.cos(heading);

        double absoluteAngle = Math.toDegrees(
                Math.atan2(targetY - lockTurretFieldY, targetX - lockTurretFieldX));
        double angle = normalizeAngle(absoluteAngle - Math.toDegrees(heading)) + autoTurretOffset;
        if (angle < 90) {
            angle += leftSideOffset;
        }

        double servoPosition = clampServoPosition(angleToServoPosition(angle));
        TurretState.MANUAL.setValue(servoPosition);
        TurretState.MANUAL.activate();
    }


    public double robotAngleToPoseDeg(Pose target) {
        return Math.toDegrees(Math.atan2(
                target.getY() - robot.follower.getPose().getY(),
                target.getX() - robot.follower.getPose().getX()));
    }

    public void unlock() {
        TurretState.AUTOAIM.activate();
    }

    private double getCurrentTurretSignedAngleDeg() {
        double scale = (TurretState.LEFT.getValue() - TurretState.RIGHT.getValue()) / 180.0;
        double liveServoPos = turretServoFront.getPosition();
        return (liveServoPos - TurretState.CENTER.getValue()) / scale;
    }

    private double angleToServoPosition(double angleDeg) {
        double angle = normalizeAngle(angleDeg);
        double scale = (TurretState.LEFT.getValue() - TurretState.RIGHT.getValue()) / 180.0;
        return (angle > 180)
                ? TurretState.CENTER.getValue() + scale * (angle - 360)
                : TurretState.CENTER.getValue() + scale * angle;
    }

    private double clampServoPosition(double position) {
        double lo = Math.min(TurretState.LEFT.getValue(), TurretState.RIGHT.getValue());
        double hi = Math.max(TurretState.LEFT.getValue(), TurretState.RIGHT.getValue());
        return Math.max(lo, Math.min(hi, position));
    }

    /** Normalizes to [0, 360) — callers rely on the wrap-at-180 branch, not a signed range. */
    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    private double calculateShortestError(double target, double current) {
        double error = target - current;
        while (error >  180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    /** Always false: Limelight is stubbed, so the obelisk motif stays at the DecodeContext default. */
    public boolean detectObelisk() {
        return false;
    }
}
