package org.firstinspires.ftc.teamcode.modules;

import static org.firstinspires.ftc.teamcode.decode.DecodeRobot.drivetrainTelemetry;
import static org.firstinspires.ftc.teamcode.decode.DecodeRobot.robot;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.AnalogInput;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.control.PidController;
import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import static org.firstinspires.ftc.teamcode.architecture.OptimizationToggles.*;

@Config
public class Drivetrain extends Module {
    private final EnhancedMotor fl, bl, br, fr;
    private final AnalogInput floodgate;

    public final ElapsedTime bonkTimer = new ElapsedTime();


    private final double ENCODER_TO_RPM = 60.0 / 537.7;

    public static class EnableMotors {
        public boolean enableFl = true;
        public boolean enableBl = true;
        public boolean enableBr = true;
        public boolean enableFr = true;
    }

    public static class CurrentLimiterConfig {
        public boolean enabled = true;
        public double currentThresholdMin = 25.0;
        public double currentThresholdMax = 30.0;
        public double currentOverTime = 0;
        public double minCurrentOverTime = 2;
        public double integratedCurrentLimit = 400000;
        public double decayRate = 0.9;
        public double decayLoopMs = 5;
    }

    public static EnableMotors enableMotors = new EnableMotors();
    public static CurrentLimiterConfig currentLimiterConfig = new CurrentLimiterConfig();
    private ElapsedTime currentLoopTimer;
    private int currentLimiterLoopCounter = 0;
    private double cachedCurrentLimiterMultiplier = 1.0;

    private boolean headingLocked = false;
    private double lockedHeading = 0;
    private final PidController headingLockController = new PidController()
            .withGains(1, 0, 0.2, 0.0)
            .withFeedforward(0, 0)
            .withContinuousInput(-Math.PI, Math.PI);

    public enum DriveState implements State {
        MANUAL(0),
        EXTERNAL(0);

        DriveState(double value) {
            setValue(value);
        }
    }

    private double flPower, blPower, brPower, frPower;
    private double lastCurrentLimiterMultiplier = 1.0;
    private final double[] cachedMotorPowers = new double[4];
    public Drivetrain(HardwareMap hardwareMap) {
        super();
        setTelemetryEnabled(drivetrainTelemetry.TOGGLE);

        fl = new EnhancedMotor(hardwareMap, "fl")
                .withCachingTolerance(0.05);
        bl = new EnhancedMotor(hardwareMap, "bl")
                .withCachingTolerance(0.05);
        fr = new EnhancedMotor(hardwareMap, "fr")
                .withCachingTolerance(0.05);
        br = new EnhancedMotor(hardwareMap, "br")
                .withCachingTolerance(0.05);

        floodgate = hardwareMap.get(AnalogInput.class, "floodgate");
    }

    @Override
    public void init(){
        super.init();
        currentLoopTimer = new ElapsedTime();
        currentLimiterLoopCounter = 0;
        cachedCurrentLimiterMultiplier = 1.0;
    }
    @Override
    protected void initStates() {
        setStates(DriveState.MANUAL);
    }

    @Override
    protected void read() {

    }

    @Override
    protected void write() {
        if (!getState(DriveState.class).equals(DriveState.EXTERNAL)) {
            if (enableMotors.enableFl) {
                fl.setPower(flPower);
            } else {
                fl.setPower(0);
            }

            if (enableMotors.enableBl) {
                bl.setPower(blPower);
            } else {
                bl.setPower(0);
            }

            if (enableMotors.enableBr) {
                br.setPower(brPower);
            } else {
                br.setPower(0);
            }

            if (enableMotors.enableFr) {
                fr.setPower(frPower);
            } else {
                fr.setPower(0);
            }
        }
    }

    public void setTargets(double fl, double bl, double br, double fr) {
        double maxPower = Math.max(
                Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br))));

        if (maxPower > 1.0) {
            fl /= maxPower;
            bl /= maxPower;
            fr /= maxPower;
            br /= maxPower;
        }

        if (!optimizeCurrentLimiterComputation) {
            lastCurrentLimiterMultiplier = getCurrentLimiterMultiplier();
            currentLoopTimer.reset();
        } else {
            int every = Math.max(1, optimizeCurrentLimiterEveryNLoops);
            if ((currentLimiterLoopCounter++ % every) == 0) {
                cachedCurrentLimiterMultiplier = computeCurrentLimiterMultiplier(optimizeCurrentLimiterTelemetry);
                currentLoopTimer.reset();
            }
            lastCurrentLimiterMultiplier = cachedCurrentLimiterMultiplier;
        }
        flPower = fl * lastCurrentLimiterMultiplier;
        blPower = bl * lastCurrentLimiterMultiplier;
        brPower = br * lastCurrentLimiterMultiplier;
        frPower = fr * lastCurrentLimiterMultiplier;
    }

    public void setRawTargets(double fl, double bl, double br, double fr) {
        flPower = fl;
        blPower = bl;
        brPower = br;
        frPower = fr;
    }

    public void stop() {
        setTargets(0, 0, 0, 0);
    }

    public boolean isBonk() {
        if (robot.follower.isBusy() && !robot.follower.getCurrentPath().isAtParametricEnd() && robot.follower.getVelocity().getMagnitude() < 2) {
            if (bonkTimer.milliseconds() > 2000) {
                return true;
            }
        } else {
            bonkTimer.reset();
        }
        return false;
    }

    public double getFloodgateCurrent() {
        double voltage = floodgate.getVoltage();
        return (voltage / 3.3) * 80.0;
    }

    public double getCurrentLimiterMultiplier() {
        return computeCurrentLimiterMultiplier(true);
    }

    private double computeCurrentLimiterMultiplier(boolean emitTelemetry) {
//        if (!currentLimiterConfig.enabled) {
//            return 1.0;
//        }

        double current = getFloodgateCurrent();
        currentLimiterConfig.currentOverTime += Math.pow(current, 2) * currentLoopTimer.milliseconds();
        currentLimiterConfig.currentOverTime *= Math.pow(currentLimiterConfig.decayRate, (currentLoopTimer.milliseconds() / currentLimiterConfig.decayLoopMs));
        if (emitTelemetry) {
            robot.telemetry.addDashboardData("currentOverTime", "%.2f", currentLimiterConfig.currentOverTime);
            robot.telemetry.addDashboardData("final current scale", "%.3f", 1.0 - currentLimiterConfig.currentOverTime / currentLimiterConfig.integratedCurrentLimit);
        }


        if (current <= currentLimiterConfig.currentThresholdMin) { //25
            return 1.0;
        }
//        if (current >= currentLimiterConfig.currentThresholdMax) { //30
//            return 0.0;
//        }
        double range = currentLimiterConfig.currentThresholdMax - currentLimiterConfig.currentThresholdMin;
        double excess = current - currentLimiterConfig.currentThresholdMin;
//        return 1.0 - (excess / range);

        //theoretical integral current scaling. used 400000 because theoretically (I^2 * t) shouldn't exceed that
        return 1.0 - Range.clip(currentLimiterConfig.currentOverTime / currentLimiterConfig.integratedCurrentLimit, 0, 1);


    }

    public void lockHeading() {
        lockedHeading = robot.follower.getPose().getHeading();
        headingLockController.reset();
        headingLocked = true;
    }

    public void lockHeading(double headingRadians) {
        lockedHeading = headingRadians;
        headingLockController.reset();
        headingLocked = true;
    }

    public void unlockHeading() {
        headingLocked = false;
    }

    public boolean isHeadingLocked() {
        return headingLocked;
    }

    public void setMecanumTargets(double y, double x, double rx, boolean fieldCentric) {
        if (fieldCentric) {
            double heading = -robot.follower.getPose().getHeading();
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);

            if (Context.allianceColor == AllianceColor.BLUE) {
                y = -y;
            }

            double rotatedX = x * cos - y * sin;
            double rotatedY = x * sin + y * cos;

            y = rotatedY;
            x = rotatedX;
        }

        if (headingLocked) {
            double currentHeading = robot.follower.getPose().getHeading();
            double headingError = MathFunctions.getSmallestAngleDifference(currentHeading, lockedHeading)
                    * MathFunctions.getTurnDirection(currentHeading, lockedHeading);
            headingLockController.update(headingError, 0);
            rx = -Range.clip(headingLockController.calculate(), -1, 1);
        }

        double frontLeft = y + x + rx;
        double backLeft = y - x + rx;
        double frontRight = y - x - rx;
        double backRight = y + x - rx;

        setTargets(frontLeft, backLeft, backRight, frontRight);
    }

    public EnhancedMotor getFl() {
        return fl;
    }

    public EnhancedMotor getBl() {
        return bl;
    }

    public EnhancedMotor getBr() {
        return br;
    }

    public EnhancedMotor getFr() {
        return fr;
    }

    public double[] getMotorPowers() {
        if (optimizeMotorPowersCaching) {
            cachedMotorPowers[0] = flPower;
            cachedMotorPowers[1] = blPower;
            cachedMotorPowers[2] = brPower;
            cachedMotorPowers[3] = frPower;
            return cachedMotorPowers;
        }
        return new double[]{flPower, blPower, brPower, frPower};
    }

    @Override
    protected void onTelemetry() {
        if (drivetrainTelemetry.TOGGLE) {
            logDashboard("Drive Mode", getState(DriveState.class));
            logDashboard("Motor Powers", "FL:%.2f BL:%.2f FR:%.2f BR:%.2f", flPower, blPower, frPower, brPower);

            logDashboard("FL Power", "%.3f", fl.getPower());
            logDashboard("BL Power", "%.3f", bl.getPower());
            logDashboard("BR Power", "%.3f", br.getPower());
            logDashboard("FR Power", "%.3f", fr.getPower());
            logDashboard("FL Position (ticks)", fl.getCurrentPosition());
            logDashboard("FR Position (ticks)", fr.getCurrentPosition());
            logDashboard("FL Velocity (RPM)", "%.1f", fl.getVelocity() * ENCODER_TO_RPM);
            logDashboard("BL Velocity (RPM)", "%.1f", bl.getVelocity() * ENCODER_TO_RPM);
            logDashboard("BR Velocity (RPM)", "%.1f", br.getVelocity() * ENCODER_TO_RPM);
            logDashboard("FR Velocity (RPM)", "%.1f", fr.getVelocity() * ENCODER_TO_RPM);

            logDashboard("Floodgate Current (A)", "%.2f", getFloodgateCurrent());
            logDashboard("Current Limiter Multiplier", "%.2f", lastCurrentLimiterMultiplier);

            if (drivetrainTelemetry.current) {
                logDashboard("FL Current (A)", "%.2f", fl.getCurrent(CurrentUnit.AMPS));
                logDashboard("BL Current (A)", "%.2f", bl.getCurrent(CurrentUnit.AMPS));
                logDashboard("BR Current (A)", "%.2f", br.getCurrent(CurrentUnit.AMPS));
                logDashboard("FR Current (A)", "%.2f", fr.getCurrent(CurrentUnit.AMPS));
            }
        }
    }
}
