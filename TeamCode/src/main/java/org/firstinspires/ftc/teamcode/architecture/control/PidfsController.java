package org.firstinspires.ftc.teamcode.architecture.control;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * PID + feed-forward + static-friction kick. Fields are public for live {@code @Config} tuning.
 *
 * <ul>
 *   <li>{@code p, i, d} — standard PID gains.</li>
 *   <li>{@code f} — feed-forward proportional to the target.</li>
 *   <li>{@code s} — static-friction kick: a constant push in the direction of error to overcome
 *       stiction. Zeroed when {@code |error| <= staticDeadband} so the controller doesn't
 *       chatter across setpoint.</li>
 * </ul>
 */
public class PidfsController {
    public double p = 0, i = 0, d = 0, f = 0, s = 0;
    public double integralMax = Double.POSITIVE_INFINITY;
    public boolean resetIntegralOnTargetChange = true;
    public boolean derivativeOnMeasurement = false;
    /** Errors with |error| <= this skip the static-friction kick to avoid chatter at setpoint. */
    public double staticDeadband = 0.0;
    private double previousError;
    private double previousPosition;
    private double error;
    private double integralSum;
    private double position;
    private double target;
    private double errorDerivative;
    private final ElapsedTime timer;
    private boolean firstUpdate = true;

    public PidfsController() {
        timer = new ElapsedTime();
    }

    public double getPidfs() {
        double proportional = error * p;
        double integral = integralSum * i;
        double derivative = errorDerivative * d;
        double feedforward = target * f;
        double staticKick = Math.abs(error) <= staticDeadband ? 0 : Math.signum(error) * s;

        return proportional + integral + derivative + feedforward + staticKick;
    }

    public void updatePosition(double update) {
        previousPosition = position;
        position = update;
        previousError = error;
        error = target - position;

        double deltaTimeSeconds = timer.seconds();
        timer.reset();

        if (firstUpdate) {
            firstUpdate = false;
            errorDerivative = 0;
            return;
        }

        if (deltaTimeSeconds > 0) {
            if (derivativeOnMeasurement) {
                // D on measurement avoids derivative kick when target steps
                errorDerivative = -(position - previousPosition) / deltaTimeSeconds;
            } else {
                errorDerivative = (error - previousError) / deltaTimeSeconds;
            }
        } else {
            errorDerivative = 0;
        }

        integralSum += error * deltaTimeSeconds;

        if (Math.abs(integralSum) > integralMax) {
            integralSum = Math.signum(integralSum) * integralMax;
        }
    }

    public void update(double target, double current) {
        setTarget(target);
        updatePosition(current);
    }

    public void reset() {
        previousError = 0;
        previousPosition = 0;
        error = 0;
        integralSum = 0;
        position = 0;
        target = 0;
        errorDerivative = 0;
        firstUpdate = true;
        timer.reset();
    }

    public void setTarget(double set) {
        if (resetIntegralOnTargetChange && Math.abs(set - target) > 1e-9) {
            integralSum = 0;
        }
        target = set;
    }

    public double getError() {
        return error;
    }

    public double getIntegralSum() {
        return integralSum;
    }

    public double getErrorDerivative() {
        return errorDerivative;
    }

    public void setController(double p, double i, double d, double f, double s) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.s = s;
    }

    public void setIntegralMax(double integralMax) {
        this.integralMax = integralMax;
    }

    public boolean isAtTarget(double tolerance) {
        return Math.abs(error) <= tolerance;
    }

    public PidfsController withGains(double p, double i, double d, double f, double s) {
        setController(p, i, d, f, s);
        return this;
    }

    public PidfsController withIntegralMax(double max) {
        this.integralMax = max;
        return this;
    }

    public PidfsController withStaticDeadband(double deadband) {
        this.staticDeadband = deadband;
        return this;
    }
}
