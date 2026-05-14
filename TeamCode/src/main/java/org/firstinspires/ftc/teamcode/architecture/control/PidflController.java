package org.firstinspires.ftc.teamcode.architecture.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PidflController {
    public double p = 0, i = 0, d = 0, f = 0, l = 0;
    public double integralMax = Double.POSITIVE_INFINITY;
    public boolean resetIntegralOnTargetChange = true;
    public boolean derivativeOnMeasurement = false;
    /** Errors with |error| <= this skip the static-friction kick to avoid chatter at setpoint. */
    public double lowerLimitDeadband = 0.0;
    private double previousError;
    private double previousPosition;
    private double error;
    private double integralSum;
    private double position;
    private double target;
    private double errorDerivative;
    private final ElapsedTime timer;
    private boolean firstUpdate = true;

    public PidflController() {
        timer = new ElapsedTime();
    }

    public double getPidfl() {
        double proportional = error * p;
        double integral = integralSum * i;
        double derivative = errorDerivative * d;
        double feedforward = target * f;
        // Static-friction kick: zero inside the deadband to avoid chattering across setpoint.
        double lowerLimit = Math.abs(error) <= lowerLimitDeadband ? 0 : Math.signum(error) * l;

        return proportional + integral + derivative + feedforward + lowerLimit;
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

    public void setController(double p, double i, double d, double f, double l) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.l = l;
    }

    public void setIntegralMax(double integralMax) {
        this.integralMax = integralMax;
    }

    public boolean isAtTarget(double tolerance) {
        return Math.abs(error) <= tolerance;
    }

    public PidflController withGains(double p, double i, double d, double f, double l) {
        setController(p, i, d, f, l);
        return this;
    }

    public PidflController withIntegralMax(double max) {
        this.integralMax = max;
        return this;
    }

    public PidflController withLowerLimitDeadband(double deadband) {
        this.lowerLimitDeadband = deadband;
        return this;
    }
}
