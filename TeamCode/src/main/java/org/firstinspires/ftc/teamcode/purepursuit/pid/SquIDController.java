/*
 * Copyright (c) 2026 Vikram Janakiraman. All rights reserved.
 */

package org.firstinspires.ftc.teamcode.purepursuit.pid;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.purepursuit.math.PurePursuit;

public class SquIDController {
    public double kSQ;
    public double kF;

    public SquIDController(double kSQ, double kF) {
        this.kSQ = kSQ;
        this.kF = kF;
    }
    public SquIDController(PIDCoefficients pid) {
        kSQ = pid.p;
    }
    public void setSquID(double kSQ, double kF) {
        this.kSQ = kSQ;
        this.kF = kF;
    }
    public void setSquID(PIDCoefficients pid) {
        kSQ = pid.p;
    }
    public static double polynomial = 1.8;

    public double compute(double error) {
        if (Math.abs(error) < PurePursuit.Defaults.INSTANCE.getPosTolerance()) return 0.0;
        double squid =  kSQ * Math.pow(Math.abs(error), polynomial) * Math.signum(error);
        return squid + kF*Math.signum(error);
    }
}
