package org.firstinspires.ftc.teamcode.Spline.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config

public class Intake {

    public static double intakePower = 1;

    public static double detectionDistance = 6.5;

    public DcMotor intakeMotor;

    public DistanceSensor leftIntakeSensor;
    public DistanceSensor rightIntakeSensor;

    public Intake(HardwareMap HWMap) {
        intakeMotor = HWMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double getDistance() {
        return Math.min(leftIntakeSensor.getDistance(DistanceUnit.CM), rightIntakeSensor.getDistance(DistanceUnit.CM));
    }

    public boolean ballSensed() {
        return getDistance() <= detectionDistance;
    }
}