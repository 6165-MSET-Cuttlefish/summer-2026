package org.firstinspires.ftc.teamcode.Spline.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Transfer {

    public static double transferPower = 0.5;

    public static double detectionDistance = 5;

    public static double open = 0.4;
    public static double closed = 0.2;

    public static int loadDistance = 100;

    public static int autoLoadDistance = 2500;

    public DcMotor transferMotor;
    public DistanceSensor transferSensor;

    public Servo hardstop;

    public Transfer(HardwareMap hwMap) {
        transferMotor = hwMap.get(DcMotor.class, "transferMotor");

        transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        hardstop = hwMap.get(Servo.class, "hardstop");
    }

    public double getDistance() {
        return transferSensor.getDistance(DistanceUnit.CM);
    }

    public void openHardstop() {
        hardstop.setPosition(open);
    }

    public void closeHardstop() {
        hardstop.setPosition(closed);
    }

}
