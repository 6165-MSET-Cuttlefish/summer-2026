package org.firstinspires.ftc.teamcode.purepursuit.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.function.BooleanSupplier;

public class WaitUntilAction implements Action {
    public BooleanSupplier stop;

    public WaitUntilAction(BooleanSupplier stopCondition) {
        stop = stopCondition;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return !stop.getAsBoolean();
    }
}
