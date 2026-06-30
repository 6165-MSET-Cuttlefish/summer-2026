package org.firstinspires.ftc.teamcode.purepursuit.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;

public class LoopAction implements Action {
    InstantFunction function;
    public LoopAction(InstantFunction function) {
        this.function = function;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        function.run();
        return true;
    }
}
