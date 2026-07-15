package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;

/**
 * BLUE entry point for the ported "Close" auto. Pins {@link AllianceColor#BLUE} via
 * {@code alliance()}, which {@link Close}'s instance initializer reads before building its poses;
 * every {@code FieldPose.forAlliance} pose then mirrors across the field (x = fieldWidth - x,
 * heading = 180deg - heading), exactly as DECODE's ContextBlue + FieldPose.ColorPose mirroring did.
 * The per-alliance branches the mirror doesn't cover (intake2 row pose, the 3-control-point
 * second-row curve, the gate intake pose, and the RPM/aim offsets) are handled inside {@link Close}
 * by reading {@link AllianceColor}.
 */
@Autonomous(name = "Close BLUE", group = "A")
public class CloseBlue extends Close {
    @Override
    protected AllianceColor alliance() {
        return AllianceColor.BLUE;
    }
}
