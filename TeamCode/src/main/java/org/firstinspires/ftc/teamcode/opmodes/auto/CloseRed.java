package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;

@Autonomous(name = "Close RED", group = "A")
public class CloseRed extends Close {
    @Override
    protected AllianceColor alliance() {
        return AllianceColor.RED;
    }
}
