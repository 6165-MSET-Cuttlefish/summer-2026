package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;

@Autonomous(name = "Close BLUE", group = "A")
public class CloseBlue extends Close {
    @Override
    protected AllianceColor alliance() {
        return AllianceColor.BLUE;
    }
}
