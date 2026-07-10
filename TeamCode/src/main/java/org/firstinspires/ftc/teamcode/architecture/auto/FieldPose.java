package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;
import org.firstinspires.ftc.teamcode.architecture.core.Context;

/**
 * Author path geometry for RED; these mirror to BLUE when {@link Context#allianceColor} flips.
 */
public final class FieldPose {
    private FieldPose() {}

    public static Pose forAlliance(double x, double y, double heading) {
        return forAlliance(x, y, heading, FieldConfig.fieldWidthInches);
    }

    public static Pose forAlliance(double x, double y, double heading, double fieldWidth) {
        Pose red = new Pose(x, y, heading);
        return Context.allianceColor == AllianceColor.BLUE ? red.mirror(fieldWidth) : red;
    }
}
