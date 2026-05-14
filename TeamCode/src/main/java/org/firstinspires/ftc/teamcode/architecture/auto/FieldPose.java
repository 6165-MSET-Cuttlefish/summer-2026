package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.core.AllianceColor;
import org.firstinspires.ftc.teamcode.core.Context;

public class FieldPose {
    private FieldPose() {}

    /**
     * Mirror a RED-side pose to BLUE if Context.allianceColor is BLUE. Field width comes from
     * {@link FieldConfig#fieldWidthInches} so this stays season-portable.
     */
    public static Pose ColorPose(double x, double y, double heading) {
        return ColorPose(x, y, heading, FieldConfig.fieldWidthInches);
    }

    /** Same as {@link #ColorPose(double, double, double)} with an explicit field width. */
    public static Pose ColorPose(double x, double y, double heading, double fieldWidth) {
        if (Context.allianceColor.equals(AllianceColor.BLUE)) {
            x = fieldWidth - x;
            heading = Math.toRadians(180) - heading;
        }

        return new Pose(x, y, heading);
    }
}
