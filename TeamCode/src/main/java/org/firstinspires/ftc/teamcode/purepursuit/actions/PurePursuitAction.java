package org.firstinspires.ftc.teamcode.purepursuit.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.purepursuit.math.Bezier;
import org.firstinspires.ftc.teamcode.purepursuit.math.BezierPath;
import org.firstinspires.ftc.teamcode.purepursuit.math.Path;
import org.firstinspires.ftc.teamcode.purepursuit.math.PurePursuit;

import java.util.Objects;

public class PurePursuitAction implements Action {
    PurePursuit purePursuit;
    Path path;
    Double heading = null;
    Boolean reversed = null;
    public PurePursuitAction(PurePursuit purePursuit, Path path) {
        this.purePursuit = purePursuit;
        this.path = path;
        purePursuit.lastT = 0.0;
        purePursuit.hasReachedDestination = false;
        purePursuit.currentBezierPath = null;
        purePursuit.atEnd = false;
//        purePursuit.currentPath = (path instanceof Bezier) ? (Bezier) path : null;
    }

    public void setConstantHeading(double rad) {
        heading = rad;
    }
    public void setReversed(boolean reversed) {
        this.reversed = reversed;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (path instanceof Bezier) {
            if (Objects.nonNull(heading)) {
                purePursuit.followPathSingle((Bezier) path, telemetryPacket, heading, reversed != null && reversed);
            } else if (Objects.nonNull(reversed)) {
                purePursuit.followPathSingle((Bezier) path, telemetryPacket, null, reversed);
            } else {
                purePursuit.followPathSingle((Bezier) path, telemetryPacket);
            }
        } else if (path instanceof BezierPath) {
            if (Objects.nonNull(heading)) {
                purePursuit.followPathSingle((BezierPath) path, telemetryPacket, heading, reversed != null && reversed);
            } else if (Objects.nonNull(reversed)) {
                purePursuit.followPathSingle((BezierPath) path, telemetryPacket, null, reversed);
            } else {
                purePursuit.followPathSingle((BezierPath) path, telemetryPacket);
            }
        } else {
            throw new IllegalArgumentException("PurePursuitAction only accepts Bezier or BezierPath!");
        }

        return !purePursuit.hasReachedDestination && purePursuit.notWithinTolerance(
                purePursuit.pose, purePursuit.targetPose,
                PurePursuit.Defaults.INSTANCE.getPosTolerance(),
                PurePursuit.Defaults.INSTANCE.getHTolerance()
        );
    }
}
