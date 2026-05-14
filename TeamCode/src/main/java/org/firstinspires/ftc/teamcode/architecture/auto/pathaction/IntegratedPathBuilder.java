package org.firstinspires.ftc.teamcode.architecture.auto.pathaction;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.paths.callbacks.PathCallback;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import org.firstinspires.ftc.teamcode.core.action.Action;
import org.firstinspires.ftc.teamcode.core.action.ActionBuilder;
import org.firstinspires.ftc.teamcode.core.action.Actions;
import org.firstinspires.ftc.teamcode.core.Robot;

/**
 * Fluent wrapper around Pedro's {@link PathBuilder} that also collects "during" actions for the
 * scheduler to launch alongside the path.
 */
public class IntegratedPathBuilder {
    private final Pose startPose;
    private final PathBuilder pedroBuilder;
    private Pose lastPose;
    private final List<Action> duringActions = new ArrayList<>();
    private boolean holdEnd = true;
    private Double holdAtDistance = null;

    IntegratedPathBuilder(Pose startPose) {
        this(startPose, Robot.robot.follower.pathBuilder());
    }

    IntegratedPathBuilder(Pose startPose, PathBuilder pedroBuilder) {
        this.startPose = startPose;
        this.pedroBuilder = pedroBuilder;
        this.lastPose = startPose;
    }

    public Pose getStartPose() { return startPose; }

    public IntegratedPathBuilder addLine(Pose endPose) {
        pedroBuilder.addPath(new BezierLine(lastPose, endPose));
        lastPose = endPose;
        return this;
    }

    public IntegratedPathBuilder addCurve(Pose controlPoint, Pose endPose) {
        pedroBuilder.addPath(new BezierCurve(lastPose, controlPoint, endPose));
        lastPose = endPose;
        return this;
    }

    public IntegratedPathBuilder addCurve(Pose c1, Pose c2, Pose endPose) {
        pedroBuilder.addPath(new BezierCurve(lastPose, c1, c2, endPose));
        lastPose = endPose;
        return this;
    }

    public IntegratedPathBuilder addCurve(Pose c1, Pose c2, Pose c3, Pose endPose) {
        pedroBuilder.addPath(new BezierCurve(lastPose, c1, c2, c3, endPose));
        lastPose = endPose;
        return this;
    }

    public IntegratedPathBuilder addPath(Path path) {
        pedroBuilder.addPath(path);
        lastPose = path.endPose();
        return this;
    }

    public IntegratedPathBuilder addPath(Curve curve) {
        pedroBuilder.addPath(curve);
        lastPose = curve.getLastControlPoint();
        return this;
    }

    public IntegratedPathBuilder addPaths(Path... paths) {
        pedroBuilder.addPaths(paths);
        if (paths.length > 0) {
            lastPose = paths[paths.length - 1].endPose();
        }
        return this;
    }

    public IntegratedPathBuilder addPaths(Curve... curves) {
        pedroBuilder.addPaths(curves);
        if (curves.length > 0) lastPose = curves[curves.length - 1].getLastControlPoint();
        return this;
    }

    public IntegratedPathBuilder curveThrough(Pose prevPoint, Pose startPoint, double tension, Pose... points) {
        pedroBuilder.curveThrough(prevPoint, startPoint, tension, points);
        lastPose = points.length > 0 ? points[points.length - 1] : startPoint;
        return this;
    }

    public IntegratedPathBuilder curveThrough(double tension, Pose... points) {
        pedroBuilder.curveThrough(tension, points);
        if (points.length > 0) lastPose = points[points.length - 1];
        return this;
    }

    public IntegratedPathBuilder setConstantHeading(double heading) {
        pedroBuilder.setConstantHeadingInterpolation(heading);
        return this;
    }

    public IntegratedPathBuilder setGlobalConstantHeading(double heading) {
        pedroBuilder.setGlobalConstantHeadingInterpolation(heading);
        return this;
    }

    public IntegratedPathBuilder setLinearHeading(double startHeading, double endHeading) {
        pedroBuilder.setLinearHeadingInterpolation(startHeading, endHeading);
        return this;
    }

    public IntegratedPathBuilder setLinearHeading(double startHeading, double endHeading, double endTime) {
        pedroBuilder.setLinearHeadingInterpolation(startHeading, endHeading, endTime);
        return this;
    }

    public IntegratedPathBuilder setLinearHeading(
            double startHeading, double endHeading, double startTime, double endTime) {
        pedroBuilder.setLinearHeadingInterpolation(startHeading, endHeading, endTime, startTime);
        return this;
    }

    public IntegratedPathBuilder setGlobalLinearHeading(double startHeading, double endHeading) {
        pedroBuilder.setGlobalLinearHeadingInterpolation(startHeading, endHeading);
        return this;
    }

    public IntegratedPathBuilder setGlobalLinearHeading(double startHeading, double endHeading, double endTime) {
        pedroBuilder.setGlobalLinearHeadingInterpolation(startHeading, endHeading, endTime);
        return this;
    }

    public IntegratedPathBuilder setGlobalLinearHeading(
            double startHeading, double endHeading, double startTime, double endTime) {
        pedroBuilder.setGlobalLinearHeadingInterpolation(startHeading, endHeading, endTime, startTime);
        return this;
    }

    public IntegratedPathBuilder setTangentHeading() {
        pedroBuilder.setTangentHeadingInterpolation();
        return this;
    }

    public IntegratedPathBuilder setGlobalTangentHeading() {
        pedroBuilder.setGlobalTangentHeadingInterpolation();
        return this;
    }

    public IntegratedPathBuilder setReversed() {
        pedroBuilder.setReversed();
        return this;
    }

    public IntegratedPathBuilder setGlobalReversed() {
        pedroBuilder.setGlobalReversed();
        return this;
    }

    public IntegratedPathBuilder setHeadingInterpolation(HeadingInterpolator function) {
        pedroBuilder.setHeadingInterpolation(function);
        return this;
    }

    public IntegratedPathBuilder setGlobalHeadingInterpolation(HeadingInterpolator function) {
        pedroBuilder.setGlobalHeadingInterpolation(function);
        return this;
    }

    public IntegratedPathBuilder setBrakingStrength(int strength) {
        pedroBuilder.setBrakingStrength(strength);
        return this;
    }

    public IntegratedPathBuilder setBrakingStrength(double strength) {
        pedroBuilder.setBrakingStrength(strength);
        return this;
    }

    public IntegratedPathBuilder setBrakingStart(double set) {
        pedroBuilder.setBrakingStart(set);
        return this;
    }

    public IntegratedPathBuilder setVelocityConstraint(double set) {
        pedroBuilder.setVelocityConstraint(set);
        return this;
    }

    public IntegratedPathBuilder setTranslationalConstraint(double set) {
        pedroBuilder.setTranslationalConstraint(set);
        return this;
    }

    public IntegratedPathBuilder setHeadingConstraint(double set) {
        pedroBuilder.setHeadingConstraint(set);
        return this;
    }

    public IntegratedPathBuilder setTValueConstraint(double set) {
        pedroBuilder.setTValueConstraint(set);
        return this;
    }

    public IntegratedPathBuilder setTimeoutConstraint(double set) {
        pedroBuilder.setTimeoutConstraint(set);
        return this;
    }

    public IntegratedPathBuilder setGlobalDeceleration() {
        pedroBuilder.setGlobalDeceleration();
        return this;
    }

    public IntegratedPathBuilder setGlobalDeceleration(double brakingStart) {
        pedroBuilder.setGlobalDeceleration(brakingStart);
        return this;
    }

    public IntegratedPathBuilder setNoDeceleration() {
        pedroBuilder.setNoDeceleration();
        return this;
    }

    public IntegratedPathBuilder setConstraints(PathConstraints constraints) {
        pedroBuilder.setConstraints(constraints);
        return this;
    }

    public IntegratedPathBuilder setConstraintsForAll(PathConstraints constraints) {
        pedroBuilder.setConstraintsForAll(constraints);
        return this;
    }

    public IntegratedPathBuilder setConstraintsForLast(PathConstraints constraints) {
        pedroBuilder.setConstraintsForLast(constraints);
        return this;
    }

    public IntegratedPathBuilder addTemporalCallback(double time, Runnable runnable) {
        pedroBuilder.addTemporalCallback(time, runnable);
        return this;
    }

    public IntegratedPathBuilder addParametricCallback(double t, Runnable runnable) {
        pedroBuilder.addParametricCallback(t, runnable);
        return this;
    }

    public IntegratedPathBuilder addPoseCallback(Pose targetPoint, Runnable runnable, double initialTValueGuess) {
        pedroBuilder.addPoseCallback(targetPoint, runnable, initialTValueGuess);
        return this;
    }

    public IntegratedPathBuilder addCallback(PathCallback callback) {
        pedroBuilder.addCallback(callback);
        return this;
    }

    public IntegratedPathBuilder addCallback(PathCallback callback, int i) {
        pedroBuilder.addCallback(callback, i);
        return this;
    }

    public IntegratedPathBuilder addCallback(PathBuilder.CallbackCondition condition, Runnable action) {
        pedroBuilder.addCallback(condition, action);
        return this;
    }

    public IntegratedPathBuilder addCallback(PathBuilder.CallbackCondition condition, Runnable action, int i) {
        pedroBuilder.addCallback(condition, action, i);
        return this;
    }

    public IntegratedPathBuilder addLoopedCallback(PathCallback callback) {
        pedroBuilder.addLoopedCallback(callback);
        return this;
    }

    public IntegratedPathBuilder during(Action action) {
        duringActions.add(action);
        return this;
    }

    public IntegratedPathBuilder whenDuring(Callable<Boolean> condition, Runnable code) {
        ActionBuilder builder = Actions.builder();
        builder.waitUntil(() -> {
            try {
                return condition.call();
            } catch (Exception e) {
                return false;
            }
        });
        builder.run(code);
        duringActions.add(builder.build());
        return this;
    }

    public IntegratedPathBuilder whenDuring(Callable<Boolean> condition, Action action) {
        ActionBuilder builder = Actions.builder();
        builder.waitUntil(() -> {
            try {
                return condition.call();
            } catch (Exception e) {
                return false;
            }
        });
        builder.action(action);
        duringActions.add(builder.build());
        return this;
    }

    public IntegratedPathBuilder setHoldEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

    /**
     * Advance the scheduler as soon as {@code follower.getDistanceRemaining() <= distance},
     * leaving Pedro to finish the last inch itself and auto-engage its end-hold. Lets
     * intermediate scheduler segments overlap with Pedro's final convergence.
     */
    public IntegratedPathBuilder holdAtDistance(double distance) {
        this.holdAtDistance = distance;
        return this;
    }

    PathChain buildPath() { return pedroBuilder.build(); }
    List<Action> getDuringActions() { return new ArrayList<>(duringActions); }
    boolean isHoldEnd() { return holdEnd; }
    Double getHoldAtDistance() { return holdAtDistance; }
}
