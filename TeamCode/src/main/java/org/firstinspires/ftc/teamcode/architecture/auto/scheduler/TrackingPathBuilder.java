package org.firstinspires.ftc.teamcode.architecture.auto.scheduler;

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
import org.firstinspires.ftc.teamcode.architecture.action.Action;
import org.firstinspires.ftc.teamcode.architecture.action.ActionBuilder;
import org.firstinspires.ftc.teamcode.architecture.action.Actions;

/** Wraps Pedro's {@link PathBuilder} and collects "during" actions for the scheduler. */
public class TrackingPathBuilder {
    private final Pose startPose;
    private final PathBuilder pedroBuilder;
    private Pose lastPose;
    private final List<Action> duringActions = new ArrayList<>();
    private boolean holdEnd = true;
    private Double holdAtDistance = null;

    TrackingPathBuilder(Pose startPose, PathBuilder pedroBuilder) {
        this.startPose = startPose;
        this.pedroBuilder = pedroBuilder;
        this.lastPose = startPose;
    }

    public Pose getStartPose() { return startPose; }

    public TrackingPathBuilder addLine(Pose endPose) {
        pedroBuilder.addPath(new BezierLine(lastPose, endPose));
        lastPose = endPose;
        return this;
    }

    public TrackingPathBuilder addCurve(Pose controlPoint, Pose endPose) {
        pedroBuilder.addPath(new BezierCurve(lastPose, controlPoint, endPose));
        lastPose = endPose;
        return this;
    }

    public TrackingPathBuilder addCurve(Pose c1, Pose c2, Pose endPose) {
        pedroBuilder.addPath(new BezierCurve(lastPose, c1, c2, endPose));
        lastPose = endPose;
        return this;
    }

    public TrackingPathBuilder addCurve(Pose c1, Pose c2, Pose c3, Pose endPose) {
        pedroBuilder.addPath(new BezierCurve(lastPose, c1, c2, c3, endPose));
        lastPose = endPose;
        return this;
    }

    public TrackingPathBuilder addPath(Path path) {
        pedroBuilder.addPath(path);
        lastPose = path.endPose();
        return this;
    }

    public TrackingPathBuilder addPath(Curve curve) {
        pedroBuilder.addPath(curve);
        lastPose = curve.getLastControlPoint();
        return this;
    }

    public TrackingPathBuilder addPaths(Path... paths) {
        pedroBuilder.addPaths(paths);
        if (paths.length > 0) {
            lastPose = paths[paths.length - 1].endPose();
        }
        return this;
    }

    public TrackingPathBuilder addPaths(Curve... curves) {
        pedroBuilder.addPaths(curves);
        if (curves.length > 0) lastPose = curves[curves.length - 1].getLastControlPoint();
        return this;
    }

    public TrackingPathBuilder curveThrough(Pose prevPoint, Pose startPoint, double tension, Pose... points) {
        pedroBuilder.curveThrough(prevPoint, startPoint, tension, points);
        lastPose = points.length > 0 ? points[points.length - 1] : startPoint;
        return this;
    }

    public TrackingPathBuilder curveThrough(double tension, Pose... points) {
        if (points.length == 0) {
            throw new IllegalArgumentException("curveThrough requires at least one point");
        }
        pedroBuilder.curveThrough(tension, points);
        lastPose = points[points.length - 1];
        return this;
    }

    public TrackingPathBuilder setConstantHeading(double heading) {
        pedroBuilder.setConstantHeadingInterpolation(heading);
        return this;
    }

    public TrackingPathBuilder setGlobalConstantHeading(double heading) {
        pedroBuilder.setGlobalConstantHeadingInterpolation(heading);
        return this;
    }

    public TrackingPathBuilder setLinearHeading(double startHeading, double endHeading) {
        pedroBuilder.setLinearHeadingInterpolation(startHeading, endHeading);
        return this;
    }

    public TrackingPathBuilder setLinearHeading(double startHeading, double endHeading, double endTime) {
        pedroBuilder.setLinearHeadingInterpolation(startHeading, endHeading, endTime);
        return this;
    }

    public TrackingPathBuilder setLinearHeading(
            double startHeading, double endHeading, double startTime, double endTime) {
        pedroBuilder.setLinearHeadingInterpolation(startHeading, endHeading, endTime, startTime);
        return this;
    }

    public TrackingPathBuilder setGlobalLinearHeading(double startHeading, double endHeading) {
        pedroBuilder.setGlobalLinearHeadingInterpolation(startHeading, endHeading);
        return this;
    }

    public TrackingPathBuilder setGlobalLinearHeading(double startHeading, double endHeading, double endTime) {
        pedroBuilder.setGlobalLinearHeadingInterpolation(startHeading, endHeading, endTime);
        return this;
    }

    public TrackingPathBuilder setGlobalLinearHeading(
            double startHeading, double endHeading, double startTime, double endTime) {
        pedroBuilder.setGlobalLinearHeadingInterpolation(startHeading, endHeading, endTime, startTime);
        return this;
    }

    public TrackingPathBuilder setTangentHeading() {
        pedroBuilder.setTangentHeadingInterpolation();
        return this;
    }

    public TrackingPathBuilder setGlobalTangentHeading() {
        pedroBuilder.setGlobalTangentHeadingInterpolation();
        return this;
    }

    public TrackingPathBuilder setReversed() {
        pedroBuilder.setReversed();
        return this;
    }

    public TrackingPathBuilder setGlobalReversed() {
        pedroBuilder.setGlobalReversed();
        return this;
    }

    public TrackingPathBuilder setHeadingInterpolation(HeadingInterpolator function) {
        pedroBuilder.setHeadingInterpolation(function);
        return this;
    }

    public TrackingPathBuilder setGlobalHeadingInterpolation(HeadingInterpolator function) {
        pedroBuilder.setGlobalHeadingInterpolation(function);
        return this;
    }

    public TrackingPathBuilder setBrakingStrength(int strength) {
        pedroBuilder.setBrakingStrength(strength);
        return this;
    }

    public TrackingPathBuilder setBrakingStrength(double strength) {
        pedroBuilder.setBrakingStrength(strength);
        return this;
    }

    public TrackingPathBuilder setBrakingStart(double set) {
        pedroBuilder.setBrakingStart(set);
        return this;
    }

    public TrackingPathBuilder setVelocityConstraint(double set) {
        pedroBuilder.setVelocityConstraint(set);
        return this;
    }

    public TrackingPathBuilder setTranslationalConstraint(double set) {
        pedroBuilder.setTranslationalConstraint(set);
        return this;
    }

    public TrackingPathBuilder setHeadingConstraint(double set) {
        pedroBuilder.setHeadingConstraint(set);
        return this;
    }

    public TrackingPathBuilder setTValueConstraint(double set) {
        pedroBuilder.setTValueConstraint(set);
        return this;
    }

    public TrackingPathBuilder setTimeoutConstraint(double set) {
        pedroBuilder.setTimeoutConstraint(set);
        return this;
    }

    public TrackingPathBuilder setGlobalDeceleration() {
        pedroBuilder.setGlobalDeceleration();
        return this;
    }

    public TrackingPathBuilder setGlobalDeceleration(double brakingStart) {
        pedroBuilder.setGlobalDeceleration(brakingStart);
        return this;
    }

    public TrackingPathBuilder setNoDeceleration() {
        pedroBuilder.setNoDeceleration();
        return this;
    }

    public TrackingPathBuilder setConstraints(PathConstraints constraints) {
        pedroBuilder.setConstraints(constraints);
        return this;
    }

    public TrackingPathBuilder setConstraintsForAll(PathConstraints constraints) {
        pedroBuilder.setConstraintsForAll(constraints);
        return this;
    }

    public TrackingPathBuilder setConstraintsForLast(PathConstraints constraints) {
        pedroBuilder.setConstraintsForLast(constraints);
        return this;
    }

    public TrackingPathBuilder addTemporalCallback(double time, Runnable runnable) {
        pedroBuilder.addTemporalCallback(time, runnable);
        return this;
    }

    public TrackingPathBuilder addParametricCallback(double t, Runnable runnable) {
        pedroBuilder.addParametricCallback(t, runnable);
        return this;
    }

    public TrackingPathBuilder addPoseCallback(Pose targetPoint, Runnable runnable, double initialTValueGuess) {
        pedroBuilder.addPoseCallback(targetPoint, runnable, initialTValueGuess);
        return this;
    }

    public TrackingPathBuilder addCallback(PathCallback callback) {
        pedroBuilder.addCallback(callback);
        return this;
    }

    public TrackingPathBuilder addCallback(PathCallback callback, int i) {
        pedroBuilder.addCallback(callback, i);
        return this;
    }

    public TrackingPathBuilder addCallback(PathBuilder.CallbackCondition condition, Runnable action) {
        pedroBuilder.addCallback(condition, action);
        return this;
    }

    public TrackingPathBuilder addCallback(PathBuilder.CallbackCondition condition, Runnable action, int i) {
        pedroBuilder.addCallback(condition, action, i);
        return this;
    }

    public TrackingPathBuilder addLoopedCallback(PathCallback callback) {
        pedroBuilder.addLoopedCallback(callback);
        return this;
    }

    public TrackingPathBuilder during(Action action) {
        duringActions.add(action);
        return this;
    }

    public TrackingPathBuilder whenDuring(Callable<Boolean> condition, Runnable code) {
        ActionBuilder builder = Actions.builder();
        builder.waitUntil(() -> {
            try {
                return condition.call();
            } catch (Exception e) {
                // Adapt Callable's checked exception to the unchecked BooleanSupplier; propagate
                // (fail-fast) rather than swallowing a throwing condition into a permanent "false".
                throw new RuntimeException(e);
            }
        });
        builder.run(code);
        duringActions.add(builder.build());
        return this;
    }

    public TrackingPathBuilder whenDuring(Callable<Boolean> condition, Action action) {
        ActionBuilder builder = Actions.builder();
        builder.waitUntil(() -> {
            try {
                return condition.call();
            } catch (Exception e) {
                // Propagate (fail-fast) instead of swallowing a throwing condition into "false".
                throw new RuntimeException(e);
            }
        });
        builder.action(action);
        duringActions.add(builder.build());
        return this;
    }

    public TrackingPathBuilder setHoldEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

    /**
     * Advance the scheduler when {@code follower.getDistanceRemaining() <= distance}, letting
     * Pedro finish the last inch on its own. Lets subsequent segments overlap final convergence.
     */
    public TrackingPathBuilder holdAtDistance(double distance) {
        this.holdAtDistance = distance;
        return this;
    }

    PathChain buildPath() { return pedroBuilder.build(); }
    List<Action> getDuringActions() { return new ArrayList<>(duringActions); }
    boolean isHoldEnd() { return holdEnd; }
    Double getHoldAtDistance() { return holdAtDistance; }
}
