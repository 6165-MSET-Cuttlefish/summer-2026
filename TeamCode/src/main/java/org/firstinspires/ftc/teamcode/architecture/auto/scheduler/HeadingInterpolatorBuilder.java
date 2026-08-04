package org.firstinspires.ftc.teamcode.architecture.auto.scheduler;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import java.util.ArrayList;
import java.util.List;

/**
 * Builds piecewise {@link HeadingInterpolator}s, tracking the running t so chained segments don't
 * have to repeat their start-t.
 */
public class HeadingInterpolatorBuilder {
    private final List<HeadingInterpolator.PiecewiseNode> nodes = new ArrayList<>();
    private double currentT = 0;

    public HeadingInterpolatorBuilder linear(double startT, double endT, double startHeadingRad, double endHeadingRad) {
        nodes.add(HeadingInterpolator.PiecewiseNode.linear(startT, endT, startHeadingRad, endHeadingRad));
        currentT = endT;
        return this;
    }

    public HeadingInterpolatorBuilder linear(double endT, double startHeadingRad, double endHeadingRad) {
        return linear(currentT, endT, startHeadingRad, endHeadingRad);
    }

    public HeadingInterpolatorBuilder reversedLinear(double startT, double endT, double startHeadingRad, double endHeadingRad) {
        nodes.add(HeadingInterpolator.PiecewiseNode.reversedLinear(startT, endT, startHeadingRad, endHeadingRad));
        currentT = endT;
        return this;
    }

    public HeadingInterpolatorBuilder reversedLinear(double endT, double startHeadingRad, double endHeadingRad) {
        return reversedLinear(currentT, endT, startHeadingRad, endHeadingRad);
    }

    public HeadingInterpolatorBuilder constant(double startT, double endT, double headingRad) {
        nodes.add(new HeadingInterpolator.PiecewiseNode(startT, endT, HeadingInterpolator.constant(headingRad)));
        currentT = endT;
        return this;
    }

    public HeadingInterpolatorBuilder constant(double endT, double headingRad) {
        return constant(currentT, endT, headingRad);
    }

    public HeadingInterpolatorBuilder tangent(double startT, double endT) {
        nodes.add(new HeadingInterpolator.PiecewiseNode(startT, endT, HeadingInterpolator.tangent));
        currentT = endT;
        return this;
    }

    public HeadingInterpolatorBuilder tangent(double endT) {
        return tangent(currentT, endT);
    }

    public HeadingInterpolatorBuilder facingPoint(double startT, double endT, double x, double y) {
        nodes.add(new HeadingInterpolator.PiecewiseNode(startT, endT, HeadingInterpolator.facingPoint(x, y)));
        currentT = endT;
        return this;
    }

    public HeadingInterpolatorBuilder facingPoint(double endT, double x, double y) {
        return facingPoint(currentT, endT, x, y);
    }

    public HeadingInterpolatorBuilder facingPoint(double startT, double endT, Pose pose) {
        nodes.add(new HeadingInterpolator.PiecewiseNode(startT, endT, HeadingInterpolator.facingPoint(pose)));
        currentT = endT;
        return this;
    }

    public HeadingInterpolatorBuilder facingPoint(double endT, Pose pose) {
        return facingPoint(currentT, endT, pose);
    }

    public HeadingInterpolatorBuilder custom(double startT, double endT, HeadingInterpolator interpolator) {
        nodes.add(new HeadingInterpolator.PiecewiseNode(startT, endT, interpolator));
        currentT = endT;
        return this;
    }

    public HeadingInterpolatorBuilder custom(double endT, HeadingInterpolator interpolator) {
        return custom(currentT, endT, interpolator);
    }

    public HeadingInterpolator build() {
        if (nodes.isEmpty()) {
            throw new IllegalStateException("HeadingInterpolatorBuilder: no segments added");
        }
        // Pedro accepts overlapping/backwards t ranges silently; reject them rather than hand back
        // a confusing interpolator.
        for (int i = 0; i < nodes.size(); i++) {
            HeadingInterpolator.PiecewiseNode n = nodes.get(i);
            if (n.getFinalTValue() <= n.getInitialTValue()) {
                throw new IllegalStateException(
                        "HeadingInterpolatorBuilder: segment " + i + " has endT ("
                                + n.getFinalTValue() + ") <= startT (" + n.getInitialTValue() + ")");
            }
            if (i > 0 && n.getInitialTValue() < nodes.get(i - 1).getFinalTValue()) {
                throw new IllegalStateException(
                        "HeadingInterpolatorBuilder: segment " + i + " starts at "
                                + n.getInitialTValue() + " before previous segment ends at "
                                + nodes.get(i - 1).getFinalTValue()
                                + " — add segments in ascending t order");
            }
        }
        return HeadingInterpolator.piecewise(nodes.toArray(new HeadingInterpolator.PiecewiseNode[0]));
    }

    public double getCurrentT() {
        return currentT;
    }
}
