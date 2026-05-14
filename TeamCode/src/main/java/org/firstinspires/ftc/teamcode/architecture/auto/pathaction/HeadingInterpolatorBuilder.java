package org.firstinspires.ftc.teamcode.architecture.auto.pathaction;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import java.util.ArrayList;
import java.util.List;

/**
 * Builds piecewise heading interpolators on top of {@link HeadingInterpolator#piecewise}. Tracks
 * the running t-value so chained segments don't have to repeat their start-t.
 *
 * <p>Example:
 * <pre>{@code
 * new HeadingInterpolatorBuilder()
 *     .linear(0, .2, startHeading, midHeading)  // explicit (startT, endT)
 *     .constant(.6, midHeading)                  // auto-chained: startT = .2
 *     .linear(1, midHeading, endHeading)         // auto-chained: startT = .6
 *     .build();
 * }</pre>
 *
 * @author Maxwell Tham — 6165 MSET Cuttlefish
 * @author Eric Woo-Shem — 6165 MSET Cuttlefish
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
        return HeadingInterpolator.piecewise(nodes.toArray(new HeadingInterpolator.PiecewiseNode[0]));
    }

    public double getCurrentT() {
        return currentT;
    }
}
