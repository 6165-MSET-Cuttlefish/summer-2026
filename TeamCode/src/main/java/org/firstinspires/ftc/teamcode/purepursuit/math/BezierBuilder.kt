/*
 * Copyright (c) 2026 Vikram Janakiraman. All rights reserved.
 */
package org.firstinspires.ftc.teamcode.purepursuit.math


class BezierBuilder(@JvmField val points: MutableList<Vector>, @JvmField val headingTargets: MutableList<ParametricHeading>, @JvmField val searchRadiusTargets: MutableList<SearchRadiusTarget>) {
    fun <T> MutableList<T>.plus(t: T): MutableList<T> {
        return this.toMutableList().apply { add(t) }
    }
    constructor(): this(emptyList<Vector>().toMutableList(), emptyList<ParametricHeading>().toMutableList(), emptyList<SearchRadiusTarget>().toMutableList())

    fun addControlPoint(point: Vector): BezierBuilder {
        return BezierBuilder(points.plus(point), headingTargets, searchRadiusTargets)
    }
    fun addControlPoint() = addControlPoint(Vector(0.0, 0.0))

    fun addControlPoint(x: Double, y: Double) = addControlPoint(Vector(x,y))
    fun p(point: Vector) = addControlPoint(point)
    fun p(x: Double, y: Double) = addControlPoint(x,y)

    fun addHeadingTarget(heading: ParametricHeading): BezierBuilder {
        return BezierBuilder(points, headingTargets.plus(heading), searchRadiusTargets)
    }

    fun addSearchRadiusTarget(target: SearchRadiusTarget): BezierBuilder {
        return BezierBuilder(points, headingTargets, searchRadiusTargets.plus(target))
    }
    fun addSearchRadiusTarget(searchRadius: Double, t: Double): BezierBuilder = addSearchRadiusTarget(SearchRadiusTarget(searchRadius, t))


    fun clear() {
        points.clear()
        headingTargets.clear()
    }

    fun addHeadingTarget(heading: Double, t: Double) = addHeadingTarget(ParametricHeading(heading,t))
    fun h(heading: ParametricHeading) = addHeadingTarget(heading)
    fun h(heading: Double, t: Double) = addHeadingTarget(heading, t)

    fun build(): Bezier {
        val points = when (this.points.size) {
            3 -> this.points.toMutableList()
            2 -> this.points.toMutableList()
            !in 0..1000 -> throw IllegalArgumentException("You're not cooking. Finish the bezier curve.")
            else -> this.points.toMutableList()
        }

        val headingTargets = this.headingTargets.toMutableList()
        val searchRadiusTargets = this.searchRadiusTargets.toMutableList()

        this.points.clear()
        this.headingTargets.clear()
        this.searchRadiusTargets.clear()

        val bezier = Bezier(points, headingTargets)

        return bezier
    }
}