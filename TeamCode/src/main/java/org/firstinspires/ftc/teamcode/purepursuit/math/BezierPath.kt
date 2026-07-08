/*
 * Copyright (c) 2026 Vikram Janakiraman. All rights reserved.
 */
package org.firstinspires.ftc.teamcode.purepursuit.math

import kotlin.math.abs
import kotlin.math.floor


data class BezierPath(val beziers: MutableList<Bezier>): Path {
    var headingTargets: MutableList<ParametricHeading> = emptyList<ParametricHeading>().toMutableList()
        get() {
            return field
        }

    var searchRadTargets: MutableList<SearchRadiusTarget> = emptyList<SearchRadiusTarget>().toMutableList()
        get() {
            return field
        }

    init {
        headingTargets = beziers.flatMapIndexed { index, bezier -> bezier.headingTargets.map { headingTarget -> ParametricHeading(headingTarget.heading, headingTarget.t + index) }}.toMutableList()
        searchRadTargets = beziers.flatMapIndexed { index, bezier -> bezier.searchRadTargets.map { target -> SearchRadiusTarget(target.searchRadius, target.t + index) }}.toMutableList()
    }

    fun headingTargetsExist(): Boolean = headingTargets.size != 0
    fun searchRadTargetsExist(): Boolean = searchRadTargets.size != 0
    @Override
    override fun isFollowable(): Boolean {
        var int = 0
        val numOfIntersects = beziers.size - 1
        var count = 0
        for (curve in beziers) {
            if (int != 0) {
                val prevEnd = beziers[int - 1].end()
                val curStart = curve.start()
                if (abs(prevEnd.x - curStart.x) < 1e-6 && abs(prevEnd.y - curStart.y) < 1e-6) {
                    count += 1
                }
            }
            int++
        }
        return count == numOfIntersects
    }
    @Override
    override fun length(): Double {
        var length = 0.0
        for (curve in beziers) {
            length += curve.length()
        }
        return length
    }
    fun solve(t: Double): Pose? {
        if (t in 0.0..maxT()) {
            val int = minOf(floor(t).toInt(), beziers.size - 1)
            return beziers[int].solve(t - int)
        }
        else return null
    }
    fun findT(pose: Pose): Double? {
        var increase = 0
        var t = 0.0
        for (bezier in beziers) {
            if (bezier.isOn(pose)) {
                t = increase + bezier.findT(pose)
            }
            increase ++
        }
        return if (t <= maxT()) t
        else null
    }
    fun findT(pose: ParametricPose): Double {
        return pose.t
    }
//    fun draw(canvas: Canvas) {
//        for (bezier in beziers) {
//            bezier.draw(canvas)
//        }
//    }

    fun reverse(): BezierPath {
        val curves = beziers.reversed().map { it.reverse() }.toMutableList()
        return BezierPath(curves)
    }

    fun maxT(): Double {
        return beziers.size.toDouble()
    }
    fun start(): ParametricPose {
        return beziers[0].start().setT(0.0)
    }
    fun end(): ParametricPose {
        return beziers[beziers.size - 1].end().setT(beziers.size.toDouble())
    }
}
