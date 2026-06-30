/*
 * Copyright (c) 2026 Vikram Janakiraman. All rights reserved.
 */
package org.firstinspires.ftc.teamcode.purepursuit.math

class BezierPathBuilder(@JvmField val beziers: MutableList<Bezier>) {
    fun <T> MutableList<T>.plus(t: T): MutableList<T> {
        return this.toMutableList().apply { add(t) }
    }

    constructor(): this(emptyList<Bezier>().toMutableList())

    fun addBezier(bezier: Bezier):BezierPathBuilder {
        return BezierPathBuilder(beziers.plus(bezier))
    }
    fun clear() {
        beziers.clear()
    }
    fun build(): BezierPath {
        val beziers = this.beziers.toMutableList()
        this.beziers.clear()
        return BezierPath(beziers)
    }
}