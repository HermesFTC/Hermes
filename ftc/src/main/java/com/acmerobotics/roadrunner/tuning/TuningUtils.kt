package com.acmerobotics.roadrunner.tuning

import com.acmerobotics.roadrunner.hardware.EncoderGroup
import com.acmerobotics.roadrunner.hardware.LynxQuadratureEncoderGroup

class MidpointTimer {
    private val beginTs = System.nanoTime()
    private var lastTime: Long = 0

    fun seconds(): Double {
        return 1e-9 * (System.nanoTime() - beginTs)
    }

    fun addSplit(): Double {
        val time = System.nanoTime() - beginTs
        val midTimeSecs = 0.5e-9 * (lastTime + time)
        lastTime = time
        return midTimeSecs
    }
}

private class MutableSignal(
    val times: MutableList<Double> = mutableListOf(),
    val values: MutableList<Double> = mutableListOf()
)


// designed for manual bulk caching
private fun recordUnwrappedEncoderData(gs: List<EncoderGroup>, ts: List<Double>, er: EncoderRef, ps: MutableSignal, vs: MutableSignal) {
    val t = ts[er.groupIndex]
    val e = gs[er.groupIndex].unwrappedEncoders[er.index]
    val pv = e.getPositionAndVelocity()

    ps.times.add(t)
    ps.values.add(pv.position.toDouble())

    if (pv.velocity != null) {
        vs.times.add(t)
        vs.values.add(pv.velocity.toDouble())
    }
}

fun shouldFixVels(view: DriveView, er: EncoderRef): Boolean {
    return view.encoderGroups[er.groupIndex] is LynxQuadratureEncoderGroup
}