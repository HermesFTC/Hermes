package com.acmerobotics.roadrunner.tuning

import com.acmerobotics.roadrunner.hardware.Encoder
import com.acmerobotics.roadrunner.hardware.EncoderGroup
import kotlinx.serialization.Serializable

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

@Serializable
class MutableSignal(
    val times: MutableList<Double> = mutableListOf(),
    val values: MutableList<Double> = mutableListOf()
) {
    fun asPair(): MutableList<Pair<Double, Double>> {
        return this.times.zip(this.values) as MutableList<Pair<Double, Double>>
    }
}

internal fun avgPos(es: List<Encoder>) = es.sumOf { it.getPositionAndVelocity().position.toDouble() } / es.size

