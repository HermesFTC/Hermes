/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.tuning

import gay.zharel.hermes.math.lerp
import kotlinx.serialization.Serializable
import java.util.LinkedList
import java.util.Queue
import kotlin.math.abs

object DataFilter {

    /**
     * Sync the timing of the quasistatic test. Uses voltage timings by default
     */
    fun syncQuasistatic(params: QuasistaticParameters): QuasistaticParameters {
        return QuasistaticParameters(
            params.voltages,
            params.velocities.fromTimes(
                params.voltages.times
            )
        )
    }

    /**
     * Remove quasistatic test datapoints that do not meet the threshold velocity.
     */
    fun filterQuasistaticByVelocity(thresholdVelocity: Double, params: QuasistaticParameters): QuasistaticParameters {
        // ensure timing is the same so we can iterate
        // this also ensures equal numbers of elements
        val syncedParams = syncQuasistatic(params)

        // iterate and filter (cursed)
        for (i in params.velocities.times.indices.reversed()) {
            if (params.velocities.values[i] < thresholdVelocity) {
                params.velocities.values.removeAt(i)
                params.velocities.times.removeAt(i)
                params.voltages.values.removeAt(i)
                params.voltages.times.removeAt(i)
            }

        }

        return params
    }

    // do interpolated lut shit
    /**
     * Interpolates with the given times to find new values.
     */
    fun MutableSignal.fromTimes(times: MutableList<Double>): MutableSignal {
        val nextInterpolationPoint: Queue<Pair<Double, Double>> = LinkedList()
        nextInterpolationPoint.addAll(this.asPair())

        if (nextInterpolationPoint.isEmpty()) {
            return MutableSignal(times, mutableListOf())
        }

        var prevInterpolationPoint: Pair<Double, Double> = nextInterpolationPoint.first()

        val newValues: MutableList<Double> = arrayListOf()

        for (t in times) {

            if (nextInterpolationPoint.isNotEmpty() && t > nextInterpolationPoint.first().first) {
                prevInterpolationPoint = nextInterpolationPoint.first()
            }

            newValues.add(
                lerp(
                    t,
                    prevInterpolationPoint.first,
                    nextInterpolationPoint.first().first,
                    prevInterpolationPoint.second,
                    nextInterpolationPoint.first().second
                )
            )

        }

        return MutableSignal(
            times,
            newValues
        )
    }

}

@Serializable
data class QuasistaticParameters(
    val voltages: MutableSignal = MutableSignal(),
    val velocities: MutableSignal = MutableSignal()
)

@Serializable
data class DynamicParameters(
    val voltages: MutableSignal = MutableSignal(),
    val accelerations: MutableSignal = MutableSignal()
)