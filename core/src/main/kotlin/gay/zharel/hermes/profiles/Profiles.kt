@file:JvmName("Profiles")

package gay.zharel.hermes.profiles

import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.math.Time
import gay.zharel.hermes.math.integralScan
import gay.zharel.hermes.math.lerpLookupMap
import gay.zharel.hermes.math.rangeCentered
import gay.zharel.hermes.paths.PosePath
import kotlin.math.*

/**
 * Parameters for motion profile sampling and angular sampling.
 *
 * @property dispResolution Linear displacement sampling resolution (inches).
 * @property angResolution Angular sampling resolution (radians).
 * @property angSamplingEps Epsilon for angular integration (radians).
 */
data class ProfileParams(
    val dispResolution: Double,
    val angResolution: Double,
    val angSamplingEps: Double,
)

interface Profile {
    operator fun get(x: Double): DualNum<Time>
}

fun samplePathByRotation(
    path: PosePath,
    angResolution: Double,
    eps: Double,
): List<Double> {
    val (values, sums) = integralScan(0.0, path.length(), eps) {
        // TODO: this is pretty wasteful
        abs(path[it, 2].heading.velocity().value())
    }

    return lerpLookupMap(
        sums, values,
        rangeCentered(
            0.0, sums.last(),
            max(1, ceil(sums.last() / angResolution).toInt())
        )
    )
}


