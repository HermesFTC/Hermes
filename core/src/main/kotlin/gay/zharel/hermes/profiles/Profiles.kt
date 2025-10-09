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

/**
 * @usesMathJax
 *
 * Represents a motion profile that maps from one parameter space to time with derivatives.
 *
 * A profile defines how to traverse a path over time, typically encoding velocity and
 * acceleration constraints.
 * It maps from a displacement or other parameter \(x\) to
 * time \(t\) along with time derivatives, enabling computation of velocities and accelerations.
 *
 * Profiles are essential for trajectory generation, converting geometric paths into
 * time-parameterized trajectories that respect kinematic and dynamic constraints.
 */
interface Profile {
    /**
     * @usesMathJax
     *
     * Evaluates the profile at parameter \(x\) to obtain time and its derivatives.
     *
     * Returns a dual number containing:
     * - Value: time \(t(x)\) at the given parameter
     * - First derivative: \(dt/dx\) (inverse velocity with respect to the parameter)
     * - Higher derivatives as applicable
     *
     * @param x The parameter value at which to evaluate the profile
     * @return A [DualNum] of type [Time] containing time and its derivatives with respect to the parameter
     */
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
