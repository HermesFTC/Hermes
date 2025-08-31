package gay.zharel.hermes.profiles

import gay.zharel.hermes.geometry.RobotState
import gay.zharel.hermes.geometry.range
import gay.zharel.hermes.geometry.rangeCentered
import gay.zharel.hermes.paths.PosePath
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlin.math.ceil
import kotlin.math.max
import kotlin.math.sqrt

/**
 * Displacement profile that can be canceled at any time to yield a new displacement profile
 * that achieves the final velocity as soon as possible and then promptly ends.
 *
 * Cancellation profiles begin with a displacement of zero regardless of the provided cancellation
 * displacement in the base profile.
 *
 * Cancellation doesn't modify the base displacement profile, allowing for multiple cancellations.
 */
@Serializable
@SerialName("CancelableProfile")
class CancelableProfile(
    @JvmField val baseProfile: DisplacementProfile,
    @JvmField val disps: List<Double>,
    @JvmField val minAccels: List<Double>
) : Profile by baseProfile {
    fun cancel(x: Double): DisplacementProfile {
        val newDisps = mutableListOf(0.0)
        val vels = mutableListOf(baseProfile[x][1])
        val accels = mutableListOf<Double>()

        val rawIndex = this.disps.binarySearch(x)
        val beginIndex = if (rawIndex >= 0) {
            rawIndex
        } else {
            val insIndex = -(rawIndex + 1)
            insIndex
        }

        if (beginIndex == 0) {
            return DisplacementProfile(listOf(0.0), listOf(vels.first()), emptyList())
        }

        val targetVel = baseProfile.vels.last()
        for (index in beginIndex..disps.lastIndex) {
            val v = vels.last()
            val a = minAccels[index - 1]

            val targetDisp = newDisps.last() + (targetVel * targetVel - v * v) / (2 * a)
            if (x + targetDisp > disps[index]) {
                newDisps.add(disps[index] - x)
                vels.add(sqrt(v * v + 2 * a * (this.disps[index] - this.disps[index - 1])))
                accels.add(a)
            } else {
                newDisps.add(targetDisp)
                vels.add(targetVel)
                accels.add(a)

                break
            }
        }

        return DisplacementProfile(newDisps, vels, accels)
    }

    /**
     * Shortcut for the original displacement profile.
     */
    operator fun invoke() = baseProfile

    companion object {
        /**
         * Creates a path-based cancelable profile using the given constraints.
         *
         * @param params Profile generation parameters
         * @param path The path to follow
         * @param beginEndVel Beginning and ending velocity (must be the same to guarantee feasibility)
         * @param velConstraint Velocity constraint function
         * @param accelConstraint Acceleration constraint function
         * @return CancelableProfile optimized for the given path and constraints
         */
        fun generate(
            params: ProfileParams,
            path: PosePath,
            beginEndVel: Double,
            velConstraint: VelConstraint,
            accelConstraint: AccelConstraint,
        ): CancelableProfile {
            val len = path.length()
            val dispSamples = rangeCentered(0.0, len, max(1, ceil(len / params.dispResolution).toInt()))
            val angSamples = samplePathByRotation(path, params.angResolution, params.angSamplingEps)
            val samples = (dispSamples + angSamples).sorted()

            val maxVels = mutableListOf<Double>()
            val minAccels = mutableListOf<Double>()
            val maxAccels = mutableListOf<Double>()

            for (s in samples) {
                val pose = path[s, 2]

                maxVels.add(velConstraint.maxRobotVel(RobotState.fromDualPose(pose), path, s))

                val (minAccel, maxAccel) = accelConstraint.minMaxProfileAccel(RobotState.fromDualPose(pose), path, s)
                minAccels.add(minAccel)
                maxAccels.add(maxAccel)
            }

            return generateCancelableProfile(
                listOf(0.0) + samples.zip(samples.drop(1)).map { (a, b) -> 0.5 * (a + b) } + listOf(path.length()),
                beginEndVel, maxVels, minAccels, maxAccels
            )
        }
    }
}

/**
 * Computes an exact, time-optimal profile.
 *
 * @param[beginEndVel] beginning and ending velocity (must be the same to guarantee feasibility)
 * @param[maxVel] positive
 * @param[minAccel] negative
 * @param[maxAccel] positive
 */
fun constantProfile(
    length: Double,
    beginEndVel: Double,
    maxVel: Double,
    minAccel: Double,
    maxAccel: Double,
) = generateCancelableProfile(length, beginEndVel, { maxVel }, { minAccel }, { maxAccel }, length)

/**
 * Computes an approximately time-optimal profile by sampling the constraints according to the resolution [resolution].
 *
 * @param[beginEndVel] beginning and ending velocity, non-negative (must be the same to guarantee feasibility)
 * @param[maxVel] always returns positive
 * @param[minAccel] always returns negative
 * @param[maxAccel] always returns positive
 */
fun generateCancelableProfile(
    length: Double,
    beginEndVel: Double,
    maxVel: (Double) -> Double,
    minAccel: (Double) -> Double,
    maxAccel: (Double) -> Double,
    resolution: Double,
): CancelableProfile {
    require(length > 0.0) { "length ($length) must be positive" }
    require(resolution > 0.0) { "resolution ($resolution) must be positive" }
    require(beginEndVel >= 0.0) { "beginEndVel ($beginEndVel) must be non-negative" }

    val samples = max(1, ceil(length / resolution).toInt())

    val disps = rangeCentered(0.0, length, samples)
    val maxVels = disps.map(maxVel)
    val minAccels = disps.map(minAccel)
    val maxAccels = disps.map(maxAccel)

    return generateCancelableProfile(
        range(0.0, length, samples + 1),
        beginEndVel, maxVels, minAccels, maxAccels
    )
}

/**
 * Computes an approximately time-optimal profile from sampled constraints.
 *
 * @param[beginEndVel] beginning and ending velocity (must be the same to guarantee feasibility)
 * @param[maxVels] all positive
 * @param[minAccels] all negative
 * @param[maxAccels] all positive
 */
fun generateCancelableProfile(
    disps: List<Double>,
    beginEndVel: Double,
    maxVels: List<Double>,
    minAccels: List<Double>,
    maxAccels: List<Double>,
): CancelableProfile {
    require(maxVels.size == minAccels.size) {
        "maxVels.size() (${maxVels.size}) != minAccels.size() (${minAccels.size})"
    }
    require(maxVels.size == maxAccels.size) {
        "maxVels.size() (${maxVels.size}) != maxAccels.size() (${maxAccels.size})"
    }

    return CancelableProfile(
        mergeDisplacementProfiles(
            generateForwardProfile(disps, beginEndVel, maxVels, maxAccels),
            generateBackwardProfile(disps, maxVels, beginEndVel, minAccels),
        ),
        disps, minAccels
    )
}
