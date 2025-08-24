package com.acmerobotics.roadrunner.profiles

import com.acmerobotics.roadrunner.geometry.DualNum
import com.acmerobotics.roadrunner.geometry.RobotState
import com.acmerobotics.roadrunner.geometry.Time
import com.acmerobotics.roadrunner.geometry.range
import com.acmerobotics.roadrunner.geometry.rangeCentered
import com.acmerobotics.roadrunner.paths.PosePath
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlin.math.*
/**
 * Acceleration-limited motion profile parameterized by displacement.
 *
 * @param[disps] displacements, beginning at zero and sorted ascending
 * @param[vels] velocities at [disps] values
 * @param[accels] constant accelerations applied over each displacement interval
 */
@Serializable
@SerialName("DisplacementProfile")
data class DisplacementProfile(
    @JvmField
    val disps: List<Double>,
    @JvmField
    val vels: List<Double>,
    @JvmField
    val accels: List<Double>,
) : Profile {
    @JvmField
    val length = disps.last()

    init {
        require(disps.size == vels.size) {
            "disps.size() (${disps.size}) != vels.size() (${vels.size})"
        }
        require(disps.size == accels.size + 1) {
            "disps.size() (${disps.size}) != accels.size() + 1 (${accels.size + 1})"
        }
    }

    override operator fun get(x: Double): DualNum<Time> {
        val index = disps.binarySearch(x)
        return when {
            index >= disps.lastIndex -> DualNum(doubleArrayOf(x, vels[index], 0.0))
            index >= 0 -> DualNum(doubleArrayOf(x, vels[index], accels[index]))
            else -> {
                val insIndex = -(index + 1)
                when {
                    insIndex <= 0 -> DualNum(doubleArrayOf(x, vels.first(), 0.0))
                    insIndex >= disps.size -> DualNum(doubleArrayOf(x, vels.last(), 0.0))
                    else -> {
                        val dx = x - disps[insIndex - 1]
                        val v0 = vels[insIndex - 1]
                        val a = accels[insIndex - 1]

                        DualNum(
                            doubleArrayOf(
                                x,
                                sqrt(v0 * v0 + 2 * a * dx),
                                a
                            )
                        )
                    }
                }
            }
        }
    }

    companion object {
        fun create(
            params: ProfileParams,
            path: PosePath,
            beginVel: Double,
            velConstraint: VelConstraint,
            accelConstraint: AccelConstraint,
        ) = merge(
            forwardProfile(params, path, beginVel, velConstraint, accelConstraint),
            backwardProfile(params, path, beginVel, velConstraint, accelConstraint)
        )
    }
}

/**
 * Computes an approximately time-optimal forward profile by sampling the constraints according to the resolution
 * [resolution]. No restriction is imposed on the minimum acceleration.
 *
 * @param[beginVel] beginning velocity, non-negative
 * @param[maxVel] always returns positive
 * @param[maxAccel] always returns positive
 */
fun forwardProfile(
    length: Double,
    beginVel: Double,
    maxVel: (Double) -> Double,
    maxAccel: (Double) -> Double,
    resolution: Double,
): DisplacementProfile {
    val samples = max(1, ceil(length / resolution).toInt())

    val disps = rangeCentered(0.0, length, samples)
    val maxVels = disps.map(maxVel)
    val maxAccels = disps.map(maxAccel)
    return forwardProfile(
        range(0.0, length, samples + 1),
        beginVel, maxVels, maxAccels
    )
}

/**
 * Computes an approximately time-optimal forward profile from the center-sampled constraints. No restriction is imposed
 * on the minimum acceleration.
 *
 * The procedure uses a variant of the approach described in [section 14.6.3.5](http://lavalle.pl/planning/node794.html)
 * of LaValle's excellent book on planning.
 *
 * @param[disps] displacement interval endpoints
 * @param[beginVel] beginning velocity, non-negative
 * @param[maxVels] all positive
 * @param[maxAccels] all positive
 */
@Suppress("NAME_SHADOWING")
fun forwardProfile(
    disps: List<Double>,
    beginVel: Double,
    maxVels: List<Double>,
    maxAccels: List<Double>,
): DisplacementProfile {
    require(beginVel >= 0.0) { "beginVel ($beginVel) must be non-negative" }
    require(maxVels.all { v -> v > 0.0 }) { "maxVels must be positive" }
    require(maxAccels.all { v -> v > 0.0 }) { "maxAccels must be positive" }

    val newDisps = mutableListOf(0.0)
    val vels = mutableListOf(beginVel)
    val accels = mutableListOf<Double>()

    maxVels
        .zip(maxAccels)
        .zip(disps.drop(1))
        .fold(disps[0]) { beginDisp, (c, endDisp) ->
            val (maxVel, maxAccel) = c

            val beginVel = vels.last()
            if (beginVel >= maxVel) {
                newDisps.add(endDisp)
                vels.add(maxVel)
                accels.add(0.0)
            } else {
                val endVel = sqrt(beginVel * beginVel + 2 * maxAccel * (endDisp - beginDisp))
                if (endVel <= maxVel) {
                    newDisps.add(endDisp)
                    vels.add(endVel)
                    accels.add(maxAccel)
                } else {
                    val accelDx = (maxVel * maxVel - beginVel * beginVel) / (2 * maxAccel)

                    newDisps.add(beginDisp + accelDx)
                    vels.add(maxVel)
                    accels.add(maxAccel)

                    newDisps.add(endDisp)
                    vels.add(maxVel)
                    accels.add(0.0)
                }
            }

            endDisp
        }

    return DisplacementProfile(newDisps, vels, accels)
}

/**
 * Computes an approximately time-optimal backward profile by sampling the constraints according to the resolution
 * [resolution]. No restriction is imposed on the minimum acceleration.
 *
 * @param[maxVel] always returns positive
 * @param[endVel] ending velocity, non-negative
 * @param[minAccel] always returns negative
 */
fun backwardProfile(
    length: Double,
    maxVel: (Double) -> Double,
    endVel: Double,
    minAccel: (Double) -> Double,
    resolution: Double,
): DisplacementProfile {
    require(endVel >= 0.0) { "endVel ($endVel) must be non-negative" }

    val samples = max(1, ceil(length / resolution).toInt())

    val disps = rangeCentered(0.0, length, samples)
    val maxVels = disps.map(maxVel)
    val minAccels = disps.map(minAccel)
    return backwardProfile(
        range(0.0, length, samples + 1),
        maxVels, endVel, minAccels
    )
}

/**
 * Computes an approximately time-optimal backward profile from the center-sampled constraints. No restriction is imposed
 * on the maximum acceleration.
 *
 * @param[disps] displacement interval endpoints
 * @param[maxVels] all positive
 * @param[endVel] ending velocity, non-negative
 * @param[minAccels] all negative
 */
fun backwardProfile(
    disps: List<Double>,
    maxVels: List<Double>,
    endVel: Double,
    minAccels: List<Double>,
) = forwardProfile(
    disps.reversed().map { disps.last() - it }, endVel,
    maxVels.reversed(), minAccels.reversed().map { -it }
).let {
    DisplacementProfile(
        it.disps.map { x -> it.length - x }.reversed(),
        it.vels.reversed(),
        it.accels.reversed().map { a -> -a },
    )
}

/**
 * Merges [p1] and [p2] into another profile with the minimum velocity of the two at every point.
 */
fun merge(p1: DisplacementProfile, p2: DisplacementProfile): DisplacementProfile {
    // implicit requirements: p1, p2 represent same displacement interval [0, length]
    // TODO: this limitation is not really necessary and somewhat adversely affects cancellation profiles
    val disps = mutableListOf(0.0)
    val vels = mutableListOf(min(p1.vels[0], p2.vels[0]))
    val accels = mutableListOf<Double>()

    var lastMin1 = p1.vels[0] < p2.vels[0]

    var i = 1
    var j = 1
    while (i < p1.disps.size && j < p2.disps.size) {
        val endDisp = min(p1.disps[i], p2.disps[j])
        val accel1 = p1.accels[i - 1]
        val accel2 = p2.accels[j - 1]

        val (endVel1, endVel2) =
            if (p1.disps[i] == p2.disps[j]) {
                val p = Pair(
                    p1.vels[i],
                    p2.vels[j],
                )
                i++
                j++
                p
            } else if (p1.disps[i] < p2.disps[j]) {
                val p = Pair(
                    p1.vels[i],
                    // compute the intermediate velocity, working back from the endpoint
                    //   wny not compute velocities forward and use disps.last()?
                    //   not really sure, though this might be more numerically stable / accumulate less error
                    // TODO: I don't like max(0.0, ...) here, but it's better than NaNs
                    sqrt(
                        max(
                            0.0,
                            p2.vels[j] * p2.vels[j] -
                                2 * accel2 * (p2.disps[j] - p1.disps[i])
                        )
                    )
                )
                i++
                p
            } else {
                val p = Pair(
                    sqrt(
                        max(
                            0.0,
                            p1.vels[i] * p1.vels[i] -
                                2 * accel1 * (p1.disps[i] - p2.disps[j])
                        )
                    ),
                    p2.vels[j]
                )
                j++
                p
            }

        val min1 = endVel1 < endVel2
        if (min1 == lastMin1) {
            disps.add(endDisp)
            if (min1) {
                vels.add(endVel1)
                accels.add(accel1)
            } else {
                vels.add(endVel2)
                accels.add(accel2)
            }
        } else if (accel1 == accel2) {
            // this case mostly avoids a NaN below in weird cases
            // usually accel1 == accel2 implies min1 == lastMin1
            disps.add(endDisp)
            vels.add(min(endVel1, endVel2))
            accels.add(accel1)
        } else {
            val dx = (endVel2 * endVel2 - endVel1 * endVel1) / (2 * (accel2 - accel1))
            disps.add(endDisp - dx)
            vels.add(sqrt(endVel1 * endVel1 - 2 * accel1 * dx))
            accels.add(max(accel1, accel2))

            disps.add(endDisp)
            vels.add(min(endVel1, endVel2))
            accels.add(min(accel1, accel2))
        }

        lastMin1 = min1
    }

    return DisplacementProfile(disps, vels, accels)
}

fun forwardProfile(
    params: ProfileParams,
    path: PosePath,
    beginVel: Double,
    velConstraint: VelConstraint,
    accelConstraint: AccelConstraint,
): DisplacementProfile {
    val len = path.length()
    val dispSamples = rangeCentered(0.0, len, max(1, ceil(len / params.dispResolution).toInt()))
    val angSamples = samplePathByRotation(path, params.angResolution, params.angSamplingEps)
    val samples = (dispSamples + angSamples).sorted()

    val maxVels = mutableListOf<Double>()
    val maxAccels = mutableListOf<Double>()

    for (s in samples) {
        val pose = path[s, 2]

        maxVels.add(velConstraint.maxRobotVel(RobotState.fromDualPose(pose), path, s))

        val (_, maxAccel) = accelConstraint.minMaxProfileAccel(RobotState.fromDualPose(pose), path, s)
        maxAccels.add(maxAccel)
    }

    return forwardProfile(
        listOf(0.0) + samples.zip(samples.drop(1)).map { (a, b) -> 0.5 * (a + b) } + listOf(path.length()),
        beginVel, maxVels, maxAccels,
    )
}

fun backwardProfile(
    params: ProfileParams,
    path: PosePath,
    endVel: Double,
    velConstraint: VelConstraint,
    accelConstraint: AccelConstraint,
): DisplacementProfile {
    val len = path.length()
    val dispSamples = rangeCentered(0.0, len, max(1, ceil(len / params.dispResolution).toInt()))
    val angSamples = samplePathByRotation(path, params.angResolution, params.angSamplingEps)
    val samples = (dispSamples + angSamples).sorted()

    val maxVels = mutableListOf<Double>()
    val minAccels = mutableListOf<Double>()

    for (s in samples) {
        val pose = path[s, 2]

        maxVels.add(velConstraint.maxRobotVel(RobotState.fromDualPose(pose), path, s))

        val (minAccel, _) = accelConstraint.minMaxProfileAccel(RobotState.fromDualPose(pose), path, s)
        minAccels.add(minAccel)
    }

    return backwardProfile(
        listOf(0.0) + samples.zip(samples.drop(1)).map { (a, b) -> 0.5 * (a + b) } + listOf(path.length()),
        maxVels, endVel, minAccels,
    )
}

/**
 * Computes an approximately time-optimal profile with the given constraints and resolution.
 *
 * @param[beginEndVel] beginning and ending velocity, non-negative (must be the same to guarantee feasibility)
 * @param kV Unit of velocity per unit of voltage (u voltage should stabilize at u * kV velocity)
 * @param kA Unit of acceleration per unit of voltage (starting from 0, u voltage should accelerate at u * kA acceleration)
 * @param kS Static voltage offset, to account for forces like static friction. Nonnegative.
 * @param maxVoltage Always returns positive
 */
fun profile(
    length: Double,
    beginEndVel: Double,
    kV: Double,
    kA: Double,
    kS: Double,
    maxVoltage: (Double) -> Double,
    resolution: Double,
): CancelableProfile {
    require(length > 0.0) { "length ($length) must be positive" }
    require(resolution > 0.0) { "resolution ($resolution) must be positive" }
    require(beginEndVel >= 0.0) { "beginEndVel ($beginEndVel) must be non-negative" }

    val samples = max(1, ceil(length / resolution).toInt())

    val disps = rangeCentered(0.0, length, samples)

    // subtract off kS here to be lazy
    val actualMaxVoltages = disps.map { maxVoltage(it) - kS }

    val cancellationMinAccels = actualMaxVoltages.map { -kA * it }

    return CancelableProfile(
        merge(
            forwardProfile(disps, beginEndVel, kV, kA, actualMaxVoltages),
            backwardProfile(disps, beginEndVel, kV, kA, actualMaxVoltages),
        ),
        disps, cancellationMinAccels
    )
}

/**
 * Computes an approximately time-optimal forward profile from voltage constraints. No restriction is imposed
 * on the minimum acceleration.
 *
 * The procedure uses me and not ryan brott's procedure because having disp-based max vel and acc is stupid.
 *
 * @param[disps] displacement interval endpoints
 * @param[beginVel] beginning velocity, non-negative
 * @param[maxVoltages] all positive
 */
@Suppress("NAME_SHADOWING")
fun forwardProfile(
    disps: List<Double>,
    beginVel: Double,
    kV: Double,
    kA: Double,
    maxVoltages: List<Double>
): DisplacementProfile {
    require(beginVel >= 0.0) { "beginVel ($beginVel) must be non-negative" }
    require(maxVoltages.all { v -> v > 0.0 }) { "maxVoltages must be positive" }

    val newDisps = mutableListOf(0.0)
    val vels = mutableListOf(beginVel)
    val accels = mutableListOf<Double>()

    val getVel = velocity(kV)
    val getAccel = acceleration(kA)

    val getVelVolt = voltageFromVelocity(kV)

    maxVoltages
        .zip(disps.drop(1))
        .fold(disps[0]) { beginDisp, (maxVoltage, endDisp) ->
            val beginVel = vels.last()
            val maxVel = getVel(maxVoltage)
            val maxAccel = getAccel(maxVoltage - getVelVolt(beginVel))

            // if our vel is greater than the constraint, drop to the constraint (backwards pass will take care of accel here)
            if (beginVel >= maxVel) {
                newDisps.add(endDisp)
                vels.add(maxVel)
                accels.add(0.0)
            } else {
                // approximate using constant accel (from initial velocity)
                val endVel = sqrt(beginVel * beginVel + 2.0 * maxAccel * (endDisp - beginDisp))

                // if less than max vel, add state
                if (endVel <= maxVel) {
                    newDisps.add(endDisp)
                    vels.add(endVel)
                    accels.add(maxAccel)
                } else {
                    // otherwise, go only as far as necessary to reach max velocity and add a new sample there to ensure optimality
                    val accelDx = (maxVel * maxVel - beginVel * beginVel) / (2 * maxAccel)

                    newDisps.add(beginDisp + accelDx)
                    vels.add(maxVel)
                    accels.add(maxAccel)

                    // then append the original sample with the same velocity
                    newDisps.add(endDisp)
                    vels.add(maxVel)
                    accels.add(0.0)
                }
            }

            // set the accumulator to endDisp to ensure the next beginDisp is correct
            endDisp
        }

    return DisplacementProfile(newDisps, vels, accels)
}

/**
 * Computes an approximately time-optimal backward profile from voltage constraints. No restriction is imposed
 * on the maximum acceleration.
 *
 * The procedure uses me and not ryan brott's procedure because having disp-based max vel and acc is stupid.
 *
 * @param[disps] displacement interval endpoints
 * @param[endVel] ending velocity, non-negative
 * @param[maxVoltages] all positive
 */
fun backwardProfile(
    disps: List<Double>,
    endVel: Double,
    kV: Double,
    kA: Double,
    maxVoltages: List<Double>,
) = forwardProfile(
    disps.reversed().map { disps.last() - it }, endVel,
    kV, kA, maxVoltages.reversed()
).let {
    DisplacementProfile(
        it.disps.map { x -> it.length - x }.reversed(),
        it.vels.reversed(),
        it.accels.reversed().map { a -> -a },
    )
}

fun velocity(kV: Double): (Double) -> Double {
    return { availableVoltage: Double -> availableVoltage * kV }
}

fun voltageFromVelocity(kV: Double): (Double) -> Double {
    return { velocity: Double -> velocity / kV }
}

fun acceleration(kA: Double): (Double) -> Double {
    return { availableVoltage: Double -> availableVoltage * kA }
}

fun voltageFromAcceleration(kA: Double): (Double) -> Double {
    return { velocity: Double -> velocity / kA }
}

operator fun DisplacementProfile.plus(other: DisplacementProfile): DisplacementProfile {
    require(this.vels.last() == other.vels.first())
        { "this.vels.last() (${this.vels.last()}) != other.vels.first() (${other.vels.first()})" }

    return DisplacementProfile(
        this.disps + other.disps.drop(1),
        this.vels + other.vels.drop(1),
        this.accels + other.accels
    )
}


