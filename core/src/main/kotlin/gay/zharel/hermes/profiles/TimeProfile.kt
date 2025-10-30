/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.profiles

import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.math.Time
import gay.zharel.hermes.paths.PosePath
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlin.math.sqrt
import kotlin.math.withSign

/**
 * Acceleration-limited motion profile parameterized by time.
 *
 * @param[dispProfile] displacement profile
 * @param[times] time offsets of each displacement sample, starting at 0.0
 */
@Serializable
@SerialName("TimeProfile")
data class TimeProfile @JvmOverloads constructor(
    @JvmField
    val dispProfile: DisplacementProfile,
    @JvmField
    val times: List<Double> = timeScan(dispProfile),
) : Profile {
    @JvmField
    val duration = times.last()

    init {
        require(times.size == dispProfile.disps.size) {
            "times.size() (${times.size}) != dispProfile.disps.size() (${dispProfile.disps.size})"
        }
    }

    override operator fun get(t: Double): DualNum<Time> {
        val index = times.binarySearch(t)
        return when {
            index >= times.lastIndex ->
                DualNum(
                    doubleArrayOf(
                        dispProfile.disps[index], dispProfile.vels[index], 0.0
                    )
                )
            index >= 0 ->
                DualNum(
                    doubleArrayOf(
                        dispProfile.disps[index], dispProfile.vels[index], dispProfile.accels[index]
                    )
                )
            else -> {
                val insIndex = -(index + 1)
                when {
                    insIndex <= 0 -> {
                        val v = dispProfile.vels.first()
                        DualNum(doubleArrayOf(v * t, v, 0.0))
                    }
                    insIndex >= times.size -> {
                        val v = dispProfile.vels.last()
                        DualNum(doubleArrayOf(dispProfile.length + v * (t - duration), v, 0.0))
                    }
                    else -> {
                        val dt = t - times[insIndex - 1]
                        val x0 = dispProfile.disps[insIndex - 1]
                        val v0 = dispProfile.vels[insIndex - 1]
                        val a = dispProfile.accels[insIndex - 1]

                        DualNum(
                            doubleArrayOf(
                                (0.5 * a * dt + v0) * dt + x0,
                                a * dt + v0,
                                a
                            )
                        )
                    }
                }
            }
        }
    }

    fun inverse(x: Double): Double {
        val index = dispProfile.disps.binarySearch(x)
        return when {
            index >= dispProfile.disps.lastIndex -> times[index]
            index >= 0 -> times[index]
            else -> {
                val insIndex = -(index + 1)
                when {
                    insIndex <= 0 -> 0.0
                    insIndex >= times.size -> duration
                    else -> {
                        val dx = x - dispProfile.disps[insIndex - 1]
                        val t0 = times[insIndex - 1]
                        val v0 = dispProfile.vels[insIndex - 1]
                        val a = dispProfile.accels[insIndex - 1]

                        if (a == 0.0) {
                            t0 + dx / v0
                        } else {
                            t0 + sqrt(((v0 * v0 / a) + 2 * dx) / a).withSign(a) - v0 / a
                        }
                    }
                }
            }
        }
    }

    companion object {
        /**
         * Creates an optimal time profile using forward and backward passes.
         * This is the main entry point for generating time-optimal profiles.
         *
         * @param params Profile generation parameters
         * @param path The path to follow
         * @param beginVel Beginning and ending velocity (must be the same to guarantee feasibility)
         * @param velConstraint Velocity constraint function
         * @param accelConstraint Acceleration constraint function
         * @return TimeProfile optimized for the given path and constraints
         */
        fun generate(
            params: ProfileParams,
            path: PosePath,
            beginVel: Double,
            velConstraint: VelConstraint,
            accelConstraint: AccelConstraint,
        ): TimeProfile = TimeProfile(createOptimalDisplacementProfile(
            params, path, beginVel, velConstraint, accelConstraint
        ))
    }
}

private fun timeScan(p: DisplacementProfile): List<Double> {
    val times = mutableListOf(0.0)
    for (i in p.accels.indices) {
        times.add(
            times.last() +
                    if (p.accels[i] == 0.0) {
                        (p.disps[i + 1] - p.disps[i]) / p.vels[i]
                    } else {
                        (p.vels[i + 1] - p.vels[i]) / p.accels[i]
                    }
        )
    }
    return times
}