/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.profiles

import kotlin.math.sqrt

/**
 * Core algorithms for generating displacement profiles using forward and backward passes.
 * These functions implement the time-optimal profile generation strategy.
 */

/**
 * Generates a forward displacement profile by enforcing velocity and acceleration constraints.
 * Uses a greedy approach to maximize velocity while respecting constraints.
 *
 * @param disps Displacement interval endpoints
 * @param beginVel Beginning velocity (non-negative)
 * @param maxVels Maximum velocities at each displacement (all positive)
 * @param maxAccels Maximum accelerations at each displacement (all positive)
 * @return Forward displacement profile
 */
fun generateForwardProfile(
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
        .fold(disps[0]) { beginDisp, (constraints, endDisp) ->
            val (maxVel, maxAccel) = constraints
            val currentVel = vels.last()

            when {
                currentVel >= maxVel -> {
                    // Already at or above max velocity - maintain it
                    addConstantVelocitySegment(newDisps, vels, accels, endDisp, maxVel)
                }
                else -> {
                    // Calculate what velocity we'd reach with max acceleration
                    val projectedEndVel = calculateVelocityAfterAcceleration(
                        currentVel, maxAccel, endDisp - beginDisp
                    )

                    if (projectedEndVel <= maxVel) {
                        // Can accelerate for the entire segment
                        addAccelerationSegment(newDisps, vels, accels, endDisp, projectedEndVel, maxAccel)
                    } else {
                        // Need to split: accelerate to max velocity, then maintain it
                        val accelDistance = calculateDistanceToReachVelocity(currentVel, maxVel, maxAccel)
                        addAccelerationSegment(newDisps, vels, accels, beginDisp + accelDistance, maxVel, maxAccel)
                        addConstantVelocitySegment(newDisps, vels, accels, endDisp, maxVel)
                    }
                }
            }
            endDisp
        }

    return DisplacementProfile(newDisps, vels, accels)
}

/**
 * Generates a backward displacement profile by working backwards from the end.
 * This is implemented by reversing the problem and using the forward algorithm.
 *
 * @param disps Displacement interval endpoints
 * @param maxVels Maximum velocities at each displacement (all positive)
 * @param endVel Ending velocity (non-negative)
 * @param minAccels Minimum accelerations at each displacement (all negative)
 * @return Backward displacement profile
 */
fun generateBackwardProfile(
    disps: List<Double>,
    maxVels: List<Double>,
    endVel: Double,
    minAccels: List<Double>,
): DisplacementProfile {
    // Reverse the problem: work backwards by flipping displacements and using positive accelerations
    val reversedDisps = disps.reversed().map { disps.last() - it }
    val reversedMaxVels = maxVels.reversed()
    val reversedMaxAccels = minAccels.reversed().map { -it }

    val forwardProfile = generateForwardProfile(reversedDisps, endVel, reversedMaxVels, reversedMaxAccels)

    // Convert back to original coordinate system
    return DisplacementProfile(
        forwardProfile.disps.map { x -> forwardProfile.length - x }.reversed(),
        forwardProfile.vels.reversed(),
        forwardProfile.accels.reversed().map { a -> -a }
    )
}

/**
 * Generates a voltage-constrained forward profile using motor model parameters.
 *
 * @param disps Displacement interval endpoints
 * @param beginVel Beginning velocity (non-negative)
 * @param kV Velocity constant (velocity per unit voltage)
 * @param kA Acceleration constant (acceleration per unit voltage)
 * @param maxVoltages Maximum voltages at each displacement (all positive)
 * @return Voltage-constrained forward profile
 */
fun generateVoltageConstrainedForwardProfile(
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

    val getVel = velocityFromVoltage(kV)
    val getAccel = accelerationFromVoltage(kA)
    val getVelVolt = voltageFromVelocity(kV)

    maxVoltages
        .zip(disps.drop(1))
        .fold(disps[0]) { beginDisp, (maxVoltage, endDisp) ->
            val currentVel = vels.last()
            val maxVel = getVel(maxVoltage)
            val availableAccelVoltage = maxVoltage - getVelVolt(currentVel)
            val maxAccel = getAccel(availableAccelVoltage)

            when {
                currentVel >= maxVel -> {
                    // Already at voltage-limited velocity
                    addConstantVelocitySegment(newDisps, vels, accels, endDisp, maxVel)
                }
                else -> {
                    val projectedEndVel = calculateVelocityAfterAcceleration(
                        currentVel, maxAccel, endDisp - beginDisp
                    )

                    if (projectedEndVel <= maxVel) {
                        addAccelerationSegment(newDisps, vels, accels, endDisp, projectedEndVel, maxAccel)
                    } else {
                        val accelDistance = calculateDistanceToReachVelocity(currentVel, maxVel, maxAccel)
                        addAccelerationSegment(newDisps, vels, accels, beginDisp + accelDistance, maxVel, maxAccel)
                        addConstantVelocitySegment(newDisps, vels, accels, endDisp, maxVel)
                    }
                }
            }
            endDisp
        }

    return DisplacementProfile(newDisps, vels, accels)
}

/**
 * Generates a voltage-constrained backward profile.
 */
fun generateVoltageConstrainedBackwardProfile(
    disps: List<Double>,
    endVel: Double,
    kV: Double,
    kA: Double,
    maxVoltages: List<Double>,
): DisplacementProfile {
    val reversedProfile = generateVoltageConstrainedForwardProfile(
        disps.reversed().map { disps.last() - it },
        endVel, kV, kA,
        maxVoltages.reversed()
    )

    return DisplacementProfile(
        reversedProfile.disps.map { x -> reversedProfile.length - x }.reversed(),
        reversedProfile.vels.reversed(),
        reversedProfile.accels.reversed().map { a -> -a }
    )
}

// Helper functions for profile generation

private fun addConstantVelocitySegment(
    disps: MutableList<Double>,
    vels: MutableList<Double>,
    accels: MutableList<Double>,
    endDisp: Double,
    velocity: Double
) {
    disps.add(endDisp)
    vels.add(velocity)
    accels.add(0.0)
}

private fun addAccelerationSegment(
    disps: MutableList<Double>,
    vels: MutableList<Double>,
    accels: MutableList<Double>,
    endDisp: Double,
    endVel: Double,
    accel: Double
) {
    disps.add(endDisp)
    vels.add(endVel)
    accels.add(accel)
}

private fun calculateVelocityAfterAcceleration(
    initialVel: Double,
    acceleration: Double,
    distance: Double
): Double {
    return sqrt(initialVel * initialVel + 2 * acceleration * distance)
}

private fun calculateDistanceToReachVelocity(
    initialVel: Double,
    targetVel: Double,
    acceleration: Double
): Double {
    return (targetVel * targetVel - initialVel * initialVel) / (2 * acceleration)
}
