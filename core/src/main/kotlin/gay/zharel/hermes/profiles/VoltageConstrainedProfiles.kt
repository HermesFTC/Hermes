/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.profiles

import gay.zharel.hermes.math.rangeCentered
import kotlin.math.ceil
import kotlin.math.max

/**
 * Functions for generating displacement profiles based on voltage constraints and motor models.
 * These profiles use the motor model: V = kV * v + kA * a + kS
 */

/**
 * Generates a complete voltage-constrained profile with cancellation capability.
 * This creates forward and backward profiles, merges them, and adds cancellation constraints.
 *
 * @param length Total displacement length
 * @param beginEndVel Beginning and ending velocity (must be same for feasibility)
 * @param kV Velocity constant (velocity per unit voltage)
 * @param kA Acceleration constant (acceleration per unit voltage)
 * @param kS Static voltage offset (for static friction, etc.)
 * @param maxVoltage Function returning maximum voltage at each displacement
 * @param resolution Sampling resolution for constraints
 * @return CancelableProfile with optimal profile and cancellation constraints
 */
fun createVoltageConstrainedProfile(
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

    // Subtract kS from max voltage to account for static friction
    val availableVoltages = disps.map { maxVoltage(it) - kS }
    val cancellationMinAccels = availableVoltages.map { -accelerationFromVoltage(kA)(it) }

    val forwardProfile = generateVoltageConstrainedForwardProfile(disps, beginEndVel, kV, kA, availableVoltages)
    val backwardProfile = generateVoltageConstrainedBackwardProfile(disps, beginEndVel, kV, kA, availableVoltages)
    val mergedProfile = mergeDisplacementProfiles(forwardProfile, backwardProfile)

    return CancelableProfile(mergedProfile, disps, cancellationMinAccels)
}
