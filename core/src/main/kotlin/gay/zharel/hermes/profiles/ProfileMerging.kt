package gay.zharel.hermes.profiles

import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Algorithms for merging displacement profiles to create optimal combined profiles.
 * The merge operation takes the minimum velocity at each point between two profiles.
 */

/**
 * Merges two displacement profiles by taking the minimum velocity at every point.
 * This is used to combine forward and backward profiles to create a time-optimal profile.
 *
 * @param p1 First displacement profile
 * @param p2 Second displacement profile
 * @return Merged profile with minimum velocities from both input profiles
 */
fun mergeDisplacementProfiles(p1: DisplacementProfile, p2: DisplacementProfile): DisplacementProfile {
    // Both profiles must represent the same displacement interval [0, length]
    val disps = mutableListOf(0.0)
    val vels = mutableListOf(min(p1.vels[0], p2.vels[0]))
    val accels = mutableListOf<Double>()

    var lastMinWasFromP1 = p1.vels[0] < p2.vels[0]
    var i = 1  // Index for p1
    var j = 1  // Index for p2

    while (i < p1.disps.size && j < p2.disps.size) {
        val (endDisp, endVel1, endVel2, accel1, accel2) =
            determineNextMergePoint(p1, p2, i, j)

        // Update indices based on which displacement we processed
        when {
            p1.disps[i] == p2.disps[j] -> { i++; j++ }
            p1.disps[i] < p2.disps[j] -> i++
            else -> j++
        }

        val currentMinIsFromP1 = endVel1 < endVel2

        if (shouldCreateTransitionPoint(currentMinIsFromP1, lastMinWasFromP1, accel1, accel2)) {
            addTransitionPoint(disps, vels, accels, endDisp, endVel1, endVel2, accel1, accel2)
        } else {
            addSingleMergePoint(disps, vels, accels, endDisp, endVel1, endVel2, accel1, accel2, currentMinIsFromP1)
        }

        lastMinWasFromP1 = currentMinIsFromP1
    }

    return DisplacementProfile(disps, vels, accels)
}

/**
 * Data class to hold information about the next merge point between two profiles.
 */
private data class MergePoint(
    val endDisp: Double,
    val endVel1: Double,
    val endVel2: Double,
    val accel1: Double,
    val accel2: Double
)

/**
 * Determines the next displacement point where the profiles should be merged
 * and calculates the corresponding velocities from both profiles.
 */
private fun determineNextMergePoint(
    p1: DisplacementProfile,
    p2: DisplacementProfile,
    i: Int,
    j: Int
): MergePoint {
    val endDisp = min(p1.disps[i], p2.disps[j])
    val accel1 = p1.accels[i - 1]
    val accel2 = p2.accels[j - 1]

    val (endVel1, endVel2) = when {
        p1.disps[i] == p2.disps[j] -> {
            // Both profiles have a point at this displacement
            Pair(p1.vels[i], p2.vels[j])
        }
        p1.disps[i] < p2.disps[j] -> {
            // p1 has a point, p2 needs interpolation
            val interpolatedVel2 = interpolateVelocityBackward(
                p2.vels[j], accel2, p2.disps[j] - p1.disps[i]
            )
            Pair(p1.vels[i], interpolatedVel2)
        }
        else -> {
            // p2 has a point, p1 needs interpolation
            val interpolatedVel1 = interpolateVelocityBackward(
                p1.vels[i], accel1, p1.disps[i] - p2.disps[j]
            )
            Pair(interpolatedVel1, p2.vels[j])
        }
    }

    return MergePoint(endDisp, endVel1, endVel2, accel1, accel2)
}

/**
 * Interpolates velocity by working backward from a known endpoint.
 * This is more numerically stable than forward integration.
 */
private fun interpolateVelocityBackward(
    endVel: Double,
    acceleration: Double,
    distance: Double
): Double {
    // Use max(0.0, ...) to avoid NaNs from negative values under square root
    return sqrt(max(0.0, endVel * endVel - 2 * acceleration * distance))
}

/**
 * Determines if we need to create a transition point where the minimum switches
 * between the two profiles.
 */
private fun shouldCreateTransitionPoint(
    currentMinIsFromP1: Boolean,
    lastMinWasFromP1: Boolean,
    accel1: Double,
    accel2: Double
): Boolean {
    return currentMinIsFromP1 != lastMinWasFromP1 && accel1 != accel2
}

/**
 * Adds a transition point where the minimum velocity switches from one profile to another.
 * This creates two points: the crossover point and the final point.
 */
private fun addTransitionPoint(
    disps: MutableList<Double>,
    vels: MutableList<Double>,
    accels: MutableList<Double>,
    endDisp: Double,
    endVel1: Double,
    endVel2: Double,
    accel1: Double,
    accel2: Double
) {
    // Calculate where the velocities intersect
    val crossoverDistance = (endVel2 * endVel2 - endVel1 * endVel1) / (2 * (accel2 - accel1))
    val crossoverDisp = endDisp - crossoverDistance
    val crossoverVel = sqrt(endVel1 * endVel1 - 2 * accel1 * crossoverDistance)

    // Add crossover point
    disps.add(crossoverDisp)
    vels.add(crossoverVel)
    accels.add(max(accel1, accel2))

    // Add final point
    disps.add(endDisp)
    vels.add(min(endVel1, endVel2))
    accels.add(min(accel1, accel2))
}

/**
 * Adds a single merge point where no transition is needed.
 */
private fun addSingleMergePoint(
    disps: MutableList<Double>,
    vels: MutableList<Double>,
    accels: MutableList<Double>,
    endDisp: Double,
    endVel1: Double,
    endVel2: Double,
    accel1: Double,
    accel2: Double,
    minIsFromP1: Boolean
) {
    disps.add(endDisp)

    if (minIsFromP1) {
        vels.add(endVel1)
        accels.add(accel1)
    } else {
        vels.add(endVel2)
        accels.add(accel2)
    }
}
