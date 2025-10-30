/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.profiles

import gay.zharel.hermes.math.MinMax
import gay.zharel.hermes.geometry.RobotState
import gay.zharel.hermes.paths.PosePath
import kotlin.math.abs

/**
 * Constraint on robot velocity during motion profiling.
 *
 * Implementations of this interface define the maximum allowable robot velocity
 * at a given point along a path.
 */
fun interface VelConstraint {
    /**
     * Returns the maximum robot velocity at the specified state and path position.
     *
     * @param robotState The current state of the robot
     * @param path The path being followed
     * @param s The arc length parameter along the path
     * @return The maximum allowable robot velocity
     */
    fun maxRobotVel(robotState: RobotState, path: PosePath, s: Double): Double
}

/**
 * Constraint on robot acceleration during motion profiling.
 *
 * Implementations of this interface define the allowable acceleration range
 * at a given point along a path.
 */
fun interface AccelConstraint {
    /**
     * Returns the minimum and maximum profile acceleration at the specified state and path position.
     *
     * @param robotState The current state of the robot
     * @param path The path being followed
     * @param s The arc length parameter along the path
     * @return A [MinMax] containing the minimum and maximum allowable accelerations
     */
    fun minMaxProfileAccel(robotState: RobotState, path: PosePath, s: Double): MinMax
}

/**
 * Velocity constraint based on maximum translational velocity.
 *
 * This constraint ensures that the robot's translational velocity does not exceed
 * a specified maximum value at any point along the path.
 *
 * @property maxTransVel The maximum translational velocity (must be positive)
 * @throws IllegalArgumentException if [maxTransVel] is not positive
 */
class TranslationalVelConstraint(
    @JvmField
    val maxTransVel: Double,
) : VelConstraint {
    init {
        require(maxTransVel > 0.0) { "maxTransVel ($maxTransVel) must be positive" }
    }

    override fun maxRobotVel(robotState: RobotState, path: PosePath, s: Double) = maxTransVel
}

/**
 * Velocity constraint based on maximum angular velocity.
 *
 * This constraint ensures that the robot's angular velocity does not exceed
 * a specified maximum value. It dynamically calculates the maximum allowable
 * robot velocity based on the current angular velocity requirement.
 *
 * @property maxAngVel The maximum angular velocity (must be positive)
 * @throws IllegalArgumentException if [maxAngVel] is not positive
 */
class AngularVelConstraint(
    @JvmField
    val maxAngVel: Double,
) : VelConstraint {
    init {
        require(maxAngVel > 0.0) { "maxAngVel ($maxAngVel) must be positive" }
    }

    override fun maxRobotVel(robotState: RobotState, path: PosePath, s: Double) =
        abs(maxAngVel / robotState.vel.angVel)
}

/**
 * Composite velocity constraint that enforces the minimum of multiple constraints.
 *
 * This constraint evaluates all provided velocity constraints and returns
 * the most restrictive (minimum) maximum velocity. This is useful for combining
 * multiple constraints such as translational and angular velocity limits.
 *
 * @property constraints List of velocity constraints to combine
 */
class MinVelConstraint(
    @JvmField
    val constraints: List<VelConstraint>,
) : VelConstraint {
    override fun maxRobotVel(robotState: RobotState, path: PosePath, s: Double) =
        constraints.minOf { it.maxRobotVel(robotState, path, s) }
}

/**
 * Acceleration constraint defined by constant minimum and maximum acceleration values.
 *
 * This constraint enforces fixed acceleration limits throughout the entire path.
 * The minimum acceleration (for deceleration) must be negative, and the maximum
 * acceleration must be positive.
 *
 * @property minAccel The minimum acceleration value (must be negative)
 * @property maxAccel The maximum acceleration value (must be positive)
 * @throws IllegalArgumentException if [minAccel] is not negative or [maxAccel] is not positive
 */
class ProfileAccelConstraint(
    @JvmField
    val minAccel: Double,
    @JvmField
    val maxAccel: Double,
) : AccelConstraint {
    init {
        require(minAccel < 0.0) { "minAccel ($minAccel) must be negative" }
        require(maxAccel > 0.0) { "maxAccel ($maxAccel) must be positive" }
    }

    private val minMax = MinMax(minAccel, maxAccel)

    override fun minMaxProfileAccel(robotState: RobotState, path: PosePath, s: Double) = minMax
}

/**
 * Composite velocity constraint that applies different constraints at different path positions.
 *
 * This constraint allows for varying velocity limits along the path by switching between
 * different constraints at specified arc length offsets. The constraint searches through
 * the offsets from largest to smallest to determine which constraint applies at a given position.
 *
 * @property constraints List of velocity constraints to apply in different regions
 * @property offsets List of arc length offsets defining when each constraint becomes active.
 *                   Must have size equal to `constraints.size + 1`
 * @throws IllegalArgumentException if the size relationship between constraints and offsets is invalid
 */
class CompositeVelConstraint(
    @JvmField
    val constraints: List<VelConstraint>,
    @JvmField
    val offsets: List<Double>
) : VelConstraint {
    init {
        require(constraints.size + 1 == offsets.size) {
            "constraints.size() (${constraints.size}) + 1 != offsets.size() (${offsets.size})"
        }
    }

    override fun maxRobotVel(robotState: RobotState, path: PosePath, s: Double): Double {
        for ((offset, constraint) in offsets.zip(constraints).drop(1).reversed()) {
            if (s >= offset) {
                return constraint.maxRobotVel(robotState, path, s)
            }
        }

        return constraints.first().maxRobotVel(robotState, path, s)
    }
}

/**
 * Composite acceleration constraint that applies different constraints at different path positions.
 *
 * This constraint allows for varying acceleration limits along the path by switching between
 * different constraints at specified arc length offsets. The constraint searches through
 * the offsets from largest to smallest to determine which constraint applies at a given position.
 *
 * @property constraints List of acceleration constraints to apply in different regions
 * @property offsets List of arc length offsets defining when each constraint becomes active.
 *                   Must have size equal to `constraints.size + 1`
 * @throws IllegalArgumentException if the size relationship between constraints and offsets is invalid
 */
class CompositeAccelConstraint(
    @JvmField
    val constraints: List<AccelConstraint>,
    @JvmField
    val offsets: List<Double>
) : AccelConstraint {
    init {
        require(constraints.size + 1 == offsets.size) {
            "constraints.size() (${constraints.size}) + 1 != offsets.size() (${offsets.size})"
        }
    }

    override fun minMaxProfileAccel(robotState: RobotState, path: PosePath, s: Double): MinMax {
        for ((offset, constraint) in offsets.zip(constraints).drop(1).reversed()) {
            if (s >= offset) {
                return constraint.minMaxProfileAccel(robotState, path, s)
            }
        }

        return constraints.first().minMaxProfileAccel(robotState, path, s)
    }
}