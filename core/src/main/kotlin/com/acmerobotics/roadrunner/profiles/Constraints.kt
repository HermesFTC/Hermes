package com.acmerobotics.roadrunner.profiles

import com.acmerobotics.roadrunner.geometry.MinMax
import com.acmerobotics.roadrunner.geometry.RobotState
import com.acmerobotics.roadrunner.paths.PosePath
import kotlin.math.abs

fun interface VelConstraint {
    fun maxRobotVel(robotState: RobotState, path: PosePath, s: Double): Double
}

fun interface AccelConstraint {
    fun minMaxProfileAccel(robotState: RobotState, path: PosePath, s: Double): MinMax
}

class TranslationalVelConstraint(
    @JvmField
    val maxTransVel: Double,
) : VelConstraint {
    init {
        require(maxTransVel > 0.0) { "maxTransVel ($maxTransVel) must be positive" }
    }

    override fun maxRobotVel(robotState: RobotState, path: PosePath, s: Double) = maxTransVel
}

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

class MinVelConstraint(
    @JvmField
    val constraints: List<VelConstraint>,
) : VelConstraint {
    override fun maxRobotVel(robotState: RobotState, path: PosePath, s: Double) =
        constraints.minOf { it.maxRobotVel(robotState, path, s) }
}

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