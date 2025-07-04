package com.acmerobotics.roadrunner.profiles

import com.acmerobotics.roadrunner.geometry.Arclength
import com.acmerobotics.roadrunner.geometry.MinMax
import com.acmerobotics.roadrunner.geometry.Pose2dDual
import com.acmerobotics.roadrunner.paths.PosePath
import kotlin.math.abs

fun interface VelConstraint {
    fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): Double
}

fun interface AccelConstraint {
    fun minMaxProfileAccel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): MinMax
}

class TranslationalVelConstraint(
    @JvmField
    val minTransVel: Double,
) : VelConstraint {
    init {
        require(minTransVel > 0.0) { "minTransVel ($minTransVel) must be positive" }
    }

    override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double) = minTransVel
}

class AngularVelConstraint(
    @JvmField
    val maxAngVel: Double,
) : VelConstraint {
    init {
        require(maxAngVel > 0.0) { "maxAngVel ($maxAngVel) must be positive" }
    }

    override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double) =
        abs(maxAngVel / robotPose.heading.velocity().value())
}

class MinVelConstraint(
    @JvmField
    val constraints: List<VelConstraint>,
) : VelConstraint {
    override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double) =
        constraints.minOf { it.maxRobotVel(robotPose, path, s) }
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

    override fun minMaxProfileAccel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double) = minMax
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

    override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): Double {
        for ((offset, constraint) in offsets.zip(constraints).drop(1).reversed()) {
            if (s >= offset) {
                return constraint.maxRobotVel(robotPose, path, s)
            }
        }

        return constraints.first().maxRobotVel(robotPose, path, s)
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

    override fun minMaxProfileAccel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): MinMax {
        for ((offset, constraint) in offsets.zip(constraints).drop(1).reversed()) {
            if (s >= offset) {
                return constraint.minMaxProfileAccel(robotPose, path, s)
            }
        }

        return constraints.first().minMaxProfileAccel(robotPose, path, s)
    }
}