package me.zharel.hermes.talaria

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.wpilibj.RobotState

data class ChassisAcceleration(
    val ax: Double,
    val ay: Double,
    val alpha: Double
) {
    constructor(ax: LinearAcceleration, ay: LinearAcceleration, omega: AngularAcceleration) : this(
                ax.`in`(Units.MetersPerSecondPerSecond),
                ay.`in`(Units.MetersPerSecondPerSecond),
                omega.`in`(Units.RadiansPerSecondPerSecond)
    )

    operator fun plus(other: ChassisAcceleration) = ChassisAcceleration(
        ax + other.ax,
        ay + other.ay,
        alpha + other.alpha
    )

    operator fun minus(other: ChassisAcceleration) = ChassisAcceleration(
        ax - other.ax,
        ay - other.ay,
        alpha - other.alpha
    )

    operator fun times(scalar: Double) = ChassisAcceleration(
        ax * scalar,
        ay * scalar,
        alpha * scalar
    )

    operator fun div(scalar: Double) = ChassisAcceleration(
        ax / scalar,
        ay / scalar,
        alpha / scalar
    )

    fun toRobotRelative(robotAngle: Rotation2d) : ChassisAcceleration {
        val rotated = Translation2d(ax, ay).rotateBy(-robotAngle)
        return ChassisAcceleration(rotated.x, rotated.y, alpha)
    }

    fun toFieldRelative(robotAngle: Rotation2d) : ChassisAcceleration {
        val rotated = Translation2d(ax, ay).rotateBy(robotAngle)
        return ChassisAcceleration(rotated.x, rotated.y, alpha)
    }
}

data class RobotMotionState(
    val pos: Pose2d,
    val vel: ChassisSpeeds,
    val acc: ChassisAcceleration
)