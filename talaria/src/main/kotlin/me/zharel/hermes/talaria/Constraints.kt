@file:JvmName("Constraints")
package me.zharel.hermes.talaria

import com.acmerobotics.roadrunner.geometry.Arclength
import com.acmerobotics.roadrunner.geometry.MinMax
import com.acmerobotics.roadrunner.geometry.Pose2dDual
import com.acmerobotics.roadrunner.paths.PosePath
import com.acmerobotics.roadrunner.profiles.AccelConstraint
import com.acmerobotics.roadrunner.profiles.AngularVelConstraint
import com.acmerobotics.roadrunner.profiles.MinVelConstraint
import com.acmerobotics.roadrunner.profiles.ProfileAccelConstraint
import com.acmerobotics.roadrunner.profiles.TranslationalVelConstraint
import com.acmerobotics.roadrunner.profiles.VelConstraint
import edu.wpi.first.units.LinearAccelerationUnit
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity

val InchesPerSecondPerSecond: LinearAccelerationUnit = Units.InchesPerSecond.per(Units.Second)

fun interface VelocityConstraint : VelConstraint {
    fun maxRobotVel(state: RobotMotionState, path: Path, disp: Distance): LinearVelocity

    override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): Double =
        maxRobotVel(robotPose.talaria(), PathWrapper(path), s.inches()).`in`(Units.InchesPerSecond)
}

fun interface AccelerationConstraint : AccelConstraint {
    fun minMaxProfileAccel(state: RobotMotionState, path: Path, disp: Distance): MinMaxAccels

    override fun minMaxProfileAccel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double) =
        minMaxProfileAccel(robotPose.talaria(), PathWrapper(path), s.inches()).let {
            MinMax(it.min.`in`(Units.InchesPerSecondPerSecond), it.max.`in`(Units.InchesPerSecondPerSecond))
        }

    data class MinMaxAccels(
        val min: LinearAcceleration,
        val max: LinearAcceleration
    )
}

fun translationalVelConstraint(maxVel: LinearVelocity): VelConstraint =
    TranslationalVelConstraint(maxVel.`in`(Units.InchesPerSecond))

fun angularVelConstraint(maxVel: AngularVelocity) =
    AngularVelConstraint(maxVel.`in`(Units.RadiansPerSecond))

fun combinedVelConstraint(maxLinVel: LinearVelocity, maxAngVel: AngularVelocity) =
    MinVelConstraint(listOf(
        translationalVelConstraint(maxLinVel),
        angularVelConstraint(maxAngVel)
    ))

fun accelConstraint(minAccel: LinearAcceleration, maxAccel: LinearAcceleration) =
    ProfileAccelConstraint(minAccel.`in`(InchesPerSecondPerSecond), maxAccel.`in`(InchesPerSecondPerSecond))