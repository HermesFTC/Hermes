@file:JvmName("Constraints")
package me.zharel.hermes.wpi

import com.acmerobotics.roadrunner.profiles.AngularVelConstraint
import com.acmerobotics.roadrunner.profiles.MinVelConstraint
import com.acmerobotics.roadrunner.profiles.ProfileAccelConstraint
import com.acmerobotics.roadrunner.profiles.TranslationalVelConstraint
import edu.wpi.first.units.LinearAccelerationUnit
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity

val InchesPerSecondPerSecond: LinearAccelerationUnit = Units.InchesPerSecond.per(Units.Second)

fun translationalVelConstraint(maxVel: LinearVelocity) =
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