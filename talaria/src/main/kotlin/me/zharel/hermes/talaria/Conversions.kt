@file:JvmName("Converter")
package me.zharel.hermes.talaria

import com.acmerobotics.roadrunner.geometry.ChassisSpeeds
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Rotation2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.DurationUnit

typealias WRotation2d = edu.wpi.first.math.geometry.Rotation2d
typealias WPose2d = edu.wpi.first.math.geometry.Pose2d
typealias WChassisSpeeds = edu.wpi.first.math.kinematics.ChassisSpeeds

fun Double.inches(): Distance = Distance.ofRelativeUnits(this, Units.Inches)
fun Double.meters(): Distance = Distance.ofRelativeUnits(this, Units.Meters)

fun Distance.inchValue() = `in`(Units.Inches)
fun Distance.meterValue() = `in`(Units.Meters)

fun Angle.radianValue() = `in`(Units.Radians)

fun Time.duration() = `in`(Units.Milliseconds).milliseconds
fun Duration.time(): Time = Time.ofRelativeUnits(toDouble(DurationUnit.MILLISECONDS), Units.Milliseconds)

fun Vector2d.wpilib() = Translation2d(x.inches().meterValue(), y.inches().meterValue())
fun Translation2d.hermes() = Vector2d(measureX.inchValue(), measureY.inchValue())

fun Rotation2d.wpilib() = WRotation2d(real, imag)
fun WRotation2d.hermes() = Rotation2d(cos, sin)

fun Pose2d.wpilib() =
    WPose2d(position.x.inches(), position.y.inches(), heading.wpilib())
fun WPose2d.hermes() =
    Pose2d(translation.hermes(), rotation.hermes())

fun ChassisSpeeds.wpilib() =
    WChassisSpeeds(
        linearVel.x.inches().meterValue(),
        linearVel.y.inches().meterValue(),
        angularVel
    )
fun WChassisSpeeds.hermes() =
    ChassisSpeeds(
        Vector2d(vxMetersPerSecond, vyMetersPerSecond),
        omegaRadiansPerSecond
    )