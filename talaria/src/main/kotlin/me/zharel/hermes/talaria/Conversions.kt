@file:JvmName("Converter")
package me.zharel.hermes.talaria

import com.acmerobotics.roadrunner.geometry.Acceleration2d
import com.acmerobotics.roadrunner.geometry.DualNum
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Pose2dDual
import com.acmerobotics.roadrunner.geometry.PoseVelocity2d
import com.acmerobotics.roadrunner.geometry.Rotation2d
import com.acmerobotics.roadrunner.geometry.Rotation2dDual
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.geometry.Vector2dDual
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.DurationUnit

internal typealias WRotation2d = edu.wpi.first.math.geometry.Rotation2d
internal typealias WPose2d = edu.wpi.first.math.geometry.Pose2d
internal fun Double.inches(): Distance = Units.Inches.of(this)
internal fun Double.meters(): Distance = Units.Meters.of(this)

internal fun Double.radian(): Angle = Units.Radians.of(this)

internal fun Double.seconds(): Time = Units.Seconds.of(this)

internal fun Distance.inchValue() = `in`(Units.Inches)
internal fun Distance.meterValue() = `in`(Units.Meters)

internal fun Angle.radianValue() = `in`(Units.Radians)

internal fun Time.secondValue() = `in`(Units.Seconds)

internal fun Time.duration() = `in`(Units.Milliseconds).milliseconds
internal fun Duration.time(): Time = Units.Milliseconds.of(this.toDouble(DurationUnit.MILLISECONDS))

internal fun Vector2d.wpilib() = Translation2d(x.inches().meterValue(), y.inches().meterValue())
internal fun Translation2d.hermes() = Vector2d(measureX.inchValue(), measureY.inchValue())

internal fun Rotation2d.wpilib() = WRotation2d(real, imag)
internal fun WRotation2d.hermes() = Rotation2d(cos, sin)

internal fun Pose2d.wpilib() =
    WPose2d(position.x.inches(), position.y.inches(), heading.wpilib())
internal fun WPose2d.hermes() =
    Pose2d(translation.hermes(), rotation.hermes())

internal fun ChassisSpeeds.hermes() =
    PoseVelocity2d(
        Vector2d(
        vx.meters().inchValue(),
        vy.meters().inchValue()
        ),
        omega
    )
internal fun PoseVelocity2d.wpilib() = linearVel.wpilib().let {
        ChassisSpeeds(it.x, it.y, angVel)
    }

internal fun ChassisAcceleration.hermes() = Acceleration2d(
    Vector2d(ax.meters().inchValue(), ay.meters().inchValue()),
    alpha
)
internal fun Acceleration2d.talaria() = ChassisAcceleration(
    linearAcc.x.inches().meterValue(),
    linearAcc.y.inches().meterValue(),
    angAcc
)

fun Pose2dDual<*>.talaria() = RobotMotionState(
    value().wpilib(),
    velocity().value().wpilib(),
    velocity().acceleration().value().talaria()
)

fun <Param> RobotMotionState.hermes() = Pose2dDual(
    Vector2dDual(
        DualNum<Param>(doubleArrayOf(pos.x.meters().inchValue(), vel.vx.meters().inchValue(), acc.ax.meters().inchValue())),
        DualNum(doubleArrayOf(pos.y.meters().inchValue(), vel.vy.meters().inchValue(), acc.ay.meters().inchValue())),
    ),
    Rotation2dDual.exp(DualNum(doubleArrayOf(pos.rotation.radians, vel.omega, acc.alpha)))
)



