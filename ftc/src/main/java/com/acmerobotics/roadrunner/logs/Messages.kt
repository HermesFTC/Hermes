package com.acmerobotics.roadrunner.logs

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.PoseVelocity2dDual
import com.acmerobotics.roadrunner.geometry.Time
import com.acmerobotics.roadrunner.hardware.PositionVelocityPair
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles

class DriveCommandMessage(poseVelocity: PoseVelocity2dDual<Time?>) {
    var timestamp: Long = System.nanoTime()
    var forwardVelocity: Double = poseVelocity.linearVel.x[0]
    var forwardAcceleration: Double = poseVelocity.linearVel.x[1]
    var lateralVelocity: Double = poseVelocity.linearVel.y[0]
    var lateralAcceleration: Double = poseVelocity.linearVel.y[1]
    var angularVelocity: Double = poseVelocity.angVel[0]
    var angularAcceleration: Double = poseVelocity.angVel[1]
}

class MecanumCommandMessage(
    var voltage: Double,
    var leftFrontPower: Double,
    var leftBackPower: Double,
    var rightBackPower: Double,
    var rightFrontPower: Double
) {
    var timestamp: Long = System.nanoTime()
}

class MecanumLocalizerInputsMessage(
    var leftFront: PositionVelocityPair,
    var leftBack: PositionVelocityPair,
    var rightBack: PositionVelocityPair,
    var rightFront: PositionVelocityPair,
    angles: YawPitchRollAngles
) {
    var timestamp: Long = System.nanoTime()
    var yaw: Double = angles.getYaw(AngleUnit.RADIANS)
    var pitch = angles.getPitch(AngleUnit.RADIANS)
    var roll = angles.getRoll(AngleUnit.RADIANS)
}

class TankLocalizerInputsMessage(
    left: MutableList<PositionVelocityPair?>,
    right: MutableList<PositionVelocityPair?>
) {
    var timestamp: Long = System.nanoTime()
    var left: Array<PositionVelocityPair?>? = left.toTypedArray<PositionVelocityPair?>()
    var right: Array<PositionVelocityPair?>? = right.toTypedArray<PositionVelocityPair?>()
}

class TankCommandMessage(var voltage: Double, var leftPower: Double, var rightPower: Double) {
    var timestamp: Long = System.nanoTime()
}

class PoseMessage(pose: Pose2d) {
    var timestamp: Long = System.nanoTime()
    var x: Double = pose.position.x
    var y: Double = pose.position.y
    var heading: Double = pose.heading.toDouble()
}

class ThreeDeadWheelInputsMessage(
    var par0: PositionVelocityPair?,
    var par1: PositionVelocityPair?,
    var perp: PositionVelocityPair?
) {
    var timestamp: Long = System.nanoTime()
}

class TwoDeadWheelInputsMessage(
    var par: PositionVelocityPair?,
    var perp: PositionVelocityPair?,
    angles: YawPitchRollAngles,
    angularVelocity: AngularVelocity
) {
    var timestamp: Long = System.nanoTime()
    var yaw = angles.getYaw(AngleUnit.RADIANS)
    var pitch = angles.getPitch(AngleUnit.RADIANS)
    var roll = angles.getRoll(AngleUnit.RADIANS)

    var xRotationRate = angularVelocity.xRotationRate.toDouble()
    var yRotationRate = angularVelocity.yRotationRate.toDouble()
    var zRotationRate = angularVelocity.zRotationRate.toDouble()
}
