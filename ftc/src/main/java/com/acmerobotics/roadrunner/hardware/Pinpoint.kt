package com.acmerobotics.roadrunner.hardware

import com.acmerobotics.roadrunner.ftc.PinpointLocalizer
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.*
import kotlin.math.roundToInt

val DcMotorSimple.Direction.pinpointDirection get() = when(this) {
    DcMotorSimple.Direction.FORWARD -> GoBildaPinpointDriver.EncoderDirection.FORWARD
    DcMotorSimple.Direction.REVERSE -> GoBildaPinpointDriver.EncoderDirection.REVERSED
}

val DcMotorSimple.Direction.multiplier get() = when(this) {
    DcMotorSimple.Direction.FORWARD -> 1
    DcMotorSimple.Direction.REVERSE -> -1
}

class PinpointParEncoder(val pinpoint: PinpointLocalizer) : Encoder {
    override var direction = pinpoint.parDirection

    override fun getPositionAndVelocity() = PositionVelocityPair(
        pinpoint.driver.encoderX * direction.multiplier,
        (pinpoint.driver.getVelX(DistanceUnit.INCH) / pinpoint.inPerTick).roundToInt(),
    )
}

class PinpointPerpEncoder(val pinpoint: PinpointLocalizer) : Encoder {
    override var direction = pinpoint.perpDirection

    override fun getPositionAndVelocity() = PositionVelocityPair(
        pinpoint.driver.encoderY * direction.multiplier,
        (pinpoint.driver.getVelY(DistanceUnit.INCH) / pinpoint.inPerTick).roundToInt(),
    )
}

class PinpointEncoderGroup(
    val pinpoint: PinpointLocalizer,
) : EncoderGroup {
    override val encoders = listOf(
        PinpointParEncoder(pinpoint),
        PinpointPerpEncoder(pinpoint),
    )
    override val unwrappedEncoders = encoders

    override fun bulkRead() {
        pinpoint.update()
    }
}

// Only the methods used by tuning routines are implemented
class PinpointIMU(val pinpoint: PinpointLocalizer) : IMU, LazyImu {
    override fun getManufacturer() = HardwareDevice.Manufacturer.Other
    override fun getDeviceName() = ""
    override fun getConnectionInfo() = ""
    override fun getVersion() = 0

    override fun resetDeviceConfigurationForOpMode() {}

    override fun close() {}

    override fun initialize(parameters: IMU.Parameters?) = true

    override fun resetYaw() {
        throw NotImplementedError()
    }

    override fun getRobotYawPitchRollAngles(): YawPitchRollAngles {
        throw NotImplementedError()
    }

    override fun getRobotOrientation(
        reference: AxesReference?,
        order: AxesOrder?,
        angleUnit: AngleUnit?
    ): Orientation {
        throw NotImplementedError()
    }

    override fun getRobotOrientationAsQuaternion(): Quaternion {
        throw NotImplementedError()
    }

    override fun getRobotAngularVelocity(angleUnit: AngleUnit): AngularVelocity {
        pinpoint.update()
        return AngularVelocity(angleUnit, 0.0f, 0.0f,
            pinpoint.driver.getHeadingVelocity(angleUnit.unnormalized).toFloat(),
            0L)
    }

    override fun get() = this
}