package com.acmerobotics.roadrunner.tuning

import com.acmerobotics.dashboard.config.variable.CustomVariable
import com.acmerobotics.roadrunner.control.MecanumKinematics
import com.acmerobotics.roadrunner.control.MotorFeedforward
import com.acmerobotics.roadrunner.control.TankKinematics
import com.acmerobotics.roadrunner.ftc.PinpointLocalizer
import com.acmerobotics.roadrunner.geometry.PoseVelocity2d
import com.acmerobotics.roadrunner.geometry.PoseVelocity2dDual
import com.acmerobotics.roadrunner.geometry.Time
import com.acmerobotics.roadrunner.hardware.Encoder
import com.acmerobotics.roadrunner.hardware.EncoderGroup
import com.acmerobotics.roadrunner.hardware.LazyImu
import com.acmerobotics.roadrunner.hardware.LynxQuadratureEncoderGroup
import com.acmerobotics.roadrunner.hardware.PinpointParEncoder
import com.acmerobotics.roadrunner.logs.FlightRecorder
import com.google.gson.annotations.SerializedName
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.max


enum class DriveType {
    @SerializedName("mecanum")
    MECANUM,

    @SerializedName("tank")
    TANK
}

fun interface FeedforwardFactory {
    fun make(): MotorFeedforward
}

data class EncoderRef(
    val groupIndex: Int,
    val index: Int,
)

/**
 * % of max movement. for instance, if encoder a moves 1000 ticks and encoder b moves 100, we consider b to also have moved.
 */
const val MOVEMENT_PERCENTAGE_THRESHOLD = 0.1

interface ForwardPushLocalizerView {

    val parameters: ForwardPushLocalizerParameters

}

class ForwardPushPinpointView(hardwareMap: HardwareMap) : ForwardPushLocalizerView {

    // safety checks
    init {
        val pinpoints = hardwareMap.getAll(GoBildaPinpointDriver::class.java)
        if (pinpoints.size > 1) {
            FlightRecorder.write(
                "PINPOINT_CONFIGURATION_WARNING",
                "Multiple pinpoints detected. Using first: " + pinpoints.iterator().next().deviceName,
            )
        } else if (pinpoints.isEmpty()) {
            throw ConfigurationException("No pinpoints detected in configuration.")
        }
    }

    val pinpoint: GoBildaPinpointDriver = hardwareMap.getAll(GoBildaPinpointDriver::class.java).iterator().next()
        ?: throw ConfigurationException("No pinpoints detected in configuration.")

    val pinpointParPod = TuningAxialSensor({ pinpoint.encoderX.toDouble() })
    val pinpointPerpPod = TuningAxialSensor({ pinpoint.encoderY.toDouble() })

    override val parameters: ForwardPushPinpointParameters get() {

        if (!(pinpointPerpPod.moved xor pinpointParPod.moved)) {
            throw ConfigurationException("your pinpoint is plugged in wrong silly")
        }

        val podConfig = if (pinpointParPod.moved) PinpointEncoderType.PARALLEL else PinpointEncoderType.PERPENDICULAR

        val podDirection = when (podConfig) {
            PinpointEncoderType.PARALLEL -> pinpointParPod.direction
            PinpointEncoderType.PERPENDICULAR -> pinpointPerpPod.direction
        }

        // clean up this cursed config code later
        val ticksPerInch = when (podConfig) {
            PinpointEncoderType.PARALLEL -> pinpointParPod.value
            PinpointEncoderType.PERPENDICULAR -> pinpointPerpPod.value
        } / (HermesConfig["ForwardPush"] as CustomVariable).getVariable("InchesTravelled").value as Double

        return ForwardPushPinpointParameters(
            pinpoint.deviceName,
            podConfig,
            podDirection,
            ticksPerInch
        )
    }

}

/**
 * Relative axial sensor class that measures movement throughout the OpMode.
 * Examples: motor encoder, odometry pod.
 */
open class TuningAxialSensor(val sensorOutput: Function0<Double>, val movementThreshold: Double = 100.0) {

    val moved get() = abs(sensorOutput.invoke()) > movementThreshold
    val direction get() = if (sensorOutput.invoke() > 0) DcMotorSimple.Direction.FORWARD else DcMotorSimple.Direction.REVERSE
    val value get() = sensorOutput.invoke()

}

class TuningMotorEncoder(motor: DcMotorEx) : TuningAxialSensor({ motor.currentPosition.toDouble() }, 30.0)

/**
 * you fucked up your config and now i am mad at you
 */
class ConfigurationException(message: String? = null, cause: Throwable? = null) : Exception(message, cause) {
    constructor(cause: Throwable) : this(null, cause)
}

