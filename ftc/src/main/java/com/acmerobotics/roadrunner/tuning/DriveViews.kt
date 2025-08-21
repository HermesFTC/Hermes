package com.acmerobotics.roadrunner.tuning

import com.acmerobotics.roadrunner.control.MotorFeedforward
import com.acmerobotics.roadrunner.logs.FlightRecorder
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.serialization.SerialName
import kotlin.math.abs


enum class DriveType {
    @SerialName("mecanum")
    MECANUM,

    @SerialName("tank")
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

    fun getParameters(actualInchesTravelled: Double): ForwardPushLocalizerParameters

}

class ForwardPushPinpointView(hardwareMap: HardwareMap) : ForwardPushLocalizerView {

    val pinpoint: GoBildaPinpointDriver =
        hardwareMap.getAll(GoBildaPinpointDriver::class.java).iterator().next()
            ?: throw ConfigurationException("No pinpoints detected in configuration.")

    val pinpointParPod = TuningAxialSensor({ pinpoint.encoderX.toDouble() })
    val pinpointPerpPod = TuningAxialSensor({ pinpoint.encoderY.toDouble() })

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

    override fun getParameters(actualInchesTravelled: Double): ForwardPushPinpointParameters {

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
        } / actualInchesTravelled

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
open class TuningAxialSensor(val sensorOutput: () -> Double, val movementThreshold: Double = 100.0) {
    val moved get() = abs(sensorOutput()) > movementThreshold
    val direction get() = if (sensorOutput() > 0) DcMotorSimple.Direction.FORWARD else DcMotorSimple.Direction.REVERSE
    val value get() = sensorOutput()
}

class TuningMotorEncoder(motor: DcMotorEx) : TuningAxialSensor({ motor.currentPosition.toDouble() }, 30.0)

/**
 * you fucked up your config and now i am mad at you
 */
class ConfigurationException(message: String? = null, cause: Throwable? = null) : Exception(message, cause) {
    constructor(cause: Throwable) : this(null, cause)
}

