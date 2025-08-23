package com.acmerobotics.roadrunner.tuning

import com.acmerobotics.roadrunner.control.MecanumKinematics
import com.acmerobotics.roadrunner.control.MotorFeedforward
import com.acmerobotics.roadrunner.geometry.DualNum
import com.acmerobotics.roadrunner.geometry.PoseVelocity2dDual
import com.acmerobotics.roadrunner.geometry.Time
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.geometry.Vector2dDual
import com.acmerobotics.roadrunner.logs.FlightRecorder
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.serialization.SerialName
import kotlin.math.PI
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
 * Global object for synchronized and IO-efficient voltage reads.
 */
object VoltageCache {

    private lateinit var voltageSensor: VoltageSensor
    private var cachedVoltage: Double = 0.0
    private val timeElapsed: ElapsedTime = ElapsedTime()

    private val voltageConfig: VoltageConfig by PersistentConfigDelegate(
        "Voltage Configuration", VoltageConfig(),
        HermesConfig,
    )

    val nominalVoltage: Double by voltageConfig::nominalVoltage

    private val readIntervalSeconds: Double by voltageConfig::readIntervalSeconds

    val currentVoltage: Double
        get() {
            if (timeElapsed.seconds() > readIntervalSeconds) {
                cachedVoltage = voltageSensor.voltage
                timeElapsed.reset()
            }
            return cachedVoltage
        }

    /**
     * Initializes the VoltageCache with the voltage sensor for the OpMode.
     */
    fun init(hardwareMap: HardwareMap) {
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next()
        cachedVoltage = voltageSensor.voltage
        timeElapsed.reset()
    }


}

interface ForwardPushLocalizerView {

    fun getParameters(actualInchesTravelled: Double): ForwardPushLocalizerParameters

}

open class PinpointView(hardwareMap: HardwareMap) {
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
}

class ForwardPushPinpointView(hardwareMap: HardwareMap) : PinpointView(hardwareMap), ForwardPushLocalizerView {

    override fun getParameters(actualInchesTravelled: Double): ForwardPushPinpointParameters {

        if (!(pinpointPerpPod.moved xor pinpointParPod.moved)) {
            throw ConfigurationException("your pinpoint is plugged in wrong silly")
        }

        // can take this out later but for now catch this here
        if (pinpointPerpPod.moved) {
            throw ConfigurationException("your pinpoint pods are swapped silly, swap the wires")
        }

        val podConfig = if (pinpointParPod.moved) PinpointEncoderType.PARALLEL else PinpointEncoderType.PERPENDICULAR

        val podDirection = when (podConfig) {
            PinpointEncoderType.PARALLEL -> pinpointParPod.direction
            PinpointEncoderType.PERPENDICULAR -> pinpointPerpPod.direction
        }

        // clean up this cursed config code later
        val inPerTick =
            HermesConfig.tuningConfig.odometryPodType.inPerTick ?: (actualInchesTravelled / when (podConfig) {
                PinpointEncoderType.PARALLEL -> pinpointParPod.value
                PinpointEncoderType.PERPENDICULAR -> pinpointPerpPod.value
            })

        return ForwardPushPinpointParameters(
            pinpoint.deviceName,
            podConfig,
            podDirection,
            inPerTick,
        )
    }

}

interface LateralPushLocalizerView {

    fun getParameters(actualInchesTravelled: Double): LateralPushLocalizerParameters

}

class LateralPushPinpointView(hardwareMap: HardwareMap) : PinpointView(hardwareMap), LateralPushLocalizerView {

    override fun getParameters(actualInchesTravelled: Double): LateralPushLocalizerParameters {

        if (!(pinpointPerpPod.moved xor pinpointParPod.moved)) {
            throw ConfigurationException("your pinpoint is plugged in wrong silly")
        }

        val podConfig = if (pinpointParPod.moved) PinpointEncoderType.PARALLEL else PinpointEncoderType.PERPENDICULAR

        val podDirection = when (podConfig) {
            PinpointEncoderType.PARALLEL -> pinpointParPod.direction
            PinpointEncoderType.PERPENDICULAR -> pinpointPerpPod.direction
        }

        return LateralPushPinpointParameters(
            podConfig,
            podDirection,
        )

    }

}

/**
 * Relative axial sensor class that measures movement throughout the OpMode.
 * Examples: motor encoder, odometry pod.
 */
open class TuningAxialSensor(val sensorOutput: () -> Double, val movementThreshold: Double = 10000.0) {
    val moved get() = abs(sensorOutput()) > movementThreshold
    val direction get() = if (sensorOutput() > 0) DcMotorSimple.Direction.FORWARD else DcMotorSimple.Direction.REVERSE
    val value get() = sensorOutput()
}

class TuningMotorEncoder(motor: DcMotorEx) : TuningAxialSensor({ motor.currentPosition.toDouble() }, 30.0)


interface AngularPushLocalizerView {

    fun getParameters(actualRevolutions: Double): AngularPushLocalizerParameters

}

// TODO: add validation that the user is actually turning the robot ccw (and the pinpoint IMU is mounted correctly.)
class AngularPushPinpointView(hardwareMap: HardwareMap) : PinpointView(hardwareMap), AngularPushLocalizerView {

    override fun getParameters(actualRevolutions: Double): AngularPushLocalizerParameters {

        val trueParPodType = (HermesConfig["ForwardPushResults"] as ForwardPushPinpointParameters).parEncoder
        val truePerpPodType = (HermesConfig["LateralPushResults"] as LateralPushPinpointParameters).perpEncoder

        // we expect rotating ccw = +x, +y
        // therefore, since L = theta * r, r = L / theta
        val parYTicks = when (trueParPodType) {
            PinpointEncoderType.PARALLEL -> pinpointParPod.value
            PinpointEncoderType.PERPENDICULAR -> pinpointPerpPod.value
        } / (actualRevolutions * 2.0 * PI)

        val perpXTicks = when (truePerpPodType) {
            PinpointEncoderType.PARALLEL -> pinpointParPod.value
            PinpointEncoderType.PERPENDICULAR -> pinpointPerpPod.value
        } / (actualRevolutions * 2.0 * PI)

        return AngularPushPinpointParameters(
            parYTicks,
            perpXTicks,
        )

    }

}

interface DriveView {

    /**
     * Voltage overflows scale both to max voltage.
     * @param rotationVoltage Counter-clockwise (standard).
     */
    fun voltageDrive(forwardVoltage: Double, rotationVoltage: Double)

}

interface HolonomicDriveView : DriveView {

    /**
     * Voltage overflows scale all to max voltage.
     * @param strafeVoltage Positive goes left.
     * @param rotationVoltage Counter-clockwise (standard).
     */
    fun voltageDrive(forwardVoltage: Double, strafeVoltage: Double, rotationVoltage: Double)

}

class MecanumDriveView(parameters: MecanumParameters, hardwareMap: HardwareMap) : HolonomicDriveView {

    private val motors: List<DcMotor> = arrayListOf(
        parameters.leftFront.toDcMotor(hardwareMap),
        parameters.leftBack.toDcMotor(hardwareMap),
        parameters.rightBack.toDcMotor(hardwareMap),
        parameters.rightFront.toDcMotor(hardwareMap),
    )

    /**
     * Voltage overflows scale all to max voltage.
     * @param strafeVoltage Positive goes left.
     * @param rotationVoltage Counter-clockwise (standard).
     */
    override fun voltageDrive(
        forwardVoltage: Double,
        strafeVoltage: Double,
        rotationVoltage: Double,
    ) {
        // normalize by voltage
        val (x, y, r) = arrayListOf(
            forwardVoltage,
            strafeVoltage,
            rotationVoltage,
        ).map { it / VoltageCache.currentVoltage }

        val wheelPowers = MecanumKinematics(1.0).inverse(
            PoseVelocity2dDual(
                Vector2dDual.constant<Time>(Vector2d(x, y), 1),
                DualNum.constant<Time>(rotationVoltage, 1),
            ),
        )

        motors.zip(wheelPowers.all()).forEach { (motor, power) -> motor.power = power.value() }
    }

    /**
     * Voltage overflows scale both to max voltage.
     * @param rotationVoltage Counter-clockwise (standard).
     */
    override fun voltageDrive(forwardVoltage: Double, rotationVoltage: Double) {
        voltageDrive(forwardVoltage, 0.0, rotationVoltage)
    }

}

/**
 * you fucked up your config and now i am mad at you
 */
class ConfigurationException(message: String? = null, cause: Throwable? = null) : Exception(message, cause) {
    constructor(cause: Throwable) : this(null, cause)
}

