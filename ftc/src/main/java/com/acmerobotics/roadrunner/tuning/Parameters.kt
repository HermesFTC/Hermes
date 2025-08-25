package com.acmerobotics.roadrunner.tuning

import com.acmerobotics.roadrunner.control.MotorFeedforward
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

@Serializable
data class RobotConfig(
    var drive: DriveParameters?,
    var feedforward: FeedforwardParameters?,
    var localizer: LocalizerParameters?
)

sealed interface DriveParameters

@Serializable
data class MotorConfig(
    @JvmField var name: String,
    @JvmField var direction: DcMotorSimple.Direction
) {
    fun toDcMotor(hardwareMap: HardwareMap): DcMotor {
        val motor = hardwareMap.dcMotor.get(name)
        motor.direction = DcMotorSimple.Direction.REVERSE
        return motor
    }

    fun toDcMotorEx(hardwareMap: HardwareMap): DcMotorEx {
        return toDcMotor(hardwareMap) as DcMotorEx
    }
}

@Serializable
@SerialName("MecanumParameters")
data class MecanumParameters(
    var trackWidth: Double = 0.0,
    var wheelBase: Double = 0.0,
    var leftFront: MotorConfig,
    var leftBack: MotorConfig,
    var rightFront: MotorConfig,
    var rightBack: MotorConfig
) : DriveParameters

@Serializable
@SerialName("TankParameters")
data class TankParameters(
    var trackWidth: Double = 0.0,
    var wheelBase: Double = 0.0,
    var leftMotors: MutableList<MotorConfig>,
    var rightMotors: MutableList<MotorConfig>
)

@Serializable
data class DriveFeedforward(
    var translational: FeedforwardParameters,
    var rotational: FeedforwardParameters,
)

@Serializable
data class FeedforwardParameters(
    var kStatic: Double = 0.0,
    var kV: Double = 0.0,
    var kA: Double = 0.0
) {
    operator fun invoke() = MotorFeedforward(kStatic, kV, kA)
}

sealed interface LocalizerParameters {

    /**
     * Assemble the localizer from known tuning constants.
     */
    fun assembleIfAble(): LocalizerParameters

}

@Serializable
@SerialName("PinpointParameters")
data class PinpointParameters(
    val inPerTick: Double,
    val name: String = "pinpoint",
    val parYTicks: Double = 0.0,
    val perpXTicks: Double = 0.0,
    val parDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val perpDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
) : LocalizerParameters {

    override fun assembleIfAble(): LocalizerParameters {
        try {

            val forwardPush = HermesConfig["ForwardPushResults"] as ForwardPushPinpointParameters
            val lateralPush = HermesConfig["LateralPushResults"] as LateralPushPinpointParameters
            val angularPush = HermesConfig["AngularPushResults"] as AngularPushPinpointParameters

            return PinpointParameters(
                forwardPush.inPerTick,
                forwardPush.pinpointName,
                angularPush.parYTicks,
                angularPush.perpXTicks,
                forwardPush.parEncoderDirection,
                lateralPush.perpEncoderDirection
            )

        } catch (_: Exception) {
            return this
        }
    }

}

sealed interface ForwardPushLocalizerParameters

@Serializable
@SerialName("ForwardPushPinpointParameters")
data class ForwardPushPinpointParameters(
    val pinpointName: String,
    val parEncoder: PinpointEncoderType,
    val parEncoderDirection: DcMotorSimple.Direction,
    val inPerTick: Double,
): ForwardPushLocalizerParameters

enum class PinpointEncoderType {
    PARALLEL,
    PERPENDICULAR
}

sealed interface LateralPushLocalizerParameters

@Serializable
@SerialName("LateralPushPinpointParameters")
data class LateralPushPinpointParameters(
    val perpEncoder: PinpointEncoderType,
    val perpEncoderDirection: DcMotorSimple.Direction,
): LateralPushLocalizerParameters

sealed interface AngularPushLocalizerParameters

@Serializable
@SerialName("AngularPushPinpointParameters")
data class AngularPushPinpointParameters(
    val parYTicks: Double,
    val perpXTicks: Double
): AngularPushLocalizerParameters

@Serializable
data class VoltageConfig(
    val nominalVoltage: Double = 12.5,
    val readIntervalSeconds: Double = 0.5,
)