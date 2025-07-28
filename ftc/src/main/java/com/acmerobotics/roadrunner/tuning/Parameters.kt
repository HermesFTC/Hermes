package com.acmerobotics.roadrunner.tuning

import com.acmerobotics.roadrunner.control.MotorFeedforward
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlinx.serialization.Serializable

@Serializable
data class RobotConfig(
    val drive: DriveParameters,
    val feedforward: FeedforwardParameters
)

sealed interface DriveParameters

@Serializable
data class MotorConfig(
    @JvmField var name: String,
    @JvmField var direction: DcMotorSimple.Direction
)

@Serializable
data class MecanumParameters(
    var trackWidth: Double = 0.0,
    var wheelBase: Double = 0.0,
    var leftFront: MotorConfig,
    var leftBack: MotorConfig,
    var rightFront: MotorConfig,
    var rightBack: MotorConfig
) : DriveParameters

@Serializable
data class FeedforwardParameters(
    var kStatic: Double = 0.0,
    val kV: Double = 0.0,
    val kA: Double = 0.0
) {
    operator fun invoke() = MotorFeedforward(kStatic, kV, kA)
}