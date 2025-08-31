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
    var drive: DriveParameters = DummyParameters,
    var feedforward: DriveFeedforwardParameters = DummyParameters,
    var localizer: LocalizerParameters = DummyParameters
)

@Serializable
@SerialName("DummyParameters")
object DummyParameters : LocalizerParameters, DriveParameters, DriveFeedforwardParameters {
    override var translational: FeedforwardParameters = error("no")
    override var rotational: FeedforwardParameters = error("no")
}

sealed interface DriveParameters

@Serializable
data class MotorConfig(
    @JvmField var name: String = "",
    @JvmField var direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
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
@SerialName("CustomDrive")
object CustomDrive : DriveParameters

@Serializable
@SerialName("MecanumParameters")
data class MecanumParameters(
    var trackWidth: Double = 0.0,
    var wheelBase: Double = 0.0,
    var leftFront: MotorConfig = MotorConfig(),
    var leftBack: MotorConfig = MotorConfig(),
    var rightFront: MotorConfig = MotorConfig(),
    var rightBack: MotorConfig = MotorConfig()
) : DriveParameters

@Serializable
@SerialName("TankParameters")
data class TankParameters(
    var trackWidth: Double = 0.0,
    var wheelBase: Double = 0.0,
    var leftMotors: MutableList<MotorConfig> = arrayListOf(MotorConfig(), MotorConfig()),
    var rightMotors: MutableList<MotorConfig> = arrayListOf(MotorConfig(), MotorConfig()),
)

sealed interface DriveFeedforwardParameters {
    var translational: FeedforwardParameters
    var rotational: FeedforwardParameters
}

@Serializable
@SerialName("CustomDriveFeedforward")
object CustomDriveFeedforward : DriveFeedforwardParameters {
    override var translational: FeedforwardParameters = error("tuning opmodes do not support custom feedforward")
    override var rotational: FeedforwardParameters = error("tuning opmodes do not support custom feedforward")
}

@Serializable
@SerialName("DriveFeedforward")
data class DriveFeedforward(
    override var translational: FeedforwardParameters = FeedforwardParameters(),
    override var rotational: FeedforwardParameters = FeedforwardParameters(),
) : DriveFeedforwardParameters

@Serializable
@SerialName("HolonomicDriveFeedforward")
data class HolonomicDriveFeedforward(
    var axial: FeedforwardParameters = FeedforwardParameters(),
    var lateral: FeedforwardParameters = FeedforwardParameters(),
    override var rotational: FeedforwardParameters = FeedforwardParameters(),
) : DriveFeedforwardParameters {
    override var translational by this::axial
}

@Serializable
data class FeedforwardParameters(
    var kStatic: Double = 0.0,
    var kV: Double = 0.0,
    var kA: Double = 0.0
) {
    operator fun invoke() = MotorFeedforward(kStatic, kV, kA)
}

sealed interface LocalizerParameters

@Serializable
@SerialName("CustomLocalizer")
object CustomLocalizer : LocalizerParameters

@Serializable
@SerialName("PinpointParameters")
data class PinpointParameters(
    var inPerTick: Double = 1.0,
    var name: String = "pinpoint",
    var parYTicks: Double = 0.0,
    var perpXTicks: Double = 0.0,
    var parDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    var perpDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
) : LocalizerParameters

enum class PinpointEncoderType {
    @SerialName("Parallel")
    PARALLEL,

    @SerialName("Perpendicular")
    PERPENDICULAR
}

@Serializable
data class VoltageConfig(
    val nominalVoltage: Double = 12.5,
    val readIntervalSeconds: Double = 0.5,
)