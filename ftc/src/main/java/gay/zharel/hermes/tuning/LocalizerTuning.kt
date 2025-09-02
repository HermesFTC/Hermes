package gay.zharel.hermes.tuning

import gay.zharel.hermes.ftc.Localizer
import gay.zharel.hermes.ftc.OverflowEncoder
import gay.zharel.hermes.ftc.PositionVelocityPair
import gay.zharel.hermes.ftc.RawEncoder
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import gay.zharel.hermes.ftc.PinpointLocalizer
import kotlin.math.abs

// ===== Localizer Tuning =====

/**
 * Relative axial sensor class that measures movement throughout the OpMode.
 * Examples: motor encoder, odometry pod.
 */
open class TuningAxialSensor(val sensorOutput: () -> PositionVelocityPair, val movementThreshold: Double = 10000.0) {
    val moved
        get() = abs(sensorOutput().position) > movementThreshold

    val direction
        get() = if (sensorOutput().position > 0) DcMotorSimple.Direction.FORWARD else DcMotorSimple.Direction.REVERSE

    val value
        get() = sensorOutput()
}

class TuningMotorEncoder(encoder: OverflowEncoder) :
    TuningAxialSensor(encoder::getPositionAndVelocity, 30.0) {

    constructor(motor: DcMotorEx) : this(OverflowEncoder(RawEncoder(motor)))
}

interface LocalizerFactory {

    fun make(hardwareMap: HardwareMap): Localizer

}

object TuningLocalizerFactory : LocalizerFactory {
    override fun make(hardwareMap: HardwareMap): Localizer {
        return when (HermesConfig.config.localizer) {
            CustomLocalizer -> error("Custom localizers may not be used with the default tuning factory. Consider implementing your own.")
            DummyParameters -> error("Please select a localizer type before running this opmode!")
            is PinpointParameters -> PinpointLocalizer(
                hardwareMap,
                HermesConfig.config.localizer as PinpointParameters
            )
        }
    }
}

interface LocalizerTuner {

    fun forwardPushUpdate(actualInchesTravelled: Double) {}

    fun lateralPushUpdate(actualInchesTravelled: Double) {}

    fun angularPushUpdate(actualRevolutions: Double) {}

}

interface LocalizerViewFactory {

    fun make(hardwareMap: HardwareMap): LocalizerTuner

}

object TuningLocalizerViewFactory : LocalizerViewFactory {
    override fun make(hardwareMap: HardwareMap): LocalizerTuner {
        return when (HermesConfig.config.localizer) {
            CustomLocalizer -> error("Custom localizer views may not be used with the default tuning factory. Consider implementing your own.")
            DummyParameters -> error("Please select a localizer type before running this opmode!")
            is PinpointParameters -> PinpointTuner(hardwareMap)
        }
    }
}



