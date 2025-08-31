package gay.zharel.hermes.tuning

import gay.zharel.hermes.ftc.Localizer
import gay.zharel.hermes.ftc.OverflowEncoder
import gay.zharel.hermes.ftc.PositionVelocityPair
import gay.zharel.hermes.ftc.RawEncoder
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
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

interface LocalizerTuner {

    fun forwardPushUpdate(actualInchesTravelled: Double) {}

    fun lateralPushUpdate(actualInchesTravelled: Double) {}

    fun angularPushUpdate(actualRevolutions: Double) {}

}

interface LocalizerViewFactory {

    fun make(hardwareMap: HardwareMap): LocalizerTuner

}




