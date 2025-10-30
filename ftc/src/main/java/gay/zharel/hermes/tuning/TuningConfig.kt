/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.tuning

import android.content.Context
import com.qualcomm.ftccommon.FtcEventLoop
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes

object TuningConfigListener : PersistentConfig.Subscriber {

    private var lastLocalizer = HermesConfig.tuningConfig.localizer

    init {
        HermesConfig.addSubscriber(this)
    }

    override fun onUpdate(configVariables: MutableMap<String, Any?>) {

        // if the localizer is now different
        if (HermesConfig.tuningConfig.localizer != lastLocalizer) {
            when (HermesConfig.tuningConfig.localizer) {
                LocalizerType.CUSTOM -> HermesConfig.config.localizer = DummyParameters
                LocalizerType.TWO_WHEEL -> HermesConfig.config.localizer = DummyParameters
                LocalizerType.THREE_WHEEL -> HermesConfig.config.localizer = DummyParameters
                LocalizerType.GOBILDA_PINPOINT -> HermesConfig.config.localizer =
                    PinpointParameters()

                LocalizerType.SPARKFUN_OTOS -> HermesConfig.config.localizer = DummyParameters
                LocalizerType.MOTOR_ENCODERS -> HermesConfig.config.localizer = DummyParameters
            }
            lastLocalizer = HermesConfig.tuningConfig.localizer
        }
    }

}

object HardwareMapCache {
    private lateinit var opModeManager: OpModeManagerImpl

    val hardwareMap: HardwareMap get() = opModeManager.hardwareMap ?: HardwareMap(null, null) // pls don't try to use hwmap b4 it works

    @OnCreateEventLoop
    @JvmStatic
    fun register(context: Context, eventLoop: FtcEventLoop) {
        opModeManager = eventLoop.opModeManager
    }

}

@Serializable
data class TuningConfig(
    var odometryPodType: OdometryPodType = OdometryPodType.OTHER,
    var localizer: LocalizerType = LocalizerType.CUSTOM,
    var drive: DriveType = DriveType.CUSTOM,
    var warnings: List<String> = arrayListOf(),
    val forwardPush: ForwardPushConfig = ForwardPushConfig(),
    val lateralPush: LateralPushConfig = LateralPushConfig(),
    val angularPush: AngularPushConfig = AngularPushConfig(),
    val drivetrainConfig: DrivetrainConfigurationConfig = DrivetrainConfigurationConfig(),
    val forwardRamp: ForwardRampConfig = ForwardRampConfig(),
    val forwardStep: ForwardStepConfig = ForwardStepConfig(),
    val angularRamp: AngularRampConfig = AngularRampConfig(),
    val angularStep: AngularStepConfig = AngularStepConfig(),
) {

    init {
        TuningConfigListener // start tuning listener as soon as we instantiate a config
    }

}

enum class DriveType {
    @SerialName("Mecanum")
    MECANUM,

    @SerialName("Tank")
    TANK,

    @SerialName("Custom")
    CUSTOM
}


enum class OdometryPodType {
    @SerialName("goBILDA 4 Bar")
    GOBILDA_4_BAR,

    @SerialName("goBILDA Swingarm")
    GOBILDA_SWINGARM,

    @SerialName("Other")
    OTHER
}

enum class LocalizerType {
    @SerialName("Custom")
    CUSTOM,

    @SerialName("Two Wheel")
    TWO_WHEEL,

    @SerialName("Three Wheel")
    THREE_WHEEL,

    @SerialName("goBILDA Pinpoint")
    GOBILDA_PINPOINT,

    @SerialName("SparkFUN OTOS")
    SPARKFUN_OTOS,

    @SerialName("Motor Encoders")
    MOTOR_ENCODERS
}

/**
 * Returns the defined `inPerTick` constant, or `null` if none exists.
 */
val OdometryPodType.inPerTick: Double? get() = when(this) {
    OdometryPodType.GOBILDA_4_BAR -> 0.001978956 // numbers from goBILDA pinpoint driver
    OdometryPodType.GOBILDA_SWINGARM -> 0.002968434 // numbers from goBILDA pinpoint driver
    OdometryPodType.OTHER -> null
}

@Serializable
data class ForwardPushConfig(
    var actualInchesTravelled: Double = 24.0,
)

@Serializable
data class LateralPushConfig(
    var actualInchesTravelled: Double = 24.0,
)

@Serializable
data class AngularPushConfig(
    var actualRevolutions: Double = 1.0,
)

@Serializable
data class DrivetrainConfigurationConfig(
    var motorNames: MutableList<String> = arrayListOf(),
    var chosenMotors: MutableList<String> = arrayListOf("", "", "", "", "", "", "", "",)
)

@Serializable
data class ForwardRampConfig(
    var thresholdInchesPerSecond: Double = 0.1,
    var voltagePerSecond: Double = 1.0,
    var direction: Double = 1.0,
    var regressionParams: QuasistaticParameters = QuasistaticParameters()
)

@Serializable
data class ForwardStepConfig(
    var voltageStep: Double = 10.0,
    var direction: Double = 1.0,
    var regressionParams: DynamicParameters = DynamicParameters()
)

@Serializable
data class AngularRampConfig(
    var thresholdRadiansPerSecond: Double = 0.1,
    var voltagePerSecond: Double = 1.0,
    var direction: Double = 1.0,
    var regressionParams: QuasistaticParameters = QuasistaticParameters()
)

@Serializable
data class AngularStepConfig(
    var voltageStep: Double = 10.0,
    var direction: Double = 1.0,
    var regressionParams: DynamicParameters = DynamicParameters()
)