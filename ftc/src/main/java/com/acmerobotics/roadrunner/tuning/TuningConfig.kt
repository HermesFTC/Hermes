package com.acmerobotics.roadrunner.tuning

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

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

@Serializable
data class TuningConfig(
    var odometryPodType: OdometryPodType = OdometryPodType.OTHER,
    var localizer: LocalizerType = LocalizerType.CUSTOM,
    var drive: DriveType = DriveType.CUSTOM,
    var warnings: List<String> = arrayListOf(),
    val forwardPush: ForwardPushConfig = ForwardPushConfig(),
    val lateralPush: LateralPushConfig = LateralPushConfig(),
    val angularPush: AngularPushConfig = AngularPushConfig(),
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
data class ForwardRampConfig(
    var thresholdInchesPerSecond: Double = 0.1,
    var voltagePerSecond: Double = 1.0,
    var direction: Double = 1.0
)

@Serializable
data class ForwardStepConfig(
    var voltageStep: Double = 10.0,
    var direction: Double = 1.0
)

@Serializable
data class AngularRampConfig(
    var thresholdRadiansPerSecond: Double = 0.1,
    var voltagePerSecond: Double = 1.0,
    var direction: Double = 1.0
)

@Serializable
data class AngularStepConfig(
    var voltageStep: Double = 10.0,
    var direction: Double = 1.0
)