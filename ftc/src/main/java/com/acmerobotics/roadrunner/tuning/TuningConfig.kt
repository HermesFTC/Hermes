package com.acmerobotics.roadrunner.tuning

import kotlinx.serialization.Serializable

@Serializable
data class TuningConfig(
    var odometryPodType: OdometryPodType = OdometryPodType.OTHER,
    var localizer: LocalizerType = LocalizerType.CUSTOM,
    var warnings: List<String> = arrayListOf(),
    val forwardPush: ForwardPushConfig = ForwardPushConfig(),
    val lateralPush: LateralPushConfig = LateralPushConfig(),
    val angularPush: AngularPushConfig = AngularPushConfig(),
    val forwardRamp: ForwardRampConfig = ForwardRampConfig(),
    val forwardStep: ForwardStepConfig = ForwardStepConfig(),
    val angularRamp: AngularRampConfig = AngularRampConfig(),
    val angularStep: AngularStepConfig = AngularStepConfig(),
)

enum class OdometryPodType {
    GOBILDA_4_BAR,
    GOBILDA_SWINGARM,
    OTHER
}

enum class LocalizerType {
    CUSTOM,
    TWO_WHEEL,
    THREE_WHEEL,
    GOBILDA_PINPOINT,
    SPARKFUN_OTOS,
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
    var voltagePerSecond: Double = 1.0
)

@Serializable
data class ForwardStepConfig(
    var voltageStep: Double = 10.0
)

@Serializable
data class AngularRampConfig(
    var thresholdRadiansPerSecond: Double = 0.1,
    var voltagePerSecond: Double = 1.0
)

@Serializable
data class AngularStepConfig(
    var voltageStep: Double = 10.0
)