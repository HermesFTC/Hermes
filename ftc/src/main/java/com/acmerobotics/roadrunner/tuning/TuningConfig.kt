package com.acmerobotics.roadrunner.tuning

import kotlinx.serialization.Serializable

@Serializable
data class TuningConfig(
    val odometryPodType: OdometryPodType = OdometryPodType.OTHER,
    val forwardPush: ForwardPushConfig = ForwardPushConfig(),
    val lateralPush: LateralPushConfig = LateralPushConfig(),
    val angularPush: AngularPushConfig = AngularPushConfig()
)

enum class OdometryPodType {
    GOBILDA_4_BAR,
    GOBILDA_SWINGARM,
    OTHER
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