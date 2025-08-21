package com.acmerobotics.roadrunner.tuning

import kotlinx.serialization.Serializable

@Serializable
data class TuningConfig(
    val forwardPush: ForwardPushConfig = ForwardPushConfig(),
    val lateralPush: LateralPushConfig = LateralPushConfig(),
    val angularPush: AngularPushConfig = AngularPushConfig()
)

@Serializable
data class ForwardPushConfig(
    var actualInchesTravelled: Double = 24.0
)

@Serializable
data class LateralPushConfig(
    var actualInchesTravelled: Double = 24.0
)

@Serializable
data class AngularPushConfig(
    var actualRevolutions: Double = 1.0
)