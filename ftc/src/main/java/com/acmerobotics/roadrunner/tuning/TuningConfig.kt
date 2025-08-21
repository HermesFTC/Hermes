package com.acmerobotics.roadrunner.tuning

import kotlinx.serialization.Serializable

@Serializable
data class TuningConfig(
    val forwardPush: ForwardPushConfig,
    val lateralPush: LateralPushConfig
)

@Serializable
data class ForwardPushConfig(
    var actualInchesTravelled: Double
)

@Serializable
data class LateralPushConfig(
    var actualInchesTravelled: Double
)