/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.tuning

import gay.zharel.hermes.ftc.PositionVelocityPair
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import gay.zharel.fateweaver.flight.FlightRecorder
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.PI

class PinpointTuner(hardwareMap: HardwareMap) : LocalizerTuner {
    val pinpoint: GoBildaPinpointDriver =
        hardwareMap.getAll(GoBildaPinpointDriver::class.java).iterator().next()
            ?: throw ConfigurationException("No pinpoints detected in configuration.")

    val pinpointParPod = TuningAxialSensor(
        { PositionVelocityPair(pinpoint.encoderX.toDouble(), pinpoint.getVelX(DistanceUnit.INCH) / config.inPerTick) }
    )
    val pinpointPerpPod = TuningAxialSensor(
        { PositionVelocityPair(pinpoint.encoderY.toDouble(), pinpoint.getVelY(DistanceUnit.INCH) / config.inPerTick) }
    )

    val config: PinpointParameters get() = HermesConfig.config.localizer as PinpointParameters

    // safety checks
    init {
        val pinpoints = hardwareMap.getAll(GoBildaPinpointDriver::class.java)
        if (pinpoints.size > 1) {
            FlightRecorder.write(
                "PINPOINT_CONFIGURATION_WARNING",
                "Multiple pinpoints detected. Using first: " + pinpoints.iterator().next().deviceName,
            )
        } else if (pinpoints.isEmpty()) {
            throw ConfigurationException("No pinpoints detected in configuration.")
        }
    }

    override fun forwardPushUpdate(actualInchesTravelled: Double) {
        if (!(pinpointPerpPod.moved xor pinpointParPod.moved)) {
            throw ConfigurationException("your pinpoint is plugged in wrong silly")
        }

        // can take this out later but for now catch this here
        if (pinpointPerpPod.moved) {
            throw ConfigurationException("your pinpoint pods are swapped silly, swap the wires")
        }

        val podConfig = if (pinpointParPod.moved) PinpointEncoderType.PARALLEL else PinpointEncoderType.PERPENDICULAR

        config.name = pinpoint.deviceName

        config.parDirection = when (podConfig) {
            PinpointEncoderType.PARALLEL -> pinpointParPod.direction
            PinpointEncoderType.PERPENDICULAR -> pinpointPerpPod.direction
        }

        // clean up this cursed config code later
        config.inPerTick =
            HermesConfig.tuningConfig.odometryPodType.inPerTick ?: (actualInchesTravelled / when (podConfig) {
                PinpointEncoderType.PARALLEL -> pinpointParPod.value.position
                PinpointEncoderType.PERPENDICULAR -> pinpointPerpPod.value.position
            })
    }

    override fun lateralPushUpdate(actualInchesTravelled: Double) {

        if (!(pinpointPerpPod.moved xor pinpointParPod.moved)) {
            throw ConfigurationException("your pinpoint is plugged in wrong silly")
        }

        val podConfig = if (pinpointParPod.moved) PinpointEncoderType.PARALLEL else PinpointEncoderType.PERPENDICULAR

        config.perpDirection = when (podConfig) {
            PinpointEncoderType.PARALLEL -> pinpointParPod.direction
            PinpointEncoderType.PERPENDICULAR -> pinpointPerpPod.direction
        }
    }

    // TODO: add validation that the user is actually turning the robot ccw (and the pinpoint IMU is mounted correctly.)
    override fun angularPushUpdate(actualRevolutions: Double) {

        // leaving this in here so that i can make swapped-pod tuning work later
        val trueParPodType = PinpointEncoderType.PARALLEL
        val truePerpPodType = PinpointEncoderType.PERPENDICULAR

        // we expect rotating ccw = +x, +y
        // therefore, since L = theta * r, r = L / theta
        config.parYTicks = when (trueParPodType) {
            PinpointEncoderType.PARALLEL -> pinpointParPod.value
            PinpointEncoderType.PERPENDICULAR -> pinpointPerpPod.value
        }.position / (actualRevolutions * 2.0 * PI)

        config.perpXTicks = when (truePerpPodType) {
            PinpointEncoderType.PARALLEL -> pinpointParPod.value
            PinpointEncoderType.PERPENDICULAR -> pinpointPerpPod.value
        }.position / (actualRevolutions * 2.0 * PI)
    }
}