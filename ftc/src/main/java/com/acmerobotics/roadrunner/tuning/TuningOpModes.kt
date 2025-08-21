package com.acmerobotics.roadrunner.tuning

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareMap

class ForwardPushTest(val localizerView: ForwardPushLocalizerView) : OpMode() {

    val actualInches: Double get() = HermesConfig.tuningConfig.forwardPush.actualInchesTravelled

    override fun init() {

    }

    override fun loop() {

    }

    override fun stop() {
        val params = localizerView.getParameters(actualInches)
        HermesConfig.addConfigVariable("ForwardPushResults", params)
        HermesConfig.config.localizer = HermesConfig.config.localizer?.assembleIfAble()
    }

}

class LateralPushTest(val localizerView: LateralPushLocalizerView) : OpMode() {

    val actualInches: Double get() = HermesConfig.tuningConfig.lateralPush.actualInchesTravelled

    override fun init() {

    }

    override fun loop() {

    }

    override fun stop() {
        val params = localizerView.getParameters(actualInches)
        HermesConfig.addConfigVariable("LateralPushResults", params)
        HermesConfig.config.localizer = HermesConfig.config.localizer?.assembleIfAble()
    }

}

class AngularPushTest(val localizerView: AngularPushLocalizerView) : OpMode() {

    val actualRevolutions: Double get() = HermesConfig.tuningConfig.angularPush.actualRevolutions

    override fun init() {

    }

    override fun loop() {

    }

    override fun stop() {
        val params = localizerView.getParameters(actualRevolutions)
        HermesConfig.addConfigVariable("AngularPushResults", params)
        HermesConfig.config.localizer = HermesConfig.config.localizer?.assembleIfAble()
    }

}