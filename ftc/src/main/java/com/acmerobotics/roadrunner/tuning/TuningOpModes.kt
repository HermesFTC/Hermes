package com.acmerobotics.roadrunner.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.ValueProvider
import com.acmerobotics.dashboard.config.reflection.FieldProvider
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareMap

class ForwardPushTest() : OpMode() {

    val localizerView: ForwardPushLocalizerView by lazy { generateLocalizerView(hardwareMap) }

    val actualInches: Double get() = HermesConfig.tuningConfig.forwardPush.actualInchesTravelled

    override fun init() {
        localizerView
    }

    override fun loop() {

    }

    override fun stop() {
        val params = localizerView.getParameters(actualInches)
        HermesConfig.addConfigVariable("ForwardPushResults", params)
    }

    companion object {
        fun generateLocalizerView(hardwareMap: HardwareMap): ForwardPushLocalizerView = ForwardPushPinpointView(hardwareMap)
    }

}