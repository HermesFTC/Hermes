package com.acmerobotics.roadrunner.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.ValueProvider
import com.acmerobotics.dashboard.config.reflection.FieldProvider
import com.qualcomm.robotcore.eventloop.opmode.OpMode

class ForwardPushTest() : OpMode() {

    override fun init() {
        FtcDashboard.getInstance().addConfigVariable(
            "ForwardPush", "Actual Inches",
            FieldProvider<Double>(
                Companion.javaClass.getField("actualInchesTravelled"), // CURSED SHIT
                Companion,
            ),
        )
    }

    override fun loop() {}

    override fun stop() {

    }

    companion object {
        var actualInchesTravelled = 24.0
    }

}