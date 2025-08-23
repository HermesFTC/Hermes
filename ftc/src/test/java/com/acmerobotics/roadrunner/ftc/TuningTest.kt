package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.serialization.HermesJsonFormat
import com.acmerobotics.roadrunner.tuning.MecanumParameters
import com.acmerobotics.roadrunner.tuning.MotorConfig
import com.acmerobotics.roadrunner.tuning.MutableSignal
import com.acmerobotics.roadrunner.tuning.RegressionParameters
import com.acmerobotics.roadrunner.tuning.RobotConfig
import com.fasterxml.jackson.core.JsonFactory
import com.fasterxml.jackson.databind.ObjectMapper
import com.qualcomm.robotcore.hardware.DcMotorSimple
import io.kotest.core.spec.style.FunSpec
import kotlinx.serialization.ExperimentalSerializationApi
import kotlinx.serialization.json.encodeToStream
import kotlin.test.Test
import java.io.File

val outputFile = File("").resolve("output.json")

@OptIn(ExperimentalSerializationApi::class)
class TuningTest {

    @Test
    fun `Tuning Serialization Test`() {
        // populate with random data
        val xs = MutableSignal()
        val ys = MutableSignal()

        val DELTA_TIME = 0.001
        var time = 0.001

        for (i in 1..100) {
            xs.times.add(time)
            xs.values.add(Math.random())

            ys.times.add(time)
            ys.values.add(Math.random())

            time += DELTA_TIME
        }

        val params = RegressionParameters(
            xs,
            ys
        )

        ObjectMapper(JsonFactory())
            .writerWithDefaultPrettyPrinter()
            .writeValue(outputFile, params)
    }

}