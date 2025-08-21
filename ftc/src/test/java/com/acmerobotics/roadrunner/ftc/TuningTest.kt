package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.serialization.HermesJsonFormat
import com.acmerobotics.roadrunner.tuning.MecanumParameters
import com.acmerobotics.roadrunner.tuning.MotorConfig
import com.acmerobotics.roadrunner.tuning.RobotConfig
import com.qualcomm.robotcore.hardware.DcMotorSimple
import io.kotest.core.spec.style.FunSpec
import kotlinx.serialization.ExperimentalSerializationApi
import kotlinx.serialization.json.encodeToStream
import org.junit.jupiter.api.Test
import java.io.File

val outputFile = File("").resolve("output.json")

@OptIn(ExperimentalSerializationApi::class)
class TuningTest: FunSpec({

    test("Tuning Serialization Test") {
        HermesJsonFormat.encodeToStream(
            MecanumParameters(
                10.0,
                10.0,
                MotorConfig("hi", DcMotorSimple.Direction.FORWARD),
                MotorConfig("hi2", DcMotorSimple.Direction.FORWARD),
                MotorConfig("hi3", DcMotorSimple.Direction.FORWARD),
                MotorConfig("hi4", DcMotorSimple.Direction.FORWARD),
            ),
            outputFile.outputStream()
        )
    }

})