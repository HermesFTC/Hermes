/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.ftc

import gay.zharel.hermes.serialization.HermesJsonFormat
import gay.zharel.hermes.tuning.MecanumParameters
import gay.zharel.hermes.tuning.MotorConfig
import gay.zharel.hermes.tuning.MutableSignal
import gay.zharel.hermes.tuning.QuasistaticParameters
import gay.zharel.hermes.tuning.RobotConfig
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

        val params = QuasistaticParameters(
            xs,
            ys
        )

        ObjectMapper(JsonFactory())
            .writerWithDefaultPrettyPrinter()
            .writeValue(outputFile, params)
    }

}