package gay.zharel.hermes.serialization

import gay.zharel.hermes.*
import gay.zharel.hermes.geometry.range
import gay.zharel.hermes.trajectories.CompositeCancelableTrajectory
import gay.zharel.hermes.trajectories.CompositeTrajectory
import gay.zharel.hermes.trajectories.TrajectoryBuilder
import io.kotest.core.spec.style.FunSpec
import io.kotest.matchers.shouldBe
import kotlinx.serialization.ExperimentalSerializationApi
import kotlinx.serialization.json.decodeFromStream
import kotlinx.serialization.json.encodeToStream
import java.io.File
import kotlin.time.DurationUnit
import kotlin.time.measureTime

val outputFile = File("").resolve("output.json")

@OptIn(ExperimentalSerializationApi::class)
class SerializerTest : FunSpec(
    {
        test("Test Serialization") {
            val traj = TrajectoryBuilder(
                TEST_TRAJECTORY_BUILDER_PARAMS,
                randomPose(),
                0.0,
                TEST_VEL_CONSTRAINT,
                TEST_ACCEL_CONSTRAINT,
            )
                .splineTo(randomPoint(), randomAngle())
                .splineTo(randomPoint(), randomAngle())
                .strafeTo(randomPoint())
                .splineTo(randomPoint(), randomAngle())
                .build()

            HermesJsonFormat.encodeToStream(traj, outputFile.outputStream()) shouldBe Unit
        }

        test("Test Deserialization") {
            val expected = TrajectoryBuilder(
                TEST_TRAJECTORY_BUILDER_PARAMS,
                randomPose(),
                0.0,
                TEST_VEL_CONSTRAINT,
                TEST_ACCEL_CONSTRAINT,
            )
                .splineTo(randomPoint(), randomAngle())
                .splineTo(randomPoint(), randomAngle())
                .strafeTo(randomPoint())
                .splineTo(randomPoint(), randomAngle())
                .buildToComposite()

            HermesJsonFormat.encodeToStream(expected, outputFile.outputStream())

            val actual: CompositeCancelableTrajectory = HermesJsonFormat.decodeFromStream(outputFile.inputStream())

            actual.length() shouldBe expected.length()

            range(0.0, expected.length(), 100).forEach {
                val actPose = actual[it]
                val expPose = expected[it]

                actPose.value() shouldBe expPose.value()
                actPose.velocity().value() shouldBe expPose.velocity().value()
            }
        }

        test("Time Deserialization") {
            val durations = (0..100).map {
                val expected = TrajectoryBuilder(
                        TEST_TRAJECTORY_BUILDER_PARAMS,
                randomPose(),
                0.0,
                TEST_VEL_CONSTRAINT,
                TEST_ACCEL_CONSTRAINT,
                )
                .splineTo(randomPoint(), randomAngle())
                .splineTo(randomPoint(), randomAngle())
                .strafeTo(randomPoint())
                .splineTo(randomPoint(), randomAngle())
                .buildToComposite()

                val encTime = measureTime {
                    HermesJsonFormat.encodeToStream(expected, outputFile.outputStream())
                }

                val decTime = measureTime {
                    HermesJsonFormat.decodeFromStream<CompositeCancelableTrajectory>(outputFile.inputStream())
                }

                encTime to decTime
            }

            println("encoding times ${durations.map { it.first.toDouble(DurationUnit.MILLISECONDS) }}")
            println("decoding times ${durations.map { it.second.toDouble(DurationUnit.MILLISECONDS) }}")
        }
    },
)