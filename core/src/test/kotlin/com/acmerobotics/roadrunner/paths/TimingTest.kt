package com.acmerobotics.roadrunner.paths

import com.acmerobotics.roadrunner.TEST_ACCEL_CONSTRAINT
import com.acmerobotics.roadrunner.TEST_PROFILE_PARAMS
import com.acmerobotics.roadrunner.TEST_TRAJECTORY_BUILDER_PARAMS
import com.acmerobotics.roadrunner.TEST_VEL_CONSTRAINT
import com.acmerobotics.roadrunner.duration
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.profiles.CancelableProfile
import com.acmerobotics.roadrunner.randomPoint
import com.acmerobotics.roadrunner.trajectories.PositionPathSeqBuilder
import kotlin.system.measureTimeMillis
import kotlin.test.Test
import kotlin.time.measureTimedValue

class TimingTests {
    @Test
    fun `profile timing test`() {
        val posPaths = PositionPathSeqBuilder(
            Vector2d(0.0, 0.0),
            0.0,
            TEST_TRAJECTORY_BUILDER_PARAMS.arcLengthSamplingEps
        ).splineTo(Vector2d(10.0, 10.0), 0.0)
            .splineTo(Vector2d(-10.0, 0.0), Math.PI)
            .splineTo(Vector2d(0.0, 0.0), Math.PI/2)
            .build()

        val posePaths = posPaths.map {
            TangentPath(it, 0.0)
        }

        val times = posePaths.mapIndexed { i, it ->
            i to
            measureTimeMillis {
                CancelableProfile.generate(TEST_PROFILE_PARAMS, it, 0.0, TEST_VEL_CONSTRAINT, TEST_ACCEL_CONSTRAINT)
            }
        }

        times.forEach {
            assert(it.second < 25) { "path ${it.first} took ${it.second} to profile" }
        }
    }

    @Test
    fun `bezier timing test`() {
        val pointList = (0..9).map {
            listOf(randomPoint(), randomPoint(), randomPoint())
        }

        val times = (0..9).map {
            measureTimedValue {
                val bezier = ArclengthReparamCurve2d(
                    BezierCurve2dInternal(pointList[it]),
                    TEST_TRAJECTORY_BUILDER_PARAMS.arcLengthSamplingEps,
                )

                val path = TangentPath(bezier, 0.0)
                val profile = CancelableProfile.generate(TEST_PROFILE_PARAMS, path, 0.0, TEST_VEL_CONSTRAINT, TEST_ACCEL_CONSTRAINT)

                path to profile
            }
        }

        times.forEachIndexed { i, it ->
            println("time $i: ${it.duration.inWholeMilliseconds} ms")
            println("length $i: ${it.value.first.path.length()}")
            assert(it.duration.inWholeMilliseconds < 100)
        }

        val totalTime = times.sumOf { it.duration.inWholeMilliseconds }
        val avgTime = totalTime / times.size.toDouble()

        val totalLen = times.sumOf { it.value.first.path.length() }
        val avgLen = totalLen / times.size.toDouble()

        println("total time $totalTime, avg time $avgTime")
        println("total len $totalLen, avg len $avgLen")
    }

    @Test
    fun `line timing test`() {
        val points = (0..9).map {
            randomPoint() to randomPoint()
        }

        val times = points.map {
            measureTimedValue {
                val path = TangentPath(
                    Line(it.first, it.second),
                    0.0
                )

                val profile = CancelableProfile.generate(TEST_PROFILE_PARAMS, path, 0.0, TEST_VEL_CONSTRAINT, TEST_ACCEL_CONSTRAINT)

                path to profile
            }
        }

        times.forEachIndexed { i, it ->
            println("time $i: ${it.duration.inWholeMilliseconds} ms")
            println("length $i: ${it.value.first.path.length()}")
            println("duration $i: ${it.value.second.duration()}")
            assert(it.duration.inWholeMilliseconds < 100)
        }

        val totalTime = times.sumOf { it.duration.inWholeMilliseconds }
        val avgTime = totalTime / times.size.toDouble()

        val totalLen = times.sumOf { it.value.first.path.length() }
        val avgLen = totalLen / times.size.toDouble()

        val totalDuration = times.sumOf { it.value.second.duration() }
        val avgDuration = totalDuration / times.size.toDouble()

        println("total generation time $totalTime, avg generation time $avgTime")
        println("total len $totalLen, avg len $avgLen")
        println("total duration $totalDuration, avg duration $avgDuration")
    }
}