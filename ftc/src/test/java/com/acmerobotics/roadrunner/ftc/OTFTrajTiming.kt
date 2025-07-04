package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.randomAngle
import com.acmerobotics.roadrunner.randomPoint
import com.acmerobotics.roadrunner.randomPose
import com.acmerobotics.roadrunner.repr
import io.mockk.every
import kotlin.system.measureTimeMillis
import kotlin.test.Test
import kotlin.test.assertEquals

class OTFTrajTiming {
    @Test
    fun `test test drive`() {
        assertEquals(Pose2d.zero, TEST_DRIVE.localizer.pose)
    }

    @Test
    fun `test lineTo`() {
        val times = (1..100).map {
            measureTimeMillis {
                val drive = TEST_DRIVE
                every { drive.localizer.pose } returns randomPose()

                drive.lineTo(randomPoint()).generate()
            }
        }

        println(times)
    }

    @Test
    fun `time splineTo`() {
        val times = (1..100).map {
            measureTimeMillis {
                val drive = TEST_DRIVE
                every { drive.localizer.pose } returns randomPose()

                drive.splineTo(randomPoint(), randomAngle())
            }
        }

        println(times)
    }
}