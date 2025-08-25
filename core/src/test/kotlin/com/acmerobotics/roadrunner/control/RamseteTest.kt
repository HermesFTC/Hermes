package com.acmerobotics.roadrunner.control

import com.acmerobotics.roadrunner.TEST_PROFILE_PARAMS
import com.acmerobotics.roadrunner.geometry.*
import com.acmerobotics.roadrunner.paths.TangentPath
import com.acmerobotics.roadrunner.profiles.CancelableProfile
import com.acmerobotics.roadrunner.profiles.ProfileAccelConstraint
import com.acmerobotics.roadrunner.profiles.TimeProfile
import com.acmerobotics.roadrunner.saveChart
import com.acmerobotics.roadrunner.trajectories.PositionPathSeqBuilder
import org.junit.jupiter.api.Test
import org.knowm.xchart.XYChart
import org.knowm.xchart.style.markers.None
import org.knowm.xchart.style.theme.MatlabTheme
import kotlin.math.PI
import kotlin.test.assertEquals

class RamseteTest {
    @Test
    fun testRamsete() {
        testRamseteHelper(false)
        testRamseteHelper(true)
    }

    fun testRamseteHelper(reversed: Boolean) {
        val path = TangentPath(
            PositionPathSeqBuilder(
                Vector2d(0.0, 0.0),
                0.0,
                1e-6
            )
                .splineTo(
                    Vector2d(30.0, 30.0),
                    PI / 4
                )
                .lineToY(60.0)
                .splineTo(
                    Vector2d(60.0, 90.0),
                    PI,
                )
                .build()
                .first(),
            if (reversed) PI else 0.0
        )

        val trackWidth = 15.0
        val kinematics = TankKinematics(trackWidth)

        val profile = TimeProfile(CancelableProfile.generate(
            TEST_PROFILE_PARAMS,
            path, 0.0,
            WheelVelConstraint(kinematics, 10.0),
            ProfileAccelConstraint(-20.0, 20.0),
        ).baseProfile)

        val controller = RamseteController(trackWidth, bBar = 2.0)

        // var pose = (
        //     Vector2(-1.0, -1.0),
        //     PI / 8
        // )

        var pose = Pose2d(
            Vector2d(-5.0, -10.0),
            if (reversed) PI else 0.0
        )

        val targetPoses = mutableListOf<Pose2d>()
        val poses = mutableListOf(pose)

        val dt = 0.01
        var t = 0.0
        while (t < profile.duration) {
            val s = profile[t]

            val targetPose = path[s.value(), 3]

            val command = controller.compute(s, targetPose, pose).value()

            pose +=
                Twist2d(
                    command.linearVel * dt,
                    command.angVel * dt
                )

            targetPoses.add(targetPose.value())
            poses.add(pose)

            t += dt
        }

        val graph = XYChart(600, 400)
        graph.title = "Ramsete Follower Sim"
        graph.addSeries(
            "Target Trajectory",
            targetPoses.map { it.position.x }.toDoubleArray(),
            targetPoses.map { it.position.y }.toDoubleArray()
        )
        graph.addSeries(
            "Actual Trajectory",
            poses.map { it.position.x }.toDoubleArray(),
            poses.map { it.position.y }.toDoubleArray()
        )
        graph.seriesMap.values.forEach { it.marker = None() }
        graph.styler.theme = MatlabTheme()
        saveChart("ramseteFollower${if (reversed) "Reversed" else ""}", graph)
    }

    @Test
    fun testNewRamseteForward() {
        testNewRamseteHelper(false)
    }

    @Test
    fun testNewRamseteReversed() {
        return testNewRamseteHelper(true)
    }

    fun testNewRamseteHelper(reversed: Boolean) {
        val path = TangentPath(
            PositionPathSeqBuilder(
                Vector2d(0.0, 0.0),
                0.0,
                1e-6
            )
                .splineTo(
                    Vector2d(30.0, 30.0),
                    PI / 4
                )
                .lineToY(60.0)
                .splineTo(
                    Vector2d(60.0, 90.0),
                    PI,
                )
                .build()
                .first(),
            if (reversed) PI else 0.0
        )

        val trackWidth = 15.0
        val kinematics = TankKinematics(trackWidth)

        val profile = TimeProfile(
            CancelableProfile.generate(
                TEST_PROFILE_PARAMS,
                path, 0.0,
                WheelVelConstraint(kinematics, 10.0),
                ProfileAccelConstraint(-20.0, 20.0),
            ).baseProfile
        )

        val controller = RamseteController(trackWidth, bBar = 2.0)

        // var pose = (
        //     Vector2(-1.0, -1.0),
        //     PI / 8
        // )

        var pose = Pose2d(
            Vector2d(-5.0, -10.0),
            if (reversed) PI else 0.0
        )


        val targetPoses = mutableListOf<Pose2d>()
        val poses = mutableListOf(pose)

        val dt = 0.01
        var t = 0.0
        while (t < profile.duration) {
            val s = profile[t]

            val targetPose = path[s.value(), 3]

            val oldCommand: PoseVelocity2dDual<Time> = controller.computeOld(s, targetPose, pose)
            val newCommand = controller.compute(s, targetPose, pose)

            val command = oldCommand.value()

            pose +=
                Twist2d(
                    command.linearVel * dt,
                    command.angVel * dt
                )

            targetPoses.add(targetPose.value())
            poses.add(pose)

            t += dt

            assertEqualsDoubleList(oldCommand.flatten(), newCommand.flatten())
        }
    }
}

fun <Param : DualParameter> PoseVelocity2dDual<Param>.flatten() : List<Double> {
    return linearVel.x.values() + linearVel.y.values() + angVel.values()
}

fun assertEqualsDoubleList(a: List<Double>, b: List<Double>, tolerance: Double = 1e-5) {
    assertEquals(a.size, b.size)
    for (i in a.indices) {
        assertEquals(a[i], b[i], tolerance)
    }
}
