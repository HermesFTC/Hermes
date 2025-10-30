/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.control

import gay.zharel.hermes.TEST_PROFILE_PARAMS
import gay.zharel.hermes.geometry.*
import gay.zharel.hermes.math.Arclength
import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.math.DualParameter
import gay.zharel.hermes.math.Time
import gay.zharel.hermes.math.sinc
import gay.zharel.hermes.paths.TangentPath
import gay.zharel.hermes.profiles.CancelableProfile
import gay.zharel.hermes.profiles.ProfileAccelConstraint
import gay.zharel.hermes.profiles.TimeProfile
import gay.zharel.hermes.saveChart
import gay.zharel.hermes.trajectories.PositionPathSeqBuilder
import org.junit.jupiter.api.Test
import org.knowm.xchart.XYChart
import org.knowm.xchart.style.markers.None
import org.knowm.xchart.style.theme.MatlabTheme
import kotlin.math.PI
import kotlin.math.sqrt
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

        val newController = RamseteController(trackWidth, bBar = 2.0)
        val oldController = OldRamseteController(trackWidth)

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

            val oldCommand: PoseVelocity2dDual<Time> = oldController.compute(s, targetPose, pose)
            val newCommand = newController.compute(s, targetPose, pose)

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

class OldRamseteController(
    trackWidth: Double,
    val zeta: Double = 0.7,
    bBar: Double = 2.0,
) {
    val b = bBar / (trackWidth * trackWidth)

    /**
     * Computes the velocity and acceleration command. The frame `Target` is the reference robot, and the frame `Actual`
     * is the measured, physical robot.
     *
     * @return velocity command in the actual frame
     */
    fun compute(
        s: DualNum<Time>,
        targetPose: Pose2dDual<Arclength>,
        actualPose: Pose2d,
    ): PoseVelocity2dDual<Time> {
        val targetHeading = targetPose.heading.value()
        val tangentHeading = targetPose.position.drop(1).value().angleCast()
        val dir = tangentHeading.real * targetHeading.real + tangentHeading.imag * targetHeading.imag
        val vRef = dir * s[1]

        val omegaRef = targetPose.reparam(s).heading.velocity()[0]

        val k = 2.0 * zeta * sqrt(omegaRef * omegaRef + b * vRef * vRef)

        val error = targetPose.value().minusExp(actualPose)
        return PoseVelocity2dDual.constant(
            PoseVelocity2d(
                Vector2d(
                    vRef * error.heading.real + k * error.position.x,
                    0.0
                ),
                omegaRef + k * error.heading.log() + b * vRef * sinc(error.heading.log()) * error.position.y,
            ),
            2
        )
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
