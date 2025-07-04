package com.acmerobotics.roadrunner.paths

import com.acmerobotics.roadrunner.geometry.DualNum
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Pose2dDual
import com.acmerobotics.roadrunner.geometry.PoseVelocity2d
import com.acmerobotics.roadrunner.geometry.Rotation2dDual
import com.acmerobotics.roadrunner.geometry.Time
import com.acmerobotics.roadrunner.geometry.Twist2d
import com.acmerobotics.roadrunner.geometry.Vector2dDual
import com.acmerobotics.roadrunner.profiles.DisplacementProfile
import com.acmerobotics.roadrunner.saveChart
import org.junit.jupiter.api.Test
import org.knowm.xchart.XYChartBuilder
import org.knowm.xchart.style.markers.Circle
import org.knowm.xchart.style.theme.MatlabTheme

class Demo {
    @Test
    fun test() {
        val path = TangentPath(
            ArclengthReparamCurve2d(
                QuinticSpline2dInternal(
                    QuinticSpline1d(
                        DualNum(doubleArrayOf(0.0, 40.0, 0.0)),
                        DualNum(doubleArrayOf(80.0, 40.0, 0.0)),
                    ),
                    QuinticSpline1d(
                        DualNum(doubleArrayOf(0.0, 0.0, 0.0)),
                        DualNum(doubleArrayOf(20.0, 0.0, 0.0)),
                    ),
                ),
                1e-6,
            ),
            0.0
        )

        val profile = DisplacementProfile(
            listOf(0.0, path.length()),
            listOf(1.0, 1.0),
            listOf(0.0)
        )

        var s = 0.0
        var pose = Pose2dDual<Time>(
            Vector2dDual(
                DualNum.Companion.constant(5.0, 3),
                DualNum.Companion.constant(5.0, 3),
            ),
            Rotation2dDual.Companion.exp(DualNum.Companion.constant(0.0, 3)),
        )

        val measured = mutableListOf<Pose2d>()
        val targets = mutableListOf<Pose2d>()

        while (true) {
            s = path.project(pose.position.value(), s)

            if (s >= path.length() - 0.25) {
                break
            }

            val targetPose = path[s, 3].reparam(profile[s])

            measured.add(pose.value())
            targets.add(targetPose.value())

            val error = targetPose.value().minusExp(pose.value())
            val correction = PoseVelocity2d(
                error.position * 0.5,
                error.heading.log() * 0.01,
            )

            val velocity = (targetPose.velocity() + correction).value()

            val dt = 0.01
            pose += Twist2d(
                velocity.linearVel * dt,
                velocity.angVel * dt,
            )
        }

        val chart = XYChartBuilder()
            .width(600).height(500)
            .title("Demo").xAxisTitle("x").yAxisTitle("y")
            .build()
        chart.styler.theme = MatlabTheme()

        chart.addSeries(
            "Target",
            targets.map { it.position.x }.toDoubleArray(),
            targets.map { it.position.y }.toDoubleArray(),
        ).let {
            it.marker = Circle()
            it.lineWidth = 0.0f
        }

        chart.addSeries(
            "Actual",
            measured.map { it.position.x }.toDoubleArray(),
            measured.map { it.position.y }.toDoubleArray(),
        ).let {
            it.marker = Circle()
            it.lineWidth = 0.0f
        }

        saveChart("demo", chart)
    }
}