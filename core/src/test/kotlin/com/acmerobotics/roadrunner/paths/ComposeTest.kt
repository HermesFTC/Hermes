package com.acmerobotics.roadrunner.paths

import com.acmerobotics.roadrunner.TEST_ACCEL_CONSTRAINT
import com.acmerobotics.roadrunner.TEST_PROFILE_PARAMS
import com.acmerobotics.roadrunner.TEST_TRAJECTORY_BUILDER_PARAMS
import com.acmerobotics.roadrunner.TEST_VEL_CONSTRAINT
import com.acmerobotics.roadrunner.profiles.forwardProfile
import com.acmerobotics.roadrunner.randomAngle
import com.acmerobotics.roadrunner.randomPoint
import com.acmerobotics.roadrunner.randomPose
import com.acmerobotics.roadrunner.trajectories.CompositeTrajectory
import com.acmerobotics.roadrunner.trajectories.DisplacementTrajectory
import com.acmerobotics.roadrunner.trajectories.PositionPathSeqBuilder
import com.acmerobotics.roadrunner.trajectories.TrajectoryBuilder
import kotlin.random.Random
import kotlin.test.Test
import kotlin.test.assertEquals

class ComposeTest {
    @Test
    fun `composing pose paths`() {
        val posPaths = PositionPathSeqBuilder(
            randomPoint(),
            randomAngle(),
            TEST_TRAJECTORY_BUILDER_PARAMS.arcLengthSamplingEps
        ).splineTo(randomPoint(), randomAngle())
            .splineTo(randomPoint(), randomAngle())
            .splineTo(randomPoint(), randomAngle())
            .build()

        val posePaths = posPaths.map {
            TangentPath(it, 0.0)
        }

        val composed = CompositePosePath(posePaths)
        val profile = forwardProfile(TEST_PROFILE_PARAMS, composed, 0.0, TEST_VEL_CONSTRAINT, TEST_ACCEL_CONSTRAINT)
        val composedTraj = DisplacementTrajectory(composed, profile)

        val trajs = posePaths.map {
            val profile = forwardProfile(TEST_PROFILE_PARAMS, it, 0.0, TEST_VEL_CONSTRAINT, TEST_ACCEL_CONSTRAINT)
            DisplacementTrajectory(it, profile)
        }

        val compositeTraj = CompositeTrajectory(trajs)

        assert(composedTraj.length() == composed.length())

        (0..100).map {
            Random.Default.nextDouble(0.0, composedTraj.length())
        }.sorted().forEach {
            val composedPose = composedTraj[it].value()
            val compositePose = compositeTraj[it].value()
            println("disp $it, composed pose ${composedPose}, composite pose $compositePose")
            assertEquals(composedPose, compositePose)
        }
    }

    @Test
    fun `run trajectory compose`() {
        val traj = TrajectoryBuilder(
            TEST_TRAJECTORY_BUILDER_PARAMS,
            randomPose(),
            0.0,
            TEST_VEL_CONSTRAINT,
            TEST_ACCEL_CONSTRAINT
        )
            .splineTo(randomPoint(), randomAngle())
            .splineTo(randomPoint(), randomAngle())
            .strafeTo(randomPoint())
            .splineTo(randomPoint(), randomAngle())
            .build()

        val samples = (0..100).map { Random.Default.nextDouble(0.0, traj.length()) }.sorted()

        samples.forEach {
            val pose = traj[it].value()
            println("disp $it, pose $pose")
        }
    }
}