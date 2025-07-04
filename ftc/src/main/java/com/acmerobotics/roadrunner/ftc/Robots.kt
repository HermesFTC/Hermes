package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.actions.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.control.RobotKinematics
import com.acmerobotics.roadrunner.control.RobotPosVelController
import com.acmerobotics.roadrunner.control.WheelIncrements
import com.acmerobotics.roadrunner.control.WheelVelocities
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.PoseVelocity2d
import com.acmerobotics.roadrunner.geometry.PoseVelocity2dDual
import com.acmerobotics.roadrunner.geometry.Time
import com.acmerobotics.roadrunner.paths.PosePath
import com.acmerobotics.roadrunner.profiles.AccelConstraint
import com.acmerobotics.roadrunner.profiles.VelConstraint
import com.acmerobotics.roadrunner.profiles.forwardProfile
import com.acmerobotics.roadrunner.trajectories.DisplacementTrajectory
import com.acmerobotics.roadrunner.trajectories.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectories.TurnConstraints

interface Drive {
    val localizer: Localizer
    val controller: RobotPosVelController

    val followerParams: FollowerParams
    val defaultVelConstraint: VelConstraint get() = followerParams.velConstraint
    val defaultAccelConstraint: AccelConstraint get() = followerParams.accelConstraint
    val defaultTurnConstraints: TurnConstraints

    fun trajectoryBuilder(startPose: Pose2d): TrajectoryBuilder
    fun trajectoryBuilder() = trajectoryBuilder(localizer.pose)

    fun setDrivePowers(powers: PoseVelocity2dDual<Time>)
    fun setDrivePowersWithFF(powers: PoseVelocity2dDual<Time>)

    fun setDrivePowers(powers: PoseVelocity2d) =
        setDrivePowers(PoseVelocity2dDual.constant(powers, 3))
    fun setDrivePowersWithFF(powers: PoseVelocity2d) =
        setDrivePowersWithFF(PoseVelocity2dDual.constant(powers, 3))

    fun updatePoseEstimate(): PoseVelocity2d {
        return localizer.update()
    }

    /**
     * Creates and returns a time-optimal profile for the given path.
     */
    fun createProfile(path: PosePath) = forwardProfile(
            followerParams.profileParams,
            path,
            0.0,
            followerParams.velConstraint,
            followerParams.accelConstraint,
        )

    /**
     * Creates and returns a time-optimal profile for the given path.
     */
    fun createTrajectory(path: PosePath) = DisplacementTrajectory(
        path,
        createProfile(path)
    )
}

interface Localizer {
    var pose: Pose2d
    val poseHistory: MutableList<Pose2d>

    fun update(): PoseVelocity2d
}