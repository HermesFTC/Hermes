package gay.zharel.hermes.ftc

import gay.zharel.hermes.control.RobotPosVelController
import gay.zharel.hermes.geometry.Pose2d
import gay.zharel.hermes.geometry.PoseVelocity2d
import gay.zharel.hermes.geometry.PoseVelocity2dDual
import gay.zharel.hermes.geometry.Time
import gay.zharel.hermes.paths.PosePath
import gay.zharel.hermes.profiles.AccelConstraint
import gay.zharel.hermes.profiles.VelConstraint
import gay.zharel.hermes.profiles.DisplacementProfile
import gay.zharel.hermes.trajectories.DisplacementTrajectory
import gay.zharel.hermes.trajectories.TrajectoryBuilder
import gay.zharel.hermes.trajectories.TurnConstraints

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
    fun createProfile(path: PosePath) = DisplacementProfile.generate(
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
