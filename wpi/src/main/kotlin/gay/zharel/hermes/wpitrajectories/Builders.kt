@file:JvmName("TrajectoryBuilders")

package gay.zharel.hermes.wpitrajectories

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.units.measure.LinearVelocity
import gay.zharel.hermes.paths.IdentityPoseMap
import gay.zharel.hermes.paths.PoseMap
import gay.zharel.hermes.profiles.ProfileParams
import gay.zharel.hermes.trajectories.TrajectoryBuilder
import gay.zharel.hermes.trajectories.TrajectoryBuilderParams
import gay.zharel.hermes.wpiconversions.ips
import gay.zharel.hermes.wpiconversions.mps

val DEFAULT_PROFILE_PARAMS = ProfileParams(0.25, Math.PI / 8, 1e-4,)
val DEFAULT_BUILDER_PARAMS = TrajectoryBuilderParams(1e-6, DEFAULT_PROFILE_PARAMS)

class DifferentialTrajectoryBuilder internal constructor(
    private val builder: TrajectoryBuilder,
    private val kinematics: DifferentialDriveKinematics
) {
    @JvmOverloads constructor(
        constraint: DifferentialDriveConstraint,
        startPose: Pose2d,
        params: TrajectoryBuilderParams = DEFAULT_BUILDER_PARAMS,
        beginEndVel: LinearVelocity = 0.0.mps,
        poseMap: PoseMap = IdentityPoseMap,
    ) : this(
        TrajectoryBuilder(
            params = params,
            beginPose = startPose,
            beginEndVel = beginEndVel.ips,
            baseVelConstraint = constraint,
            baseAccelConstraint = constraint,
            poseMap = poseMap
        ),
        constraint.kinematics
    )
}