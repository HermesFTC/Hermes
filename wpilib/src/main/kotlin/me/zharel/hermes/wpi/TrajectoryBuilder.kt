package me.zharel.hermes.wpi

import com.acmerobotics.roadrunner.profiles.AccelConstraint
import com.acmerobotics.roadrunner.profiles.VelConstraint
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Distance

typealias iTrajectoryBuilder = com.acmerobotics.roadrunner.trajectories.TrajectoryBuilder

data class ProfileParams(
    val dispResolution: Double,
    val angResolution: Double,
    val angSamplingEps: Double,
)

fun ProfileParams.hermes() = com.acmerobotics.roadrunner.profiles.ProfileParams(
    dispResolution,
    angResolution,
    angSamplingEps
)


data class TrajectoryBuilderParams(
    val arclengthSamplingEps: Double,
    val profileParams: ProfileParams,
)

internal fun TrajectoryBuilderParams.hermes() = com.acmerobotics.roadrunner.trajectories.TrajectoryBuilderParams(
    arclengthSamplingEps,
    profileParams.hermes()
)



class TrajectoryBuilder(
    params: TrajectoryBuilderParams,
    beginPose: Pose2d,
    beginEndVel: Double,
    baseVelConstraint: VelConstraint,
    baseAccelConstraint: AccelConstraint,
    poseMap: PoseMap = PoseMap.Identity,
) {
    var builder = iTrajectoryBuilder(
        params.hermes(),
        beginPose.hermes(),
        beginEndVel,
        baseVelConstraint,
        baseAccelConstraint,
        poseMap.hermes()
    )

    fun setTangent(tangent: Rotation2d) = apply {
        builder = builder.setTangent(tangent.hermes())
    }

    fun setReversed(reversed: Boolean) = apply {
        builder = builder.setReversed(reversed)
    }

    fun forward(ds: Distance) = apply {
        builder = builder.forward(ds.inchValue())
    }

    fun lineToX(posX: Distance) = apply {
        builder = builder.lineToX(posX.inchValue())
    }

    fun lineToY(posY: Distance) {
        builder = builder.lineToY(posY.inchValue())
    }

    fun strafeTo(pos: Translation2d) = apply {
        builder = builder.strafeTo(pos.hermes())
    }

    fun splineTo(pos: Translation2d, tangent: Rotation2d) = apply {
        builder = builder.splineTo(pos.hermes(), tangent.hermes())
    }

    fun splineToLinearHeading(pose: Pose2d, tangent: Rotation2d) = apply {
        builder = builder.splineToLinearHeading(pose.hermes(), tangent.hermes())
    }

    fun splineToSplineHeading(pose: Pose2d, tangent: Rotation2d) = apply {
        builder = builder.splineToSplineHeading(pose.hermes(), tangent.hermes())
    }

    fun build(): Trajectory {
        return TrajectoryWrapper(builder.buildToComposite())
    }

}