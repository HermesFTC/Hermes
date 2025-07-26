package me.zharel.hermes.wpi

import com.acmerobotics.roadrunner.control.RobotPosVelController
import com.acmerobotics.roadrunner.profiles.AccelConstraint
import com.acmerobotics.roadrunner.profiles.VelConstraint
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds

interface Drive {
    val localizer: Localizer
    val controller: RobotPosVelController

    val followerParams: FollowerParams
    val defaultVelConstraint: VelConstraint get() = followerParams.velConstraint
    val defaultAccelConstraint: AccelConstraint get() = followerParams.accelConstraint

    fun trajectoryBuilder(startPose: Pose2d): TrajectoryBuilder
    fun trajectoryBuilder() = trajectoryBuilder(localizer.pose)

    fun setDrivePowers(powers: ChassisSpeeds)
}
