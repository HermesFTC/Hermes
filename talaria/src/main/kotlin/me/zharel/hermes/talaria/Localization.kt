package me.zharel.hermes.talaria

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds

interface Localizer {
    var pose: Pose2d

    fun getVel(): ChassisSpeeds

    fun update(): ChassisSpeeds
}