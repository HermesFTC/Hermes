package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.PoseVelocity2d

interface Localizer {
    var pose: Pose2d
    val poseHistory: MutableList<Pose2d>

    fun update(): PoseVelocity2d
}