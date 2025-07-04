@file:JvmName("Drawing")

package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.dashboard.canvas.Canvas

@JvmOverloads
fun drawRobot(canvas: Canvas, pose: Pose2d, radius: Double = 9.0) {
    canvas.setStrokeWidth(1)
    canvas.strokeCircle(pose.position.x, pose.position.y, radius)

    val halfV = pose.heading.vec() * (0.5 * radius)
    val p1 = pose.position + halfV
    val p2 = p1 + halfV

    canvas.strokeLine(p1.x, p1.y, p2.x, p2.y)
}

fun Drive.drawPoseHistory(canvas: Canvas) {
    val xPoints = localizer.poseHistory.map { it.position.x }.toDoubleArray()
    val yPoints = localizer.poseHistory.map { it.position.y }.toDoubleArray()

    canvas.setStrokeWidth(1)
    canvas.setStroke("#3F51B5")
    canvas.strokePolyline(xPoints, yPoints)
}