package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.actions.Action
import com.acmerobotics.roadrunner.actions.ActionEx
import com.acmerobotics.roadrunner.actions.Actions
import com.acmerobotics.roadrunner.actions.drawPoseHistory
import com.acmerobotics.roadrunner.actions.drawRobot
import com.acmerobotics.roadrunner.geometry.PoseVelocity2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.geometry.xs
import com.acmerobotics.roadrunner.geometry.ys
import com.acmerobotics.roadrunner.paths.PosePath
import com.acmerobotics.roadrunner.profiles.AccelConstraint
import com.acmerobotics.roadrunner.profiles.VelConstraint
import com.acmerobotics.roadrunner.trajectories.TimeTurn
import com.acmerobotics.roadrunner.trajectories.Trajectory
import kotlin.properties.Delegates

class FollowTrajectoryAction(private val follower: Follower, private val drive: Drive) : Action {
    private val xPoints: DoubleArray = follower.points.map { it.x }.toDoubleArray()
    private val yPoints: DoubleArray = follower.points.map { it.y }.toDoubleArray()

    constructor(traj: Trajectory<*>, drive: Drive) : this(DisplacementFollower(traj, drive), drive)

    constructor(
        path: PosePath,
        drive: Drive,
        velConstraintOverride: VelConstraint,
        accelConstraintOverride: AccelConstraint,
    ) : this(DisplacementFollower(path, drive, velConstraintOverride, accelConstraintOverride), drive)

    override fun run(p: TelemetryPacket): Boolean {
        follower.follow()

        p.put("x", drive.localizer.pose.position.x)
        p.put("y", drive.localizer.pose.position.y)
        p.put("heading (deg)", Math.toDegrees(drive.localizer.pose.heading.toDouble()))

        val error = follower.currentTarget.minusExp(drive.localizer.pose)
        p.put("xError", error.position.x)
        p.put("yError", error.position.y)
        p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()))

        // only draw when active; only one drive action should be active at a time
        val c = p.fieldOverlay()
        drive.drawPoseHistory(c)

        c.setStroke("#4CAF50")
        drawRobot(c, follower.currentTarget)

        c.setStroke("#3F51B5")
        drawRobot(c, drive.localizer.pose)

        c.setStroke("#4CAF50FF")
        c.setStrokeWidth(1)
        c.strokePolyline(xPoints, yPoints)

        return follower.isDone
    }
}

class TurnAction(
    @JvmField
    val turn: TimeTurn,
    @JvmField
    val drive: Drive,
) : ActionEx() {
    var startTime by Delegates.notNull<Double>()

    override fun init(p: TelemetryPacket) {
        startTime = Actions.now()
    }

    override fun loop(p: TelemetryPacket): Boolean {
        val t = Actions.now() - startTime

        if (t > turn.duration) {
            drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
            return false
        }

        val target = turn[t]
        val robotVel = drive.localizer.update()

        val command = drive.controller.compute(
            target,
            drive.localizer.pose,
            robotVel,
        )

        drive.setDrivePowersWithFF(command)

        val c = p.fieldOverlay()
        drive.drawPoseHistory(c)

        c.setStroke("#4CAF50")
        drawRobot(c, target.value())

        c.setStroke("#3F51B5")
        drawRobot(c, drive.localizer.pose)

        c.setStroke("#7C4DFFFF")
        c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)

        return true
    }
}