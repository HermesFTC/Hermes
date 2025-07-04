package com.acmerobotics.roadrunner.actions

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.trajectories.TimeTrajectory
import com.acmerobotics.roadrunner.trajectories.TimeTurn
import kotlin.time.DurationUnit

class TrajectoryAction(val t: TimeTrajectory) : Action {
    override fun run(p: TelemetryPacket): Boolean {
        TODO("Not yet implemented")
    }

    override fun preview(fieldOverlay: Canvas) {
        TODO("Not yet implemented")
    }

    override fun toString() = "Trajectory"
}

class TurnAction(val t: TimeTurn) : Action {
    override fun run(p: TelemetryPacket): Boolean {
        TODO("Not yet implemented")
    }

    override fun preview(fieldOverlay: Canvas) {
        TODO("Not yet implemented")
    }

    override fun toString() = "Turn"
}

class LabelAction(val s: String) : Action {
    override fun run(p: TelemetryPacket): Boolean {
        TODO("Not yet implemented")
    }

    override fun preview(fieldOverlay: Canvas) {
        TODO("Not yet implemented")
    }

    override fun toString() = s
}

fun sexpFromAction(a: Action): Sexp =
    when (a) {
        is TrajectoryAction -> Sexp.Atom("traj")
        is TurnAction -> Sexp.Atom("turn")
        is LabelAction -> Sexp.Atom(a.s)
        is SleepAction -> Sexp.list(Sexp.Atom("sleep"), Sexp.Atom(String.format("%.10f", a.dt.toDouble(DurationUnit.SECONDS))))
        is SequentialAction -> Sexp.list(listOf(Sexp.Atom("seq")) + a.actions.map(::sexpFromAction))
        is ParallelAction -> Sexp.list(listOf(Sexp.Atom("par")) + a.actions.map(::sexpFromAction))
        is NullAction -> Sexp.Atom("null")
        else -> Sexp.Atom("unk")
    }