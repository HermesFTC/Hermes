package com.acmerobotics.roadrunner.actions

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket

/**
 * Action combinator that executes the action group [initialActions] in series. Each action is run one after the other.
 * When an action completes, the next one is immediately run. This action completes when the last action completes.
 */
data class SequentialAction(
    val initialActions: List<Action>
) : Action {
    var actions = initialActions.flatMap {
        if (it is SequentialAction) it.initialActions else listOf(it)
    }
        private set

    constructor(vararg actions: Action) : this(actions.asList())

    override tailrec fun run(p: TelemetryPacket): Boolean {
        if (actions.isEmpty()) {
            return false
        }

        return if (actions.first().run(p)) {
            true
        } else {
            actions = actions.drop(1)
            run(p)
        }
    }

    override fun preview(fieldOverlay: Canvas) {
        for (a in initialActions) {
            a.preview(fieldOverlay)
        }
    }
}

/**
 * Action combinator that executes the action group [initialActions] in parallel. Each call to [run] on this action
 * calls [run] on _every_ live child action in the order provided. Completed actions are removed from the rotation
 * and _do not_ prevent the completion of other actions. This action completes when all of [initialActions] have.
 */
data class ParallelAction(
    val initialActions: List<Action>
) : Action {
    var actions = initialActions.flatMap {
        if (it is ParallelAction) it.initialActions else listOf(it)
    }
        private set

    constructor(vararg actions: Action) : this(actions.asList())

    override fun run(p: TelemetryPacket): Boolean {
        actions = actions.filter { it.run(p) }
        return actions.isNotEmpty()
    }

    override fun preview(fieldOverlay: Canvas) {
        for (a in initialActions) {
            a.preview(fieldOverlay)
        }
    }
}

/**
 * Action combinator that executes the action group [initialActions] in parallel. Each call to [run] on this action
 * calls [run] on _every_ live child action in the order provided. Once one action ends, all other actions are ended.
 */
class RaceAction(
    initialActions: List<Action>
) : Action {
    var actions = initialActions
        private set
    var interrupting = false

    constructor(vararg actions: Action) : this(actions.asList())

    override fun run(p: TelemetryPacket): Boolean {
        val remaining = actions.filter { it.run(p) }
        if (interrupting) {
            return remaining.isNotEmpty()
        } else if (actions.size != remaining.size) {
            interrupting = true
            actions = remaining.filter { it is Interruptible }
                .map { (it as Interruptible).onInterrupt() }
        }
        return true
    }

    override fun preview(fieldOverlay: Canvas) {
        for (a in actions) {
            a.preview(fieldOverlay)
        }
    }
}
