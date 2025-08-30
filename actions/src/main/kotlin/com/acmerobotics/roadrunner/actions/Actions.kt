package com.acmerobotics.roadrunner.actions

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import java.util.function.Consumer
import kotlin.time.ComparableTimeMark
import kotlin.time.Duration
import kotlin.time.Duration.Companion.nanoseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit
import kotlin.time.TimeSource.Monotonic.markNow
import kotlin.time.toKotlinDuration

/**
 * Concurrent task for cooperative multitasking with some FTC dashboard hooks. Actions may have a mutable state.
 */
fun interface Action {
    /**
     * Runs a single uninterruptible block. Returns true if the action should run again and false if it has completed.
     * A telemetry packet [p] is provided to record any information on the action's progress.
     */
    fun run(p: TelemetryPacket): Boolean

    /**
     * Draws a preview of the action on canvas [fieldOverlay].
     */
    fun preview(fieldOverlay: Canvas) {}

    /**
     * Returns a new action that executes this action followed by [a].
     */
    fun then(a: Action) = SequentialAction(this, a)

    /**
     * Returns a new action that executes this action followed by [f].
     */
    fun then(f: InstantFunction) = SequentialAction(this, InstantAction(f))

    /**
     * Returns a new action that executes this action followed by [a].
     */
    fun then(a: () -> Action) = SequentialAction(this, a())

    /**
     * Returns a new action that executes this action in parallel with [a].
     */
    fun with(a: Action) = ParallelAction(this, a)

    /**
     * Returns a new action that executes this action in parallel with [f].
     */
    fun with(f: InstantFunction) = ParallelAction(this, InstantAction(f))

    /**
     * Returns a new action that executes this action in parallel with [a].
     */
    fun with(a: () -> Action) = ParallelAction(this, a())

    /**
     * Returns a new action that executes this action in parallel with [a],
     * but stops both actions when one finishes first.
     */
    fun race(a: Action) = RaceAction(this, a)

    /**
     * Returns a new action that executes this action in parallel with [f],
     * but stops both actions when one finishes first.
     */
    fun race(f: InstantFunction) = RaceAction(this, InstantAction(f))

    /**
     * Returns a new action that executes this action in parallel with [a],
     * but stops both actions when one finishes first.
     */
    fun race(a: () -> Action) = RaceAction(this, a())

    /**
     * Returns a new action that waits [dt] before executing this action.
     */
    fun delay(dt: Duration) = Actions.sleep(dt).then(this)

    /**
     * Returns a new action that waits [dt] before executing this action.
     */
    fun delay(dt: java.time.Duration) = Actions.sleep(dt).then(this)

    /**
     * Returns an interruptible copy of this action, with [onInterruption] occurring on interrupt.
     */
    fun interruptible(onInterruption: Action) = object : Interruptible {
        override fun onInterrupt(): Action = onInterruption

        override fun run(p: TelemetryPacket): Boolean = this@Action.run(p)

        override fun preview(fieldOverlay: Canvas) = this@Action.preview(fieldOverlay)
    }

    /**
     * Returns an interruptible copy of this action, with [onInterruption] occurring on interrupt.
     */
    fun interruptible(onInterruption: InstantFunction) = interruptible(InstantAction(onInterruption))

    /**
     * Returns an interruptible copy of this action, with [onInterruption] occurring on interrupt.
     */
    fun interruptible(onInterruption: () -> Action) = interruptible(onInterruption())

    /**
     * Returns a copy of this with dashboard preview [preview].
     */
    fun withPreview(preview: (Canvas) -> Unit) = object : Action {
        override fun run(p: TelemetryPacket) = this@Action.run(p)
        override fun preview(fieldOverlay: Canvas) = preview(fieldOverlay)
    }

    /**
     * Returns a copy of this with dashboard preview [preview].
     */
    fun withPreview(preview: Consumer<Canvas>) = withPreview(preview::accept)
}

open class ActionEx @JvmOverloads constructor(
    private val initBlock: (TelemetryPacket) -> Unit = { },
    private val loopBlock: (TelemetryPacket) -> Boolean = { false },
    private val endBlock: (TelemetryPacket) -> Unit = { }
) : Action {

    open fun init(packet: TelemetryPacket) = initBlock(packet)
    open fun loop(packet: TelemetryPacket) = loopBlock(packet)
    open fun end(packet: TelemetryPacket) = endBlock(packet)

    private val sequential = SequentialAction(
        { init(it).let { false } },
        { loop(it) },
        { end(it).let { false } }
    )

    final override fun run(p: TelemetryPacket) = sequential.run(p)

    fun withInit(initBlock: (TelemetryPacket) -> Unit) = ActionEx(initBlock, loopBlock, endBlock)
    fun withLoop(loopBlock: (TelemetryPacket) -> Boolean) = ActionEx(initBlock, loopBlock, endBlock)
    fun withEnd(endBlock: (TelemetryPacket) -> Unit) = ActionEx(initBlock, loopBlock, endBlock)
}

/**
 * Utility object for action-related functionality.
 */
object Actions {
    /**
     * Returns the current time in seconds.
     */
    @JvmStatic fun now() = System.nanoTime().nanoseconds.toDouble(DurationUnit.SECONDS)

    /**
     * Sleeps for [dt].
     */
    @JvmStatic fun sleep(dt: Duration): Action {
        lateinit var mark: ComparableTimeMark
        return ActionEx()
            .withInit { mark = markNow() }
            .withLoop { mark.elapsedNow() < dt }
    }

    /**
     * Sleeps for [dt].
     */
    @JvmStatic fun sleep(dt: java.time.Duration) = sleep(dt.toKotlinDuration())
}

fun interface InstantFunction {
    fun run()
}

/**
 * Instant action that executes [f] immediately.
 */
class InstantAction(val f: InstantFunction) : Action {
    override fun run(p: TelemetryPacket): Boolean {
        f.run()
        return false
    }
}

/**
 * Null action that does nothing.
 */
class NullAction : Action {
    override fun run(p: TelemetryPacket) = false
}

/**
 * An action that can be interrupted, providing a specific action to execute upon interruption.
 */
interface Interruptible : Action {

    /**
     * Returns the action to execute upon interruption.
     */
    fun onInterrupt(): Action
}
