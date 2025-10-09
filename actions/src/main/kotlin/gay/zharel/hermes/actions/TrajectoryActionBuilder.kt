package gay.zharel.hermes.actions

import gay.zharel.hermes.math.Arclength
import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.geometry.Pose2d
import gay.zharel.hermes.geometry.Pose2dDual
import gay.zharel.hermes.geometry.Rotation2d
import gay.zharel.hermes.geometry.Rotation2dDual
import gay.zharel.hermes.geometry.Vector2d
import gay.zharel.hermes.geometry.Vector2dDual
import gay.zharel.hermes.profiles.AccelConstraint
import gay.zharel.hermes.profiles.VelConstraint
import gay.zharel.hermes.paths.IdentityPoseMap
import gay.zharel.hermes.paths.MappedPosePath
import gay.zharel.hermes.paths.PoseMap
import gay.zharel.hermes.trajectories.TimeTrajectory
import gay.zharel.hermes.trajectories.TimeTurn
import gay.zharel.hermes.trajectories.Trajectory
import gay.zharel.hermes.trajectories.TrajectoryBuilder
import gay.zharel.hermes.trajectories.TrajectoryBuilderParams
import gay.zharel.hermes.trajectories.TurnConstraints
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds
import kotlin.time.toKotlinDuration

private fun seqCons(hd: Action, tl: Action) = SequentialAction(hd, tl).withoutNulls().let {
    if (it.actions.size == 1) it.actions.first() else it
}

fun SequentialAction.withoutNulls(): SequentialAction = SequentialAction(actions.filter { it !is NullAction })
fun ParallelAction.withoutNulls(): ParallelAction = ParallelAction(actions.filter { it !is NullAction })

class SleepAction(val dt: Duration) : Action by Actions.sleep(dt) {
    constructor(dt: java.time.Duration) : this(dt.toKotlinDuration())
    constructor(dt: Double) : this(dt.seconds)
}

private sealed class MarkerFactory(
    val segmentIndex: Int,
) {
    abstract fun make(t: TimeTrajectory, segmentDisp: Double): Action
}

private class TimeMarkerFactory(segmentIndex: Int, val dt: Double, val a: Action) : MarkerFactory(segmentIndex) {
    override fun make(t: TimeTrajectory, segmentDisp: Double) =
        seqCons(SleepAction((t.profile.inverse(segmentDisp) + dt).seconds), a)
}

private class DispMarkerFactory(segmentIndex: Int, val ds: Double, val a: Action) : MarkerFactory(segmentIndex) {
    override fun make(t: TimeTrajectory, segmentDisp: Double) =
        seqCons(SleepAction(t.profile.inverse(segmentDisp + ds).seconds), a)
}

fun interface TurnActionFactory {
    fun make(t: TimeTurn): Action
}

fun interface TrajectoryActionFactory {
    fun make(t: Trajectory<*>): Action
}

/**
 * Builder that combines trajectories, turns, and other actions.
 */
class TrajectoryActionBuilder private constructor(
    // constants
    val turnActionFactory: TurnActionFactory,
    val trajectoryActionFactory: TrajectoryActionFactory,
    val trajectoryBuilderParams: TrajectoryBuilderParams,
    val beginEndVel: Double,
    val baseTurnConstraints: TurnConstraints,
    val baseVelConstraint: VelConstraint,
    val baseAccelConstraint: AccelConstraint,
    val poseMap: PoseMap,
    // vary throughout
    private val tb: TrajectoryBuilder,
    private val n: Int,
    // lastPose, lastTangent are post-mapped
    private val lastPoseUnmapped: Pose2d,
    private val lastPose: Pose2d,
    private val lastTangent: Rotation2d,
    private val ms: List<MarkerFactory>,
    private val cont: (Action) -> Action,
) {
    @JvmOverloads
    constructor(
        turnActionFactory: TurnActionFactory,
        trajectoryActionFactory: TrajectoryActionFactory,
        trajectoryBuilderParams: TrajectoryBuilderParams,
        beginPose: Pose2d,
        beginEndVel: Double,
        baseTurnConstraints: TurnConstraints,
        baseVelConstraint: VelConstraint,
        baseAccelConstraint: AccelConstraint,
        poseMap: PoseMap = IdentityPoseMap,
    ) :
        this(
            turnActionFactory,
            trajectoryActionFactory,
            trajectoryBuilderParams,
            beginEndVel,
            baseTurnConstraints,
            baseVelConstraint,
            baseAccelConstraint,
            poseMap,
            TrajectoryBuilder(
                trajectoryBuilderParams,
                beginPose, beginEndVel,
                baseVelConstraint, baseAccelConstraint,
                poseMap,
            ),
            0,
            beginPose,
            poseMap.map(beginPose),
            poseMap.map(beginPose).heading,
            emptyList(),
            { it },
        )

    private constructor(
        ab: TrajectoryActionBuilder,
        tb: TrajectoryBuilder,
        n: Int,
        lastPoseUnmapped: Pose2d,
        lastPose: Pose2d,
        lastTangent: Rotation2d,
        ms: List<MarkerFactory>,
        cont: (Action) -> Action,
    ) :
        this(
            ab.turnActionFactory,
            ab.trajectoryActionFactory,
            ab.trajectoryBuilderParams,
            ab.beginEndVel,
            ab.baseTurnConstraints,
            ab.baseVelConstraint,
            ab.baseAccelConstraint,
            ab.poseMap,
            tb,
            n,
            lastPoseUnmapped,
            lastPose,
            lastTangent,
            ms,
            cont
        )

    /**
     * Ends the current trajectory in progress. No-op if no trajectory segments are pending.
     */
    fun endTrajectory() =
        if (n == 0) {
            require(ms.isEmpty()) { "Cannot end trajectory with pending markers" }

            this
        } else {
            val ts = tb.buildToList()
            val endPoseUnmapped = when (ts.last().path) {
                is MappedPosePath -> (ts.last().path as MappedPosePath).basePath.end(2)
                else -> ts.last().path.end(2)
            }.value()
            val end = ts.last().path.end(2)
            val endPose = end.value()
            val endTangent = end.velocity().value().linearVel.angleCast()
            TrajectoryActionBuilder(
                this,
                TrajectoryBuilder(
                    trajectoryBuilderParams,
                    endPoseUnmapped,
                    beginEndVel,
                    baseVelConstraint,
                    baseAccelConstraint,
                    poseMap,
                ),
                0,
                endPoseUnmapped,
                endPose,
                endTangent,
                emptyList()
            ) { tail ->
                val (aNew, msRem) = ts.zip(ts.scan(0) { acc, t -> acc + t.offsets.size - 1 }).foldRight(
                    Pair(tail, ms)
                ) { (traj, offset), (acc, ms) ->
                    val timeTraj = TimeTrajectory(traj)
                    val actions = mutableListOf<Action>(seqCons(trajectoryActionFactory.make(timeTraj), acc))
                    val msRem = mutableListOf<MarkerFactory>()
                    for (m in ms) {
                        val i = m.segmentIndex - offset
                        if (i >= 0) {
                            actions.add(m.make(timeTraj, traj.offsets[i]))
                        } else {
                            msRem.add(m)
                        }
                    }

                    when (actions.size) {
                        0 -> Pair(NullAction(), msRem)
                        1 -> Pair(actions.first(), msRem)
                        else -> Pair(ParallelAction(actions).withoutNulls(), msRem)
                    }
                }

                require(msRem.isEmpty()) { "Unresolved markers" }

                cont(aNew)
            }
        }

    /**
     * Stops the current trajectory (like [endTrajectory]) and adds action [a] next.
     */
    fun stopAndAdd(a: Action): TrajectoryActionBuilder {
        val b = endTrajectory()
        return TrajectoryActionBuilder(b, b.tb, b.n, b.lastPoseUnmapped, b.lastPose, b.lastTangent, b.ms) { tail ->
            b.cont(seqCons(a, tail))
        }
    }
    fun stopAndAdd(f: InstantFunction) = stopAndAdd(InstantAction(f))

    /**
     * Waits [t] seconds.
     */
    fun waitSeconds(t: Double): TrajectoryActionBuilder {
        require(t >= 0.0) { "Time ($t) must be non-negative" }

        return stopAndAdd(SleepAction(t.seconds))
    }

    /**
     * Schedules action [a] to execute in parallel starting at a displacement [ds] after the last trajectory segment.
     * The action start is clamped to the span of the current trajectory.
     *
     * Cannot be called without an applicable pending trajectory.
     */
    // TODO: Should calling this without an applicable trajectory implicitly begin an empty trajectory and execute the
    // action immediately?
    fun afterDisp(ds: Double, a: Action): TrajectoryActionBuilder {
        require(ds >= 0.0) { "Displacement ($ds) must be non-negative" }

        return TrajectoryActionBuilder(
            this, tb, n, lastPoseUnmapped, lastPose, lastTangent,
            ms + listOf(DispMarkerFactory(n, ds, a)), cont
        )
    }
    fun afterDisp(ds: Double, f: InstantFunction) = afterDisp(ds, InstantAction(f))

    /**
     * Schedules action [a] to execute in parallel starting [dt] seconds after the last trajectory segment, turn, or
     * other action.
     */
    fun afterTime(dt: Double, a: Action): TrajectoryActionBuilder {
        require(dt >= 0.0) { "Time ($dt) must be non-negative" }

        return if (n == 0) {
            TrajectoryActionBuilder(this, tb, 0, lastPoseUnmapped, lastPose, lastTangent, emptyList()) { tail ->
                val m = seqCons(SleepAction(dt.seconds), a)
                if (tail is NullAction) {
                    cont(m)
                } else {
                    cont(ParallelAction(tail, m))
                }
            }
        } else {
            TrajectoryActionBuilder(
                this, tb, n, lastPoseUnmapped, lastPose, lastTangent,
                ms + listOf(TimeMarkerFactory(n, dt, a)), cont
            )
        }
    }
    fun afterTime(dt: Double, f: InstantFunction) = afterTime(dt, InstantAction(f))

    /**
     * Sets the starting tangent of the next path segment.
     * See [RoadRunner docs](https://rr.brott.dev/docs/v1-0/guides/tangents/).
     */
    fun setTangent(r: Rotation2d) =
        TrajectoryActionBuilder(this, tb.setTangent(r), n, lastPoseUnmapped, lastPose, lastTangent, ms, cont)

    /**
     * Sets the starting tangent of the next path segment.
     * See [RoadRunner docs](https://rr.brott.dev/docs/v1-0/guides/tangents/).
     */
    fun setTangent(r: Double) = setTangent(Rotation2d.exp(r))

    /**
     * Reverses the next path segment; actually a call to [setTangent(Math.PI)][setTangent]!
     */
    fun setReversed(reversed: Boolean) =
        TrajectoryActionBuilder(this, tb.setReversed(reversed), n, lastPoseUnmapped, lastPose, lastTangent, ms, cont)

    /**
     * Turns [angle] radians.
     */
    @JvmOverloads
    fun turn(angle: Double, turnConstraintsOverride: TurnConstraints? = null): TrajectoryActionBuilder {
        val b = endTrajectory()
        val mappedAngle =
            poseMap.map(
                Pose2dDual(
                    Vector2dDual.constant(b.lastPose.position, 2),
                    Rotation2dDual.constant<Arclength>(b.lastPose.heading, 2) + DualNum(listOf(0.0, angle))
                )
            ).heading.velocity().value()
        val b2 = b.stopAndAdd(
            turnActionFactory.make(
                TimeTurn(b.lastPose, mappedAngle, turnConstraintsOverride ?: baseTurnConstraints)
            )
        )
        val lastPoseUnmapped = Pose2d(b2.lastPoseUnmapped.position, b2.lastPoseUnmapped.heading + angle)
        val lastPose = Pose2d(b2.lastPose.position, b2.lastPose.heading + mappedAngle)
        val lastTangent = b2.lastTangent + mappedAngle
        return TrajectoryActionBuilder(
            b2,
            TrajectoryBuilder(
                trajectoryBuilderParams,
                lastPoseUnmapped,
                beginEndVel,
                baseVelConstraint,
                baseAccelConstraint,
                poseMap
            ),
            b2.n, lastPoseUnmapped, lastPose, lastTangent, b2.ms, b2.cont
        )
    }
    /**
     * Turns to face heading [heading].
     */
    @JvmOverloads
    fun turnTo(heading: Rotation2d, turnConstraintsOverride: TurnConstraints? = null): TrajectoryActionBuilder {
        val b = endTrajectory()
        return b.turn(heading - b.lastPose.heading, turnConstraintsOverride)
    }
    @JvmOverloads
    fun turnTo(heading: Double, turnConstraintsOverride: TurnConstraints? = null) =
        turnTo(Rotation2d.exp(heading), turnConstraintsOverride)

    /**
     * Adds a line segment that goes forward [ds] in the current direction.
     */
    @JvmOverloads
    fun forward(
        ds: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.forward(
            ds, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes forward [ds] while maintaining current heading.
     * Equivalent to [forward].
     */
    @JvmOverloads
    fun forwardConstantHeading(
        ds: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.forwardConstantHeading(
            ds, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes forward [ds],
     * changing heading from current to [heading] using linear interpolation.
     */
    @JvmOverloads
    fun forwardLinearHeading(
        ds: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.forwardLinearHeading(
            ds, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes forward [ds],
     * changing heading from current to [heading] using linear interpolation.
     */
    @JvmOverloads
    fun forwardLinearHeading(
        ds: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.forwardLinearHeading(
            ds, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes forward [ds],
     * changing heading from current to [heading] using spline interpolation.
     */
    @JvmOverloads
    fun forwardSplineHeading(
        ds: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.forwardSplineHeading(
            ds, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes forward [ds],
     * changing heading from current to [heading] using spline interpolation.
     */
    @JvmOverloads
    fun forwardSplineHeading(
        ds: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.forwardSplineHeading(
            ds, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes to x-coordinate [posX].
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the x-axis, this throws an error.
     */
    @JvmOverloads
    fun lineToX(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToX(
            posX, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes to x-coordinate [posX].
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the x-axis, this throws an error.
     * Equivalent to [lineToX].
     */
    @JvmOverloads
    fun lineToXConstantHeading(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToXConstantHeading(
            posX, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes to x-coordinate [posX],
     * while changing heading from current to [heading] using linear interpolation.
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the x-axis, this throws an error.
    */
    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToXLinearHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes to x-coordinate [posX],
     * while changing heading from current to [heading] using linear interpolation.
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the x-axis, this throws an error.
     */
    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToXLinearHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes to x-coordinate [posX],
     * while changing heading from current to [heading] using spline interpolation.
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the x-axis, this throws an error.
     */
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToXSplineHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes to x-coordinate [posX],
     * while changing heading from current to [heading] using spline interpolation.
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the x-axis, this throws an error.
     */
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToXSplineHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes to y-coordinate [posY].
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the y-axis, this throws an error.
     */
    @JvmOverloads
    fun lineToY(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToY(
            posY, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes to y-coordinate [posY].
     * * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the y-axis, this throws an error.
     * Equivalent to [lineToY].
     */
    @JvmOverloads
    fun lineToYConstantHeading(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToYConstantHeading(
            posY, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes to y-coordinate [posY],
     * while changing heading from current to [heading] using linear interpolation.
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the y-axis, this throws an error.
     */
    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToYLinearHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes to y-coordinate [posY],
     * while changing heading from current to [heading] using linear interpolation.
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the y-axis, this throws an error.
     */
    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToYLinearHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes to y-coordinate [posY],
     * while changing heading from current to [heading] using spline interpolation.
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the y-axis, this throws an error.
     */
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToYSplineHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes to y-coordinate [posY],
     * while changing heading from current to [heading] using spline interpolation.
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the y-axis, this throws an error.
     */
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.lineToYSplineHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes to [pos].
     */
    @JvmOverloads
    fun strafeTo(
        pos: Vector2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.strafeTo(
            pos, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes to [pos].
     * Equivalent to [strafeTo].
     */
    @JvmOverloads
    fun strafeToConstantHeading(
        pos: Vector2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.strafeToConstantHeading(
            pos, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes to [pos],
     * changing heading from current to [heading] using linear interpolation.
     */
    @JvmOverloads
    fun strafeToLinearHeading(
        pos: Vector2d,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.strafeToLinearHeading(
            pos, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes to [pos],
     * changing heading from current to [heading] using linear interpolation.
     */
    @JvmOverloads
    fun strafeToLinearHeading(
        pos: Vector2d,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.strafeToLinearHeading(
            pos, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes to [pos],
     * changing heading from current to [heading] using spline interpolation.
     */
    @JvmOverloads
    fun strafeToSplineHeading(
        pos: Vector2d,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.strafeToSplineHeading(
            pos, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a line segment that goes to [pos],
     * changing heading from current to [heading] using spline interpolation.
     */
    @JvmOverloads
    fun strafeToSplineHeading(
        pos: Vector2d,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.strafeToSplineHeading(
            pos, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pos] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending [pos] and [tangent].
     */
    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.splineTo(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pos] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending [pos] and [tangent].
     */
    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.splineTo(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pos] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending [pos] and [tangent].
     * The robot's heading remains constant
     * as opposed to matching the tangent.
     */
    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.splineToConstantHeading(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pos] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending [pos] and [tangent].
     * The robot's heading remains constant
     * as opposed to matching the tangent.
     */
    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.splineToConstantHeading(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pose.position][pose] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending position and [tangent].
     * The robot's heading linearly interpolates from its current heading
     * to [pose.heading][pose].
     */
    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.splineToLinearHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pose.position][pose] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending position and [tangent].
     * The robot's heading linearly interpolates from its current heading
     * to [pose.heading][pose].
     */
    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.splineToLinearHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pose.position][pose] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending position and [tangent].
     * The robot's heading interpolates from its current heading
     * to [pose.heading][pose] using spline interpolation.
     */
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.splineToSplineHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pose.position][pose] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending position and [tangent].
     * The robot's heading interpolates from its current heading
     * to [pose.heading][pose] using spline interpolation.
     */
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.splineToSplineHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a curved path using a Bezier curve,
     * with the current pose of this builder
     * as the first control point and [controlPoints]
     * as the remaining control points.
     * The robot's heading follows the path tangent.
     */
    @JvmOverloads
    fun bezierTo(
        controlPoints: List<Vector2d>,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.bezierTo(controlPoints, velConstraintOverride, accelConstraintOverride),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a curved path using a Bezier curve,
     * with the current pose of this builder
     * as the first control point and [controlPoints]
     * as the remaining control points.
     */
    fun bezierTo(
        vararg controlPoints: Vector2d,
    ) = TrajectoryActionBuilder(
        this,
        tb.bezierTo(*controlPoints),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a curved path using a Bezier curve,
     * with the current pose of this builder
     * as the first control point and [controlPoints]
     * as the remaining control points.
     * The robot's heading remains constant at its current heading.
     */
    @JvmOverloads
    fun bezierToConstantHeading(
        controlPoints: List<Vector2d>,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.bezierToConstantHeading(controlPoints, velConstraintOverride, accelConstraintOverride),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a curved path using a Bezier curve,
     * with the current pose of this builder
     * as the first control point and [controlPoints]
     * as the remaining control points.
     * The robot's heading is linearly interpolated
     * from its current heading to [heading].
     */
    @JvmOverloads
    fun bezierToLinearHeading(
        controlPoints: List<Vector2d>,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.bezierToLinearHeading(controlPoints, heading, velConstraintOverride, accelConstraintOverride),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a curved path using a Bezier curve,
     * with the current pose of this builder
     * as the first control point and [controlPoints]
     * as the remaining control points.
     * The robot's heading is linearly interpolated
     * from its current heading to [heading].
     */
    @JvmOverloads
    fun bezierToLinearHeading(
        controlPoints: List<Vector2d>,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.bezierToLinearHeading(controlPoints, heading, velConstraintOverride, accelConstraintOverride),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a curved path using a Bezier curve,
     * with the current pose of this builder
     * as the first control point and [controlPoints]
     * as the remaining control points.
     * The robot's heading is interpolated from
     * its current heading to [heading] using a spline.
     */
    @JvmOverloads
    fun bezierToSplineHeading(
        controlPoints: List<Vector2d>,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.bezierToSplineHeading(controlPoints, heading, velConstraintOverride, accelConstraintOverride),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Adds a curved path using a Bezier curve,
     * with the current pose of this builder
     * as the first control point and [controlPoints]
     * as the remaining control points.
     * The robot's heading is interpolated from
     * its current heading to [heading] using a spline.
     */
    @JvmOverloads
    fun bezierToSplineHeading(
        controlPoints: List<Vector2d>,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryActionBuilder(
        this,
        tb.bezierToSplineHeading(controlPoints, heading, velConstraintOverride, accelConstraintOverride),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Creates a new builder with the same settings at the current pose, tangent.
     */
    fun fresh() = endTrajectory().let {
        TrajectoryActionBuilder(
            it.turnActionFactory,
            it.trajectoryActionFactory,
            it.trajectoryBuilderParams,
            it.lastPoseUnmapped,
            it.beginEndVel, it.baseTurnConstraints, it.baseVelConstraint, it.baseAccelConstraint,
            it.poseMap
        ).setTangent(it.lastTangent)
    }

    /**
     * Returns an Action object that follows the full sequence.
     */
    fun build(): Action {
        return endTrajectory().cont(NullAction())
    }
}