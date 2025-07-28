package me.zharel.hermes.talaria

import com.acmerobotics.roadrunner.geometry.Arclength
import com.acmerobotics.roadrunner.geometry.Pose2dDual
import com.acmerobotics.roadrunner.paths.PosePath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.measure.Distance

interface Path {
    val length: Distance

    operator fun get(s: Distance): Pose2d
}

class PathWrapper internal constructor(internal val posepath: PosePath) : Path {
    override val length = posepath.length().inches()

    override fun get(s: Distance): Pose2d = posepath[s.inchValue(), 1].value().wpilib()
}

internal fun Path.asPosePath() = object : PosePath {
    override fun get(s: Double, n: Int, ): Pose2dDual<Arclength> =
        Pose2dDual.constant(this@asPosePath[s.inches()].hermes(), 1)

    override fun length(): Double = this@asPosePath.length.inchValue()
}
