package gay.zharel.hermes

import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.math.DualParameter
import gay.zharel.hermes.geometry.Pose2d
import gay.zharel.hermes.geometry.Rotation2d
import gay.zharel.hermes.geometry.Vector2d
import io.kotest.property.Arb
import kotlin.random.Random
import io.kotest.property.arbitrary.arbitrary
import io.kotest.property.arbitrary.bind

fun <Param : DualParameter> Arb.Companion.DualNum(n: Int) = arbitrary { rs ->
    DualNum<Param>(List(n) { rs.random.nextDouble() })
}

fun Random.randomPoint(): Vector2d = Vector2d(
    nextDouble(-72.0, 72.0),
    nextDouble(-72.0, 72.0)
)

fun randomPoint() = Random.Default.randomPoint()

private val randomPointGen = arbitrary { rs -> rs.random.randomPoint() }

fun Arb.Companion.Vector2d() = randomPointGen

fun Random.randomAngle(): Rotation2d =
    Rotation2d.exp(nextDouble(-Math.PI, Math.PI))

fun randomAngle() = Random.Default.randomAngle()

private val randomAngleGen = arbitrary { rs -> rs.random.randomAngle() }

fun Arb.Companion.Rotation2d() = randomAngleGen


fun Random.randomPose(): Pose2d = Pose2d(randomPoint(), randomAngle())

fun randomPose() = Random.Default.randomPose()

private val randomPoseGen = Arb.bind(
    randomPointGen,
    randomAngleGen
) {
    pos, heading -> Pose2d(pos, heading)
}
fun Arb.Companion.Pose2d() = randomPoseGen