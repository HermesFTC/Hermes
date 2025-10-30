/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.paths

import gay.zharel.hermes.Rotation2d
import gay.zharel.hermes.TEST_ACCEL_CONSTRAINT
import gay.zharel.hermes.TEST_PROFILE_PARAMS
import gay.zharel.hermes.TEST_TRAJECTORY_BUILDER_PARAMS
import gay.zharel.hermes.TEST_VEL_CONSTRAINT
import gay.zharel.hermes.Vector2d
import gay.zharel.hermes.dualEqual
import gay.zharel.hermes.dualPoseEqual
import gay.zharel.hermes.geometry.Rotation2d
import gay.zharel.hermes.geometry.Vector2d
import gay.zharel.hermes.math.range
import gay.zharel.hermes.posPathSeqBuilder
import gay.zharel.hermes.profiles.generatePathBasedForwardProfile
import gay.zharel.hermes.randomAngle
import gay.zharel.hermes.randomPoint
import gay.zharel.hermes.trajectories.PositionPathSeqBuilder
import io.kotest.core.spec.style.FunSpec
import io.kotest.matchers.should
import io.kotest.matchers.shouldBe
import io.kotest.property.Arb
import io.kotest.property.arbitrary.arbitrary
import io.kotest.property.arbitrary.bind
import io.kotest.property.checkAll

data class SplineTarget(val pos: Vector2d, val tangent: Rotation2d)

val targetArb = Arb.bind(Arb.Vector2d(), Arb.Rotation2d(), ::SplineTarget)

val random3SplinePath = arbitrary { rs ->
    PositionPathSeqBuilder(
        Vector2d(0.0, 0.0),
        0.0,
        TEST_TRAJECTORY_BUILDER_PARAMS.arcLengthSamplingEps
    ).splineTo(rs.random.randomPoint(), rs.random.randomAngle())
        .splineTo(rs.random.randomPoint(), rs.random.randomAngle())
        .splineTo(rs.random.randomPoint(), rs.random.randomAngle())
}

fun PositionPathSeqBuilder.buildToComposite() =
    CompositePosePath(build().map { TangentPath(it, 0.0 ) })

class ConsistencyTest : FunSpec(
    {
        context("given the same inputs") {
            test("multiple path builds are the same") {
                checkAll(targetArb, targetArb, targetArb) { tA, tB, tC ->
                    val path1 = posPathSeqBuilder()
                        .splineTo(tA.pos, tA.tangent)
                        .splineTo(tB.pos, tB.tangent)
                        .splineTo(tC.pos, tC.tangent)
                        .buildToComposite()

                    val path2 = posPathSeqBuilder()
                        .splineTo(tA.pos, tA.tangent)
                        .splineTo(tB.pos, tB.tangent)
                        .splineTo(tC.pos, tC.tangent)
                        .buildToComposite()

                    path1.length() shouldBe path2.length()

                    val samples = range(0.0, path1.length(), 100)

                    samples.forEach {
                        path1[it, 3] should dualPoseEqual(path2[it, 3])
                    }
                }
            }


            test("multiple profile builds are the same") {
                checkAll(targetArb, targetArb, targetArb) { tA, tB, tC ->
                    val profile1 = generatePathBasedForwardProfile(
                        TEST_PROFILE_PARAMS,
                        posPathSeqBuilder()
                            .splineTo(tA.pos, tA.tangent)
                            .splineTo(tB.pos, tB.tangent)
                            .splineTo(tC.pos, tC.tangent)
                            .buildToComposite(),
                        0.0,
                        TEST_VEL_CONSTRAINT,
                        TEST_ACCEL_CONSTRAINT
                    )

                    val profile2 = generatePathBasedForwardProfile(
                        TEST_PROFILE_PARAMS,
                        posPathSeqBuilder()
                            .splineTo(tA.pos, tA.tangent)
                            .splineTo(tB.pos, tB.tangent)
                            .splineTo(tC.pos, tC.tangent)
                            .buildToComposite(),
                        0.0,
                        TEST_VEL_CONSTRAINT,
                        TEST_ACCEL_CONSTRAINT
                    )

                    profile1.length shouldBe profile2.length

                    val samples = range(0.0, profile1.length, 100)

                    samples.forEach {
                        profile1[it] should dualEqual(profile2[it])
                    }
                }
            }

            test("profile generation is deterministic") {
                checkAll(random3SplinePath) { pathBuilder ->
                    val path = pathBuilder.buildToComposite()

                    val profile1 = generatePathBasedForwardProfile(TEST_PROFILE_PARAMS, path, 0.0, TEST_VEL_CONSTRAINT, TEST_ACCEL_CONSTRAINT)
                    val profile2 = generatePathBasedForwardProfile(TEST_PROFILE_PARAMS, path, 0.0, TEST_VEL_CONSTRAINT, TEST_ACCEL_CONSTRAINT)

                    profile1.length shouldBe profile2.length

                    val samples = range(0.0, profile1.length, 100)

                    samples.forEach {
                        profile1[it] should dualEqual(profile2[it])
                    }
                }
            }

            test("trajectories are deterministic") {
                checkAll(random3SplinePath) { pathBuilder ->
                    val path = pathBuilder.buildToComposite()

                    val profile1 = generatePathBasedForwardProfile(
                        TEST_PROFILE_PARAMS,
                        path,
                        0.0,
                        TEST_VEL_CONSTRAINT,
                        TEST_ACCEL_CONSTRAINT
                    )

                    val profile2 = generatePathBasedForwardProfile(
                        TEST_PROFILE_PARAMS,
                        path,
                        0.0,
                        TEST_VEL_CONSTRAINT,
                        TEST_ACCEL_CONSTRAINT
                    )

                    profile1.length shouldBe profile2.length

                    val samples = range(0.0, profile1.length, 100)

                    samples.forEach {
                        profile1[it] should dualEqual(profile2[it])
                    }
                }
            }
        }
    }
)