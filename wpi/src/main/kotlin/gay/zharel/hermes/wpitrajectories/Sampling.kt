/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.wpitrajectories

import edu.wpi.first.math.trajectory.TrajectorySample
import edu.wpi.first.units.measure.Time
import gay.zharel.hermes.geometry.RobotState
import gay.zharel.hermes.profiles.ProfileParams
import gay.zharel.hermes.trajectories.TimeTrajectory
import gay.zharel.hermes.trajectories.TrajectoryBuilderParams
import gay.zharel.hermes.wpiconversions.HTrajectory
import gay.zharel.hermes.wpiconversions.seconds
import gay.zharel.hermes.wpiconversions.wpilib
import kotlin.time.Duration

val DEFAULT_PROFILE_PARAMS = ProfileParams(0.25, Math.PI / 8, 1e-4,)
val DEFAULT_BUILDER_PARAMS = TrajectoryBuilderParams(1e-6, DEFAULT_PROFILE_PARAMS)

fun RobotState.asSample(timestamp: Time) = TrajectorySample.Base(
    timestamp, pose.wpilib, vel.wpilib, accel.wpilib,
)

fun TimeTrajectory.sample() = profile.times.map {
    this[it].asSample(it.seconds)
}