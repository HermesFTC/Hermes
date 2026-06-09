/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.wpi.trajectories

import gay.zharel.hermes.profiles.ProfileParams
import gay.zharel.hermes.trajectories.TrajectoryBuilderParams

val DEFAULT_PROFILE_PARAMS = ProfileParams(0.25, Math.PI / 8, 1e-4)
val DEFAULT_BUILDER_PARAMS = TrajectoryBuilderParams(1e-6, DEFAULT_PROFILE_PARAMS)
