/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

@file:OptIn(InternalSerializationApi::class)
@file:JvmName("Serialization")

package gay.zharel.hermes.serialization

import gay.zharel.hermes.math.Arclength
import gay.zharel.hermes.paths.*
import gay.zharel.hermes.profiles.CancelableProfile
import gay.zharel.hermes.profiles.DisplacementProfile
import gay.zharel.hermes.profiles.Profile
import gay.zharel.hermes.profiles.TimeProfile
import gay.zharel.hermes.trajectories.*
import kotlinx.serialization.InternalSerializationApi
import kotlinx.serialization.json.Json
import kotlinx.serialization.modules.SerializersModule
import kotlinx.serialization.modules.polymorphic
import kotlinx.serialization.modules.subclass
import kotlinx.serialization.serializer

private val module = SerializersModule {
    polymorphic(PositionPath::class) {
        subclass(Line::class)
        subclass(ArclengthReparamCurve2d::class)
        subclass(serializer<CompositePositionPath<Arclength>>())
        subclass(QuinticSpline2dInternal::class)
        subclass(BezierCurve2dInternal::class)
        subclass(serializer<PositionPathView<Arclength>>())
    }

    polymorphic(PoseMap::class) {
        subclass(IdentityPoseMap::class)
    }

    polymorphic(PosePath::class) {
        subclass(TangentPath::class)
        subclass(HeadingPosePath::class)
        subclass(CompositePosePath::class)
        subclass(MappedPosePath::class)
    }

    polymorphic(Profile::class) {
        subclass(DisplacementProfile::class)
        subclass(TimeProfile::class)
        subclass(CancelableProfile::class)
    }

    polymorphic(MarkerTrigger::class) {
        subclass(AfterDispTrigger::class)
        subclass(AfterTimeTrigger::class)
        subclass(AtPointTrigger::class)
        subclass(AtPoseTrigger::class)
    }

    polymorphic(Trajectory::class) {
        subclass(DisplacementTrajectory::class)
        subclass(TimeTrajectory::class)
        subclass(CancelableTrajectory::class)
        subclass(CompositeTrajectory::class)
        subclass(CompositeCancelableTrajectory::class)
        subclass(serializer<TrajectoryWithMarkers<Arclength>>())
    }
}

val HermesJsonFormat = Json {
    serializersModule = module
    prettyPrint = true
    ignoreUnknownKeys = true
}