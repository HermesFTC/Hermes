@file:OptIn(InternalSerializationApi::class)
@file:JvmName("Serialization")

package com.acmerobotics.roadrunner.serialization

import com.acmerobotics.roadrunner.geometry.Arclength
import com.acmerobotics.roadrunner.paths.*
import com.acmerobotics.roadrunner.profiles.CancelableProfile
import com.acmerobotics.roadrunner.profiles.DisplacementProfile
import com.acmerobotics.roadrunner.profiles.Profile
import com.acmerobotics.roadrunner.profiles.TimeProfile
import com.acmerobotics.roadrunner.trajectories.*
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