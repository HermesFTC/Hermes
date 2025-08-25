package com.acmerobotics.roadrunner.profiles

import com.acmerobotics.roadrunner.geometry.RobotState
import com.acmerobotics.roadrunner.geometry.range
import com.acmerobotics.roadrunner.geometry.rangeCentered
import com.acmerobotics.roadrunner.paths.PosePath
import kotlin.math.ceil
import kotlin.math.max

/**
 * Functions for generating displacement profiles from path constraints.
 * These handle sampling the path and evaluating velocity/acceleration constraints.
 */

/**
 * Creates a complete displacement profile by generating forward and backward passes
 * and merging them for optimal performance.
 */
fun createOptimalDisplacementProfile(
    params: ProfileParams,
    path: PosePath,
    beginVel: Double,
    velConstraint: VelConstraint,
    accelConstraint: AccelConstraint,
): DisplacementProfile {
    val forwardProfile = generatePathBasedForwardProfile(params, path, beginVel, velConstraint, accelConstraint)
    val backwardProfile = generatePathBasedBackwardProfile(params, path, beginVel, velConstraint, accelConstraint)
    return mergeDisplacementProfiles(forwardProfile, backwardProfile)
}

/**
 * Generates a forward profile based on path constraints by sampling the path
 * and evaluating constraints at each sample point.
 */
fun generatePathBasedForwardProfile(
    params: ProfileParams,
    path: PosePath,
    beginVel: Double,
    velConstraint: VelConstraint,
    accelConstraint: AccelConstraint,
): DisplacementProfile {
    val samples = generatePathSamples(path, params)
    val (maxVels, maxAccels) = evaluateConstraintsAtSamples(path, samples, velConstraint, accelConstraint)
    val displacements = createDisplacementPoints(samples, path.length())

    return generateForwardProfile(displacements, beginVel, maxVels, maxAccels)
}

/**
 * Generates a backward profile based on path constraints.
 */
fun generatePathBasedBackwardProfile(
    params: ProfileParams,
    path: PosePath,
    endVel: Double,
    velConstraint: VelConstraint,
    accelConstraint: AccelConstraint,
): DisplacementProfile {
    val samples = generatePathSamples(path, params)
    val (maxVels, minAccels) = evaluateBackwardConstraintsAtSamples(path, samples, velConstraint, accelConstraint)
    val displacements = createDisplacementPoints(samples, path.length())

    return generateBackwardProfile(displacements, maxVels, endVel, minAccels)
}

/**
 * Generates sample points along the path based on displacement and angular resolution.
 * Combines uniform displacement sampling with adaptive angular sampling.
 */
fun generatePathSamples(path: PosePath, params: ProfileParams): List<Double> {
    val pathLength = path.length()

    // Generate displacement-based samples
    val dispSamples = rangeCentered(
        0.0,
        pathLength,
        max(1, ceil(pathLength / params.dispResolution).toInt())
    )

    // Generate angle-based samples for curves
    val angSamples = samplePathByRotation(path, params.angResolution, params.angSamplingEps)

    // Combine and sort all samples
    return (dispSamples + angSamples).sorted()
}

/**
 * Evaluates velocity and acceleration constraints at the given sample points.
 * Returns maximum velocities and accelerations for forward profile generation.
 */
fun evaluateConstraintsAtSamples(
    path: PosePath,
    samples: List<Double>,
    velConstraint: VelConstraint,
    accelConstraint: AccelConstraint,
): Pair<List<Double>, List<Double>> {
    val maxVels = mutableListOf<Double>()
    val maxAccels = mutableListOf<Double>()

    for (s in samples) {
        val pose = path[s, 2]
        val robotState = RobotState.fromDualPose(pose)

        maxVels.add(velConstraint.maxRobotVel(robotState, path, s))

        val (_, maxAccel) = accelConstraint.minMaxProfileAccel(robotState, path, s)
        maxAccels.add(maxAccel)
    }

    return Pair(maxVels, maxAccels)
}

/**
 * Evaluates constraints for backward profile generation.
 * Returns maximum velocities and minimum accelerations.
 */
fun evaluateBackwardConstraintsAtSamples(
    path: PosePath,
    samples: List<Double>,
    velConstraint: VelConstraint,
    accelConstraint: AccelConstraint,
): Pair<List<Double>, List<Double>> {
    val maxVels = mutableListOf<Double>()
    val minAccels = mutableListOf<Double>()

    for (s in samples) {
        val pose = path[s, 2]
        val robotState = RobotState.fromDualPose(pose)

        maxVels.add(velConstraint.maxRobotVel(robotState, path, s))

        val (minAccel, _) = accelConstraint.minMaxProfileAccel(robotState, path, s)
        minAccels.add(minAccel)
    }

    return Pair(maxVels, minAccels)
}

/**
 * Creates displacement interval endpoints from sample points.
 * Adds midpoints between samples to ensure proper interval representation.
 */
fun createDisplacementPoints(samples: List<Double>, pathLength: Double): List<Double> {
    val midpoints = samples.zip(samples.drop(1)).map { (a, b) -> 0.5 * (a + b) }
    return listOf(0.0) + midpoints + listOf(pathLength)
}

/**
 * Generates a simple displacement profile with uniform sampling and constant constraints.
 */
fun createSimpleDisplacementProfile(
    length: Double,
    beginVel: Double,
    maxVel: (Double) -> Double,
    maxAccel: (Double) -> Double,
    resolution: Double,
): DisplacementProfile {
    require(length > 0.0) { "length ($length) must be positive" }
    require(resolution > 0.0) { "resolution ($resolution) must be positive" }
    require(beginVel >= 0.0) { "beginVel ($beginVel) must be non-negative" }

    val samples = max(1, ceil(length / resolution).toInt())
    val disps = rangeCentered(0.0, length, samples)
    val maxVels = disps.map(maxVel)
    val maxAccels = disps.map(maxAccel)

    val forwardProfile = generateForwardProfile(range(0.0, length, samples + 1), beginVel, maxVels, maxAccels)
    val backwardProfile = generateSimpleBackwardProfile(length, maxVel, beginVel, { s -> -maxAccel(s) }, resolution)

    return mergeDisplacementProfiles(forwardProfile, backwardProfile)
}

/**
 * Generates a simple backward profile with uniform sampling.
 */
fun generateSimpleBackwardProfile(
    length: Double,
    maxVel: (Double) -> Double,
    endVel: Double,
    minAccel: (Double) -> Double,
    resolution: Double,
): DisplacementProfile {
    require(endVel >= 0.0) { "endVel ($endVel) must be non-negative" }

    val samples = max(1, ceil(length / resolution).toInt())
    val disps = rangeCentered(0.0, length, samples)
    val maxVels = disps.map(maxVel)
    val minAccels = disps.map(minAccel)

    return generateBackwardProfile(range(0.0, length, samples + 1), maxVels, endVel, minAccels)
}
