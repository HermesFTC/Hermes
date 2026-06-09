/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.trajectories

import gay.zharel.hermes.TEST_ACCEL_CONSTRAINT
import gay.zharel.hermes.TEST_PROFILE_PARAMS
import gay.zharel.hermes.TEST_TRAJECTORY_BUILDER_PARAMS
import gay.zharel.hermes.TEST_VEL_CONSTRAINT
import gay.zharel.hermes.geometry.Pose2d
import gay.zharel.hermes.geometry.Vector2d
import gay.zharel.hermes.paths.TangentPath
import gay.zharel.hermes.profiles.CancelableProfile
import gay.zharel.hermes.profiles.DisplacementProfile
import gay.zharel.hermes.profiles.TimeProfile
import gay.zharel.hermes.profiles.samplePathByRotation
import kotlin.test.Test
import kotlin.time.Duration
import kotlin.time.DurationUnit
import kotlin.time.measureTimedValue

/** Number of warmup iterations to run before timing measurements */
private const val WARMUP_ITERATIONS = 5

/** Number of measured iterations for averaging */
private const val MEASURED_ITERATIONS = 10

/**
 * Runs warmup iterations followed by measured iterations, returning timing statistics.
 *
 * @param warmupIterations Number of warmup iterations (not timed)
 * @param measuredIterations Number of iterations to measure
 * @param block The code block to time
 * @return TimingResult containing statistics and individual times
 */
private inline fun <T> measureWithWarmup(
    warmupIterations: Int = WARMUP_ITERATIONS,
    measuredIterations: Int = MEASURED_ITERATIONS,
    block: () -> T
): TimingResult<T> {
    // Warmup iterations - run but don't measure
    repeat(warmupIterations) {
        block()
    }

    // Measured iterations
    val times = mutableListOf<Duration>()
    var lastResult: T? = null
    repeat(measuredIterations) {
        val result = measureTimedValue { block() }
        times.add(result.duration)
        lastResult = result.value
    }

    return TimingResult(times, lastResult!!)
}

/**
 * Result of timing measurements with statistics.
 */
private data class TimingResult<T>(
    val times: List<Duration>,
    val lastValue: T
) {
    val average: Duration get() = times.reduce { acc, d -> acc + d } / times.size
    val min: Duration get() = times.minOrNull() ?: Duration.ZERO
    val max: Duration get() = times.maxOrNull() ?: Duration.ZERO
    val stdDev: Double get() {
        val avgMs = average.toDouble(DurationUnit.MILLISECONDS)
        val variance = times.map {
            val diff = it.toDouble(DurationUnit.MILLISECONDS) - avgMs
            diff * diff
        }.average()
        return kotlin.math.sqrt(variance)
    }

    fun printStats(label: String = "Timing") {
        println("$label (n=${ times.size }):")
        println("  Average: ${average.toString(DurationUnit.MILLISECONDS, 3)}")
        println("  Min: ${min.toString(DurationUnit.MILLISECONDS, 3)}")
        println("  Max: ${max.toString(DurationUnit.MILLISECONDS, 3)}")
        println("  Std Dev: %.3f ms".format(stdDev))
    }
}

/**
 * Comprehensive timing tests for trajectory generation.
 *
 * The trajectory generation process consists of several steps:
 * 1. Position Path Building - Building position path sequences (splines, lines, beziers)
 * 2. Pose Path Building - Adding heading interpolation to position paths
 * 3. Profile Generation - Generating motion profiles with velocity/acceleration constraints
 * 4. Full Trajectory Building - The entire build process using TrajectoryBuilder
 *
 * All tests include warmup iterations to account for JVM JIT compilation and class loading.
 */
class TrajectoryTimingTests {

    /**
     * Measures the time taken for each step of the trajectory generation process
     * using a typical trajectory with multiple segments.
     * Includes warmup iterations for accurate measurements.
     */
    @Test
    fun `measure individual step timing`() {
        println("=== Trajectory Generation Timing Analysis ===")
        println("(Warmup: $WARMUP_ITERATIONS iterations, Measured: $MEASURED_ITERATIONS iterations)\n")

        // Step 1: Position Path Building
        val positionPathResult = measureWithWarmup {
            PositionPathSeqBuilder(
                Vector2d(0.0, 0.0),
                0.0,
                TEST_TRAJECTORY_BUILDER_PARAMS.arcLengthSamplingEps
            )
                .splineTo(Vector2d(30.0, 30.0), 0.0)
                .splineTo(Vector2d(60.0, 0.0), -Math.PI / 2)
                .forward(20.0)
                .splineTo(Vector2d(30.0, -30.0), Math.PI)
                .build()
        }
        val positionPaths = positionPathResult.lastValue
        println("Step 1: Position Path Building")
        positionPathResult.printStats("  ")
        println("  Segments: ${positionPaths.sumOf { it.paths.size }}")
        println("  Total length: ${"%.2f".format(positionPaths.sumOf { it.length() })}")
        println()

        // Step 2: Pose Path Building (adding heading interpolation)
        val posePathResult = measureWithWarmup {
            positionPaths.flatMap { posPath ->
                var builder = PosePathSeqBuilder(posPath, 0.0)
                for (offset in posPath.offsets.drop(1)) {
                    builder = builder.tangentUntil(offset)
                }
                builder.build()
            }
        }
        val posePaths = posePathResult.lastValue
        println("Step 2: Pose Path Building (Heading Interpolation)")
        posePathResult.printStats("  ")
        println("  Pose paths: ${posePaths.size}")
        println()

        // Step 3a: Angular Sampling
        val angularSamplingResult = measureWithWarmup {
            posePaths.map { posePath ->
                samplePathByRotation(posePath, TEST_PROFILE_PARAMS.angResolution, TEST_PROFILE_PARAMS.angSamplingEps)
            }
        }
        println("Step 3a: Angular Sampling")
        angularSamplingResult.printStats("  ")
        println("  Samples per path: ${angularSamplingResult.lastValue.map { it.size }}")
        println()

        // Step 3b: Profile Generation
        val profileResult = measureWithWarmup {
            posePaths.map { posePath ->
                CancelableProfile.generate(
                    TEST_PROFILE_PARAMS,
                    posePath,
                    0.0,
                    TEST_VEL_CONSTRAINT,
                    TEST_ACCEL_CONSTRAINT
                )
            }
        }
        println("Step 3b: Full Profile Generation")
        profileResult.printStats("  ")
        println("  Profile segments: ${profileResult.lastValue.map { it.disps.size }}")
        println()

        // Total time calculation
        val totalAvg = positionPathResult.average + posePathResult.average + profileResult.average
        println("=== Summary (averages) ===")
        println("  Position Path Building: ${positionPathResult.average.toString(DurationUnit.MILLISECONDS, 3)}")
        println("  Pose Path Building:     ${posePathResult.average.toString(DurationUnit.MILLISECONDS, 3)}")
        println("  Profile Generation:     ${profileResult.average.toString(DurationUnit.MILLISECONDS, 3)}")
        println("  Total:                  ${totalAvg.toString(DurationUnit.MILLISECONDS, 3)}")
        println()
    }

    /**
     * Measures the time taken for the full TrajectoryBuilder.build() process.
     * Includes warmup to ensure JIT compilation has occurred.
     */
    @Test
    fun `measure full trajectory builder timing`() {
        println("=== Full TrajectoryBuilder Timing ===")
        println("(Warmup: $WARMUP_ITERATIONS iterations, Measured: $MEASURED_ITERATIONS iterations)\n")

        // Measure builder setup
        val builderResult = measureWithWarmup {
            TrajectoryBuilder(
                TEST_TRAJECTORY_BUILDER_PARAMS,
                Pose2d(0.0, 0.0, 0.0),
                0.0,
                TEST_VEL_CONSTRAINT,
                TEST_ACCEL_CONSTRAINT
            )
                .splineTo(Vector2d(30.0, 30.0), 0.0)
                .splineTo(Vector2d(60.0, 0.0), -Math.PI / 2)
                .forward(20.0)
                .splineTo(Vector2d(30.0, -30.0), Math.PI)
        }
        println("Builder Setup:")
        builderResult.printStats("  ")
        println()

        // Measure build() call
        val buildResult = measureWithWarmup {
            builderResult.lastValue.build()
        }
        println("Build Call:")
        buildResult.printStats("  ")
        println()

        val trajectoryWithMarkers = buildResult.lastValue
        val compositeTrajectory = trajectoryWithMarkers.trajectory as CompositeCancelableTrajectory

        println("Total Average: ${(builderResult.average + buildResult.average).toString(DurationUnit.MILLISECONDS, 3)}")
        println()
        println("Trajectory Info:")
        println("  Segments: ${compositeTrajectory.trajectories.size}")
        println("  Markers: ${trajectoryWithMarkers.markers.size}")
        println()
    }

    /**
     * Compares timing for different path segment types.
     * Includes warmup to ensure accurate comparisons.
     */
    @Test
    fun `compare segment type timing`() {
        println("=== Segment Type Timing Comparison ===")
        println("(Warmup: $WARMUP_ITERATIONS iterations, Measured: $MEASURED_ITERATIONS iterations)\n")

        // Line segment
        val lineResult = measureWithWarmup {
            TrajectoryBuilder(
                TEST_TRAJECTORY_BUILDER_PARAMS,
                Pose2d(0.0, 0.0, 0.0),
                0.0,
                TEST_VEL_CONSTRAINT,
                TEST_ACCEL_CONSTRAINT
            )
                .forward(50.0)
                .build()
        }
        println("Line (50 units):")
        lineResult.printStats("  ")
        println()

        // Spline segment
        val splineResult = measureWithWarmup {
            TrajectoryBuilder(
                TEST_TRAJECTORY_BUILDER_PARAMS,
                Pose2d(0.0, 0.0, 0.0),
                0.0,
                TEST_VEL_CONSTRAINT,
                TEST_ACCEL_CONSTRAINT
            )
                .splineTo(Vector2d(50.0, 0.0), 0.0)
                .build()
        }
        println("Spline (50 units approx):")
        splineResult.printStats("  ")
        println()

        // Complex spline segment
        val complexSplineResult = measureWithWarmup {
            TrajectoryBuilder(
                TEST_TRAJECTORY_BUILDER_PARAMS,
                Pose2d(0.0, 0.0, 0.0),
                0.0,
                TEST_VEL_CONSTRAINT,
                TEST_ACCEL_CONSTRAINT
            )
                .splineTo(Vector2d(30.0, 30.0), Math.PI / 2)
                .build()
        }
        println("Complex Spline (curved):")
        complexSplineResult.printStats("  ")
        println()

        // Bezier curve
        val bezierResult = measureWithWarmup {
            TrajectoryBuilder(
                TEST_TRAJECTORY_BUILDER_PARAMS,
                Pose2d(0.0, 0.0, 0.0),
                0.0,
                TEST_VEL_CONSTRAINT,
                TEST_ACCEL_CONSTRAINT
            )
                .bezierTo(
                    listOf(
                        Vector2d(20.0, 10.0),
                        Vector2d(40.0, -10.0),
                        Vector2d(50.0, 0.0)
                    )
                )
                .build()
        }
        println("Bezier (4 control points):")
        bezierResult.printStats("  ")
        println()

        // Summary comparison
        println("=== Summary (averages) ===")
        println("  Line:           ${lineResult.average.toString(DurationUnit.MILLISECONDS, 3)}")
        println("  Spline:         ${splineResult.average.toString(DurationUnit.MILLISECONDS, 3)}")
        println("  Complex Spline: ${complexSplineResult.average.toString(DurationUnit.MILLISECONDS, 3)}")
        println("  Bezier:         ${bezierResult.average.toString(DurationUnit.MILLISECONDS, 3)}")
        println()
    }

    /**
     * Compares timing for different heading interpolation types.
     * Includes warmup to ensure accurate comparisons.
     */
    @Test
    fun `compare heading interpolation timing`() {
        println("=== Heading Interpolation Timing Comparison ===")
        println("(Warmup: $WARMUP_ITERATIONS iterations, Measured: $MEASURED_ITERATIONS iterations)\n")

        // Tangent heading (default)
        val tangentResult = measureWithWarmup {
            TrajectoryBuilder(
                TEST_TRAJECTORY_BUILDER_PARAMS,
                Pose2d(0.0, 0.0, 0.0),
                0.0,
                TEST_VEL_CONSTRAINT,
                TEST_ACCEL_CONSTRAINT
            )
                .splineTo(Vector2d(30.0, 30.0), 0.0)
                .build()
        }
        println("Tangent Heading:")
        tangentResult.printStats("  ")
        println()

        // Constant heading
        val constantResult = measureWithWarmup {
            TrajectoryBuilder(
                TEST_TRAJECTORY_BUILDER_PARAMS,
                Pose2d(0.0, 0.0, 0.0),
                0.0,
                TEST_VEL_CONSTRAINT,
                TEST_ACCEL_CONSTRAINT
            )
                .splineToConstantHeading(Vector2d(30.0, 30.0), 0.0)
                .build()
        }
        println("Constant Heading:")
        constantResult.printStats("  ")
        println()

        // Linear heading
        val linearResult = measureWithWarmup {
            TrajectoryBuilder(
                TEST_TRAJECTORY_BUILDER_PARAMS,
                Pose2d(0.0, 0.0, 0.0),
                0.0,
                TEST_VEL_CONSTRAINT,
                TEST_ACCEL_CONSTRAINT
            )
                .splineToLinearHeading(Pose2d(30.0, 30.0, Math.PI / 2), 0.0)
                .build()
        }
        println("Linear Heading:")
        linearResult.printStats("  ")
        println()

        // Spline heading
        val splineHeadingResult = measureWithWarmup {
            TrajectoryBuilder(
                TEST_TRAJECTORY_BUILDER_PARAMS,
                Pose2d(0.0, 0.0, 0.0),
                0.0,
                TEST_VEL_CONSTRAINT,
                TEST_ACCEL_CONSTRAINT
            )
                .splineToSplineHeading(Pose2d(30.0, 30.0, Math.PI / 2), 0.0)
                .build()
        }
        println("Spline Heading:")
        splineHeadingResult.printStats("  ")
        println()

        // Summary comparison
        println("=== Summary (averages) ===")
        println("  Tangent:  ${tangentResult.average.toString(DurationUnit.MILLISECONDS, 3)}")
        println("  Constant: ${constantResult.average.toString(DurationUnit.MILLISECONDS, 3)}")
        println("  Linear:   ${linearResult.average.toString(DurationUnit.MILLISECONDS, 3)}")
        println("  Spline:   ${splineHeadingResult.average.toString(DurationUnit.MILLISECONDS, 3)}")
        println()
    }

    /**
     * Measures timing scaling with trajectory complexity (number of segments).
     * Includes warmup for each complexity level.
     */
    @Test
    fun `measure timing scaling with complexity`() {
        println("=== Timing Scaling with Complexity ===")
        println("(Warmup: $WARMUP_ITERATIONS iterations, Measured: $MEASURED_ITERATIONS iterations per complexity level)\n")

        val results = mutableMapOf<Int, TimingResult<*>>()

        for (numSegments in listOf(1, 2, 4, 8, 16)) {
            val result = measureWithWarmup {
                var builder = TrajectoryBuilder(
                    TEST_TRAJECTORY_BUILDER_PARAMS,
                    Pose2d(0.0, 0.0, 0.0),
                    0.0,
                    TEST_VEL_CONSTRAINT,
                    TEST_ACCEL_CONSTRAINT
                )

                for (i in 0 until numSegments) {
                    val angle = i * Math.PI / 4
                    val x = 20.0 * (i + 1) * kotlin.math.cos(angle)
                    val y = 20.0 * (i + 1) * kotlin.math.sin(angle)
                    builder = builder.splineTo(Vector2d(x, y), angle)
                }

                builder.build()
            }
            results[numSegments] = result
            println("$numSegments segment(s):")
            result.printStats("  ")
            println()
        }

        // Summary
        println("=== Summary (averages) ===")
        for ((numSegments, result) in results) {
            println("  $numSegments segment(s): ${result.average.toString(DurationUnit.MILLISECONDS, 3)}")
        }
        println()
    }

    /**
     * Runs multiple iterations to demonstrate timing consistency after warmup.
     * Shows raw iteration times to visualize warmup effect if warmup is disabled.
     */
    @Test
    fun `measure timing consistency`() {
        println("=== Timing Consistency Test ===")
        println("(Warmup: $WARMUP_ITERATIONS iterations, Measured: $MEASURED_ITERATIONS iterations)\n")

        val result = measureWithWarmup {
            TrajectoryBuilder(
                TEST_TRAJECTORY_BUILDER_PARAMS,
                Pose2d(0.0, 0.0, 0.0),
                0.0,
                TEST_VEL_CONSTRAINT,
                TEST_ACCEL_CONSTRAINT
            )
                .splineTo(Vector2d(30.0, 30.0), 0.0)
                .splineTo(Vector2d(60.0, 0.0), -Math.PI / 2)
                .forward(20.0)
                .splineTo(Vector2d(30.0, -30.0), Math.PI)
                .build()
        }

        println("Individual iteration times (after warmup):")
        result.times.forEachIndexed { index, duration ->
            println("  Iteration ${index + 1}: ${duration.toString(DurationUnit.MILLISECONDS, 3)}")
        }
        println()

        result.printStats("Statistics")
        println()
    }

    /**
     * Detailed breakdown of profile generation internals.
     * Includes warmup for accurate measurements.
     */
    @Test
    fun `detailed profile generation breakdown`() {
        println("=== Detailed Profile Generation Breakdown ===")
        println("(Warmup: $WARMUP_ITERATIONS iterations, Measured: $MEASURED_ITERATIONS iterations)\n")

        // Build a simple path first (with warmup)
        val posPathResult = measureWithWarmup {
            PositionPathSeqBuilder(
                Vector2d(0.0, 0.0),
                0.0,
                TEST_TRAJECTORY_BUILDER_PARAMS.arcLengthSamplingEps
            )
                .splineTo(Vector2d(30.0, 30.0), 0.0)
                .build()
                .first()
        }
        val posPath = posPathResult.lastValue
        val posePath = TangentPath(posPath, 0.0)

        println("Path Info:")
        println("  Length: ${"%.2f".format(posePath.length())}")
        println("  Path Building: ${posPathResult.average.toString(DurationUnit.MILLISECONDS, 3)} avg")
        println()

        // Measure angular sampling
        val angSamplingResult = measureWithWarmup {
            samplePathByRotation(posePath, TEST_PROFILE_PARAMS.angResolution, TEST_PROFILE_PARAMS.angSamplingEps)
        }
        println("Angular Sampling:")
        angSamplingResult.printStats("  ")
        println("  Samples: ${angSamplingResult.lastValue.size}")
        println()

        // Measure full profile generation
        val profileResult = measureWithWarmup {
            CancelableProfile.generate(
                TEST_PROFILE_PARAMS,
                posePath,
                0.0,
                TEST_VEL_CONSTRAINT,
                TEST_ACCEL_CONSTRAINT
            )
        }
        println("Full Profile Generation:")
        profileResult.printStats("  ")
        println("  Displacement samples: ${profileResult.lastValue.disps.size}")
        println()
    }

    /**
     * Demonstrates the effect of warmup by comparing cold vs warm timings.
     */
    @Test
    fun `demonstrate warmup effect`() {
        println("=== Warmup Effect Demonstration ===\n")

        // Cold run - first execution
        val coldResult = measureTimedValue {
            TrajectoryBuilder(
                TEST_TRAJECTORY_BUILDER_PARAMS,
                Pose2d(0.0, 0.0, 0.0),
                0.0,
                TEST_VEL_CONSTRAINT,
                TEST_ACCEL_CONSTRAINT
            )
                .splineTo(Vector2d(30.0, 30.0), 0.0)
                .splineTo(Vector2d(60.0, 0.0), -Math.PI / 2)
                .forward(20.0)
                .splineTo(Vector2d(30.0, -30.0), Math.PI)
                .build()
        }
        println("Cold run (1st execution): ${coldResult.duration.toString(DurationUnit.MILLISECONDS, 3)}")

        // Subsequent runs
        val subsequentTimes = (2..6).map { iteration ->
            val result = measureTimedValue {
                TrajectoryBuilder(
                    TEST_TRAJECTORY_BUILDER_PARAMS,
                    Pose2d(0.0, 0.0, 0.0),
                    0.0,
                    TEST_VEL_CONSTRAINT,
                    TEST_ACCEL_CONSTRAINT
                )
                    .splineTo(Vector2d(30.0, 30.0), 0.0)
                    .splineTo(Vector2d(60.0, 0.0), -Math.PI / 2)
                    .forward(20.0)
                    .splineTo(Vector2d(30.0, -30.0), Math.PI)
                    .build()
            }
            println("Run $iteration: ${result.duration.toString(DurationUnit.MILLISECONDS, 3)}")
            result.duration
        }

        // Warm runs after additional warmup
        println("\nAfter additional warmup:")
        repeat(10) {
            TrajectoryBuilder(
                TEST_TRAJECTORY_BUILDER_PARAMS,
                Pose2d(0.0, 0.0, 0.0),
                0.0,
                TEST_VEL_CONSTRAINT,
                TEST_ACCEL_CONSTRAINT
            )
                .splineTo(Vector2d(30.0, 30.0), 0.0)
                .splineTo(Vector2d(60.0, 0.0), -Math.PI / 2)
                .forward(20.0)
                .splineTo(Vector2d(30.0, -30.0), Math.PI)
                .build()
        }

        val warmTimes = (1..5).map { iteration ->
            val result = measureTimedValue {
                TrajectoryBuilder(
                    TEST_TRAJECTORY_BUILDER_PARAMS,
                    Pose2d(0.0, 0.0, 0.0),
                    0.0,
                    TEST_VEL_CONSTRAINT,
                    TEST_ACCEL_CONSTRAINT
                )
                    .splineTo(Vector2d(30.0, 30.0), 0.0)
                    .splineTo(Vector2d(60.0, 0.0), -Math.PI / 2)
                    .forward(20.0)
                    .splineTo(Vector2d(30.0, -30.0), Math.PI)
                    .build()
            }
            println("Warm run $iteration: ${result.duration.toString(DurationUnit.MILLISECONDS, 3)}")
            result.duration
        }

        val warmAvg = warmTimes.reduce { acc, d -> acc + d } / warmTimes.size
        println()
        println("Cold run: ${coldResult.duration.toString(DurationUnit.MILLISECONDS, 3)}")
        println("Warm average: ${warmAvg.toString(DurationUnit.MILLISECONDS, 3)}")
        println("Speedup: ${"%.2fx".format(coldResult.duration.toDouble(DurationUnit.MILLISECONDS) / warmAvg.toDouble(DurationUnit.MILLISECONDS))}")
        println()
    }

    /**
     * Compares DisplacementProfile vs TimeProfile performance.
     *
     * DisplacementProfile: Parameterized by distance along path (displacement)
     * - Query: Given displacement x, returns (x, velocity, acceleration)
     * - Useful for: Path following where you track distance traveled
     *
     * TimeProfile: Parameterized by time
     * - Query: Given time t, returns (displacement, velocity, acceleration)
     * - Useful for: Time-based trajectory following
     * - Has inverse() method to convert displacement back to time
     *
     * TimeProfile wraps DisplacementProfile and precomputes time offsets.
     */
    @Test
    fun `compare DisplacementProfile vs TimeProfile`() {
        println("=== DisplacementProfile vs TimeProfile Comparison ===")
        println("(Warmup: $WARMUP_ITERATIONS iterations, Measured: $MEASURED_ITERATIONS iterations)\n")

        // Build a path for profile generation
        val posPath = PositionPathSeqBuilder(
            Vector2d(0.0, 0.0),
            0.0,
            TEST_TRAJECTORY_BUILDER_PARAMS.arcLengthSamplingEps
        )
            .splineTo(Vector2d(30.0, 30.0), 0.0)
            .splineTo(Vector2d(60.0, 0.0), -Math.PI / 2)
            .forward(20.0)
            .build()
            .first()

        val posePath = TangentPath(posPath, 0.0)

        println("Path Info:")
        println("  Length: ${"%.2f".format(posePath.length())}")
        println()

        // ==========================================
        // GENERATION TIME COMPARISON
        // ==========================================
        println("--- Generation Time ---\n")

        // Measure CancelableProfile generation (includes DisplacementProfile)
        val dispProfileResults = measureWithWarmup {
            DisplacementProfile.generate(
                TEST_PROFILE_PARAMS,
                posePath,
                0.0,
                TEST_VEL_CONSTRAINT,
                TEST_ACCEL_CONSTRAINT
            )
        }
        val dispProfile = dispProfileResults.lastValue

        println("DisplacementProfile Generation:")
        dispProfileResults.printStats("  ")
        println("  Displacement samples: ${dispProfile.disps.size}")
        println()

        // Measure TimeProfile creation from DisplacementProfile
        val timeProfileResult = measureWithWarmup {
            TimeProfile(dispProfile)
        }
        val timeProfile = timeProfileResult.lastValue

        println("TimeProfile Creation (from existing DisplacementProfile):")
        timeProfileResult.printStats("  ")
        println("  Time samples: ${timeProfile.times.size}")
        println("  Duration: ${"%.2f".format(timeProfile.duration)} seconds")
        println()

        // Measure combined: CancelableProfile + TimeProfile
        val combinedResult = measureWithWarmup {
            val cp = CancelableProfile.generate(
                TEST_PROFILE_PARAMS,
                posePath,
                0.0,
                TEST_VEL_CONSTRAINT,
                TEST_ACCEL_CONSTRAINT
            )
            TimeProfile(cp.baseProfile)
        }

        println("Combined (CancelableProfile + TimeProfile):")
        combinedResult.printStats("  ")
        println()

        // ==========================================
        // LOOKUP TIME COMPARISON
        // ==========================================
        println("--- Lookup Time ---\n")

        val numLookups = 1000
        val dispStep = dispProfile.length / numLookups

        val dispLookupResult = measureWithWarmup {
            var sum = 0.0
            for (i in 0 until numLookups) {
                val x = i * dispStep
                val result = dispProfile[x]
                sum += result.value() // prevent optimization
            }
            sum
        }
        println("DisplacementProfile Lookups ($numLookups queries):")
        dispLookupResult.printStats("  ")
        println("  Per-lookup avg: ${"%.3f".format(dispLookupResult.average.toDouble(DurationUnit.MICROSECONDS) / numLookups)} µs")
        println()

        // Measure TimeProfile lookups by time
        val timeStep = timeProfile.duration / numLookups

        val timeLookupResult = measureWithWarmup {
            var sum = 0.0
            for (i in 0 until numLookups) {
                val t = i * timeStep
                val result = timeProfile[t]
                sum += result.value() // prevent optimization
            }
            sum
        }
        println("TimeProfile Lookups by Time ($numLookups queries):")
        timeLookupResult.printStats("  ")
        println("  Per-lookup avg: ${"%.3f".format(timeLookupResult.average.toDouble(DurationUnit.MICROSECONDS) / numLookups)} µs")
        println()

        // Measure TimeProfile.inverse() lookups (displacement -> time)
        val inverseResult = measureWithWarmup {
            var sum = 0.0
            for (i in 0 until numLookups) {
                val x = i * dispStep
                val t = timeProfile.inverse(x)
                sum += t // prevent optimization
            }
            sum
        }
        println("TimeProfile.inverse() Lookups ($numLookups queries):")
        inverseResult.printStats("  ")
        println("  Per-lookup avg: ${"%.3f".format(inverseResult.average.toDouble(DurationUnit.MICROSECONDS) / numLookups)} µs")
        println()

        // ==========================================
        // SUMMARY
        // ==========================================
        println("=== Summary ===")
        println()
        println("Generation Time:")
        println("  CancelableProfile (w/ DisplacementProfile): ${dispProfileResults.average.toString(DurationUnit.MILLISECONDS, 3)}")
        println("  TimeProfile (from DisplacementProfile):     ${timeProfileResult.average.toString(DurationUnit.MILLISECONDS, 3)}")
        println("  Combined (both):                            ${combinedResult.average.toString(DurationUnit.MILLISECONDS, 3)}")
        println()
        println("Lookup Time (per query):")
        println("  DisplacementProfile[x]:  ${"%.3f".format(dispLookupResult.average.toDouble(DurationUnit.MICROSECONDS) / numLookups)} µs")
        println("  TimeProfile[t]:          ${"%.3f".format(timeLookupResult.average.toDouble(DurationUnit.MICROSECONDS) / numLookups)} µs")
        println("  TimeProfile.inverse(x):  ${"%.3f".format(inverseResult.average.toDouble(DurationUnit.MICROSECONDS) / numLookups)} µs")
        println()
        println("Use Cases:")
        println("  - DisplacementProfile: When tracking distance traveled (e.g., encoder-based)")
        println("  - TimeProfile[t]: When following trajectory by elapsed time")
        println("  - TimeProfile.inverse(x): When converting displacement to expected time")
        println()
    }
}

