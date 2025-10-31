/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.math

import gay.zharel.hermes.math.lerp
import java.util.NavigableMap
import java.util.TreeMap


/**
 * @usesMathJax
 *
 * Linearly interpolates between \([fromLo, fromHi]\) and \([toLo, toHi]\) at value [x].
 */
fun lerp(x: Double, fromLo: Double, fromHi: Double, toLo: Double, toHi: Double) =
    if (fromLo == fromHi) 0.0
    else
        toLo + (x - fromLo) * (toHi - toLo) / (fromHi - fromLo)

/**
 * @usesMathJax
 *
 * Linearly interpolates between \(low\) and \(high\) at value [t].
 *
 * @param [t] interpolation factor, typically in the range \([0, 1]\)
 */
fun lerp(low: Double, high: Double, t: Double) = low + (high - low) * t

/**
 * @usesMathJax
 *
 * Inverse of [lerp]. Given a value [value] in the range \([low, high]\), returns the interpolation factor
 * \(t\) such that \(\text{lerp}(low, high, t) = value\).
 *
 * @param [value] value in the range \([low, high]\)
 * @return interpolation factor \(t\)
 */
fun antiLerp(value: Double, low: Double, high: Double) = (value - low) / (high - low)

/**
 * Searches for the nearest value in [source] to the given [query] value.
 * If [query] exactly matches a value in [source], the corresponding value in [target] is returned.
 * If not, the two nearest values in [source] are found, and linear interpolation is performed
 * between the corresponding values in [target].
 *
 * @param [source] sorted list of source values
 * @param [target] sorted list of target values
 * @param [query] value to search for
 */
fun lerpLookup(source: List<Double>, target: List<Double>, query: Double): Double {
    require(source.size == target.size) { "source.size (${source.size}) != target.size (${target.size})" }
    require(source.isNotEmpty()) { "source is empty" }

    val index = source.binarySearch(query)
    return if (index >= 0) {
        target[index]
    } else {
        val insIndex = -(index + 1)
        when {
            insIndex <= 0 -> target.first()
            insIndex >= source.size -> target.last()
            else -> {
                val sLo = source[insIndex - 1]
                val sHi = source[insIndex]
                val tLo = target[insIndex - 1]
                val tHi = target[insIndex]
                lerp(query, sLo, sHi, tLo, tHi)
            }
        }
    }
}

/**
 * Performs linear interpolation lookup for a list of query values.
 *
 * Given sorted `source` and `target` lists of equal length, and a sorted list of `queries`,
 * this function returns a list of interpolated values from `target` corresponding to each query in `queries`.
 * For each query:
 * - If the query is less than the smallest value in `source`, the first value in `target` is returned.
 * - If the query is greater than the largest value in `source`, the last value in `target` is returned.
 * - Otherwise,
 * linear interpolation is performed between the two nearest values in `source` and their corresponding values in `target`.
 *
 * Preconditions:
 * - `source` and `target` must be sorted in ascending order and have the same length.
 * - `queries` must be sorted in ascending order.
 *
 * @param source Sorted list of source values.
 * @param target Sorted list of target values.
 * @param queries Sorted list of query values.
 * @return List of interpolated values from `target` for each query.
 * @throws IllegalArgumentException if `source` and `target` have different sizes or if `source` is empty.
 */
fun lerpLookupMap(source: List<Double>, target: List<Double>, queries: List<Double>): List<Double> {
    require(source.size == target.size) { "source.size (${source.size}) != target.size (${target.size})" }
    require(source.isNotEmpty()) { "source is empty" }

    val result = mutableListOf<Double>()

    var i = 0
    for (query in queries) {
        if (query < source[0]) {
            result.add(target[0])
            continue
        }

        while (i + 1 < source.size && source[i + 1] < query) {
            i++
        }

        if (i + 1 == source.size) {
            result.add(target.last())
            continue
        }

        val sLo = source[i]
        val sHi = source[i + 1]
        val tLo = target[i]
        val tHi = target[i + 1]
        result.add(lerp(query, sLo, sHi, tLo, tHi))
    }

    return result
}

/**
 * A navigable map that supports interpolation between values.
 *
 * `InterpolatingMap` is a wrapper around a `TreeMap<Double, T>` that, when queried with a key,
 * returns the exact value if the key exists, or interpolates between the two nearest values otherwise.
 * The interpolation logic is provided by the user via the `interpolate` function.
 *
 * @param T The type of values stored in the map.
 * @property tree The underlying `TreeMap` storing the data.
 * @property interpolate A function that takes two values and an interpolation factor in [0.0, 1.0], and returns an interpolated value.
 *
 * @constructor Creates an empty `InterpolatingMap` with the given interpolation function.
 * @constructor Creates an `InterpolatingMap` with the given keys and values, using the provided interpolation function.
 *
 * @throws IllegalArgumentException if the number of keys and values do not match.
 */
class InterpolatingMap<T> private constructor(
    val tree: TreeMap<Double, T>,
    val interpolate: (T, T, Double) -> T
) : NavigableMap<Double, T> by tree {
    constructor(interpolator: (T, T, Double) -> T) : this(TreeMap(), interpolator)

    constructor(interpolator: (T, T, Double) -> T, keys: List<Double>, values: List<T>) :
            this(TreeMap(), interpolator) {
        require(keys.size == values.size) { "Keys and values must be the same size" }
        for (i in keys.indices) {
            tree[keys[i]] = values[i]
        }
    }

    /**
     * Gets the value associated with the given key.
     * If the key does not exist,
     * the value is interpolated between the two nearest values.
     */
    override fun get(key: Double): T {
        val low = floorEntry(key)
        val high = ceilingEntry(key)

        if (low.key == high.key) {
            return tree[key]!!
        }

        val t = lerp(key, low.key, high.key, 0.0, 1.0)

        return interpolate(low.value, high.value, t)
    }
}