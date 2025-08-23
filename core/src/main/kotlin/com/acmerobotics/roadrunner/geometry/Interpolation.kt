package com.acmerobotics.roadrunner.geometry

import java.util.NavigableMap
import java.util.TreeMap

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