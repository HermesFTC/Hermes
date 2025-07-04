package com.acmerobotics.roadrunner.geometry

import org.ejml.dense.row.decomposition.lu.LUDecompositionAlt_DDRM
import org.ejml.simple.SimpleMatrix
import kotlin.math.pow
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty


internal operator fun SimpleMatrix.unaryMinus() = this.times(-1.0)
internal operator fun SimpleMatrix.times(other: SimpleMatrix): SimpleMatrix = this.mult(other)
internal operator fun SimpleMatrix.times(other: Double): SimpleMatrix = this.scale(other)
internal operator fun Double.times(other: SimpleMatrix): SimpleMatrix = other.times(this)

enum class TYPE_1D {
    COLUMN,
    ROW,
}

/**
 * Represents a matrix of doubles.
 * Internally represented as a SimpleMatrix from EJML.
 */
class Matrix(data: Array<DoubleArray>) {
    constructor(data: DoubleArray, type1d: TYPE_1D = TYPE_1D.ROW) : this(type1d.let {
        when (it) {
            TYPE_1D.COLUMN -> arrayOf(data).transpose()
            TYPE_1D.ROW -> arrayOf(data)
        }
    })

    internal constructor(matrix: SimpleMatrix) : this(matrix.toArray2())

    internal val simple = SimpleMatrix(data)

    val numCols = simple.numCols
    val numRows = simple.numRows

    fun transpose() = Matrix(simple.transpose().toArray2())

    operator fun get(i: Int, j: Int) = simple[i, j]

    operator fun set(i: Int, j: Int, value: Double) {
        simple[i, j] = value
    }

    override fun toString(): String = (simple.toArray2()).contentDeepToString()

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false
        other as Matrix
        return simple.isIdentical(other.simple, 1e-6)
    }

    override fun hashCode(): Int {
        return simple.hashCode()
    }
}

class DiagonalElement<M>(val matrix: Matrix, val index: Int) : ReadWriteProperty<M, Double> {
    override fun getValue(
        thisRef: M,
        property: KProperty<*>,
    ): Double {
        return matrix[index, index]
    }

    override fun setValue(
        thisRef: M,
        property: KProperty<*>,
        value: Double,
    ) {
        matrix[index, index] = value
    }
}

private fun Array<DoubleArray>.transpose(): Array<DoubleArray> {
    val rows = this.size
    val cols = this[0].size
    val ret = Array(cols) { DoubleArray(rows) }
    for (i in 0 until rows) {
        for (j in 0 until cols) {
            ret[j][i] = this[i][j]
        }
    }
    return ret
}

/**
 * Creates a cost matrix from the given tolerances using Bryson's rule.
 */
fun makeBrysonMatrix(vararg tolerances: Double) = Matrix(makeBrysonMatrix(tolerances))

internal fun makeBrysonMatrix(tolerances: DoubleArray) = SimpleMatrix.diag(*tolerances.map {
    if (it.isFinite()) {
        1.0 / it.pow(2)
    } else {
        0.0
    }
}.toDoubleArray())

internal fun SimpleMatrix.lu() = LUDecompositionAlt_DDRM().let {
    it.decompose(this.ddrm)
    SimpleMatrix.wrap(it.lu)
}