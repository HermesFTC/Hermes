@file:JvmName("Matrices")
package gay.zharel.hermes.geometry

import org.ejml.dense.row.decomposition.lu.LUDecompositionAlt_DDRM
import org.ejml.simple.SimpleMatrix
import kotlin.math.pow

internal operator fun SimpleMatrix.unaryMinus() = this.times(-1.0)
internal operator fun SimpleMatrix.times(other: SimpleMatrix): SimpleMatrix = this.mult(other)
internal operator fun SimpleMatrix.times(other: Double): SimpleMatrix = this.scale(other)
internal operator fun Double.times(other: SimpleMatrix): SimpleMatrix = other.times(this)


/**
 * Represents a matrix of doubles.
 * Internally represented as a SimpleMatrix from EJML.
 */
class Matrix(data: Array<DoubleArray>) {
    /**
     * Constructor to create a [Matrix] from a list of lists.
     */
    constructor(data: Collection<Collection<Double>>) : this(data.map { it.toDoubleArray() }.toTypedArray())

    /**
     * Internal constructor to create a [Matrix] from an EJML [SimpleMatrix].
     */
    internal constructor(matrix: SimpleMatrix) : this(matrix.toArray2())

    companion object {
        /**
         * Creates a zero matrix with the given dimensions.
         */
        @JvmStatic
        fun zero(rows: Int, cols: Int) = Matrix(SimpleMatrix(rows, cols))

        /**
         * Creates a zero matrix with dimensions [size] by [size].
         */
        @JvmStatic
        fun zero(size: Int) = zero(size, size)

        /**
         * Creates an identity matrix with dimensions [size] by [size].
         */
        @JvmStatic
        fun identity(size: Int) = Matrix(SimpleMatrix.identity(size))

        /**
         * Creates a matrix with [data] along the diagonal
         * and all other elements set to 0.
         */
        @JvmStatic
        fun diagonal(vararg data: Double) = Matrix(SimpleMatrix.diag(*data))

        @JvmStatic
        fun diagonal(data: Collection<Double>) = diagonal(*data.toDoubleArray())

        /**
         * Creates a 1 by n matrix with [data] as its elements,
         * where n is [data].size.
         */
        @JvmStatic
        fun row(vararg data: Double) =
            Matrix(SimpleMatrix(1, data.size, true, *data))

        /**
         * Creates a 1 by n matrix with [data] as its elements,
         * where n is [data].size.
         */
        @JvmStatic
        fun row(data: Collection<Double>) = row(*data.toDoubleArray())

        /**
         * Creates an n by 1 matrix with [data] as its elements,
         * where n is [data].size.
         */
        @JvmStatic
        fun column(vararg data: Double) =
            Matrix(SimpleMatrix(data.size, 1, false, *data))

        /**
         * Creates an n by 1 matrix with [data] as its elements,
         * where n is [data].size.
         */
        @JvmStatic
        fun column(data: Collection<Double>) = column(*data.toDoubleArray())
    }

    internal val simple = SimpleMatrix(data)

    /**
     * The number of columns in the matrix.
     */
    @JvmField val numColumns = simple.numCols

    /**
     * The number of rows in the matrix.
     */
    @JvmField val numRows = simple.numRows

    /**
     * The size of the matrix.
     *
     * @return a pair of integers representing the number of rows and columns respectively.
     */
    @JvmField val size = numRows to numColumns

    /**
     * Returns the transpose of this matrix.
     */
    fun transpose() = Matrix(simple.transpose())


    /**
     * Returns the matrix with all elements negated.
     * This is equivalent to multiplying the matrix by -1.
     */
    operator fun unaryMinus() = Matrix(-simple)

    /** 
     * Adds another matrix to this matrix.
     * The matrices must have the same dimensions.
     */
    operator fun plus(other: Matrix) = Matrix(this.simple + other.simple)

    /**
     * Subtracts another matrix from this matrix.
     * The matrices must have the same dimensions.
     */
    operator fun minus(other: Matrix) = Matrix(this.simple - other.simple)

    /**
     * Multiplies this matrix by another matrix.
     * The number of columns in this matrix must match the number of rows in the other matrix.
     */
    operator fun times(other: Matrix) = Matrix(this.simple * other.simple)

    /**
     * Multiplies this matrix by a scalar.
     */
    operator fun times(scalar: Double) = Matrix(simple * scalar)

    /**
     * Multiplies this matrix by a scalar.
     */
    operator fun times(scalar: Int) = Matrix(simple * scalar.toDouble())

    /**
     * @usesMathJax
     *
     * Solves for X in the equation \(AX = B)\,
     * where A is this matrix and B is other.
     */
    fun solve(other: Matrix): Matrix = Matrix(this.simple.solve(other.simple))

    /**
     * Returns the element at the given indices.
     */
    operator fun get(i: Int, j: Int) = simple[i, j]

    /**
     * Sets the element at the given indices to the given [value].
     */
    operator fun set(i: Int, j: Int, value: Double) {
        simple[i, j] = value
    }

    /**
     * Returns the [n]th row of the matrix.
     */
    fun row(n: Int) = Matrix(simple.getRow(n))

    /**
     * Returns the [n]th column of the matrix.
     */
    fun column(n: Int) = Matrix(simple.getColumn(n))

    /**
     * Returns the diagonal elements of this matrix.
     */
    fun diagonals() = Matrix(simple.diag())

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

/**
 * Solves for X in the equation \(aX = b)\.
 */
fun solveMatrices(a: Matrix, b: Matrix) = a.solve(b)

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

fun lerpMatrix(start: Matrix, end: Matrix, t: Double): Matrix {
    require(start.size == end.size) { "Matrices must have the same size" }

    return (end - start) * t + start
}

internal fun lerpMatrix(start: SimpleMatrix, end: SimpleMatrix, t: Double): SimpleMatrix =
    lerpMatrix(Matrix(start), Matrix(end), t).simple

fun lerpMatrixLookup(times: List<Double>, matrices: List<Matrix>, query: Double): Matrix {
    val index = times.binarySearch(query)

    if (index >= 0) return matrices[index]

    val nextIdx = -(index + 1)
    val prevIdx = -index

    return lerpMatrix(
        matrices[prevIdx],
        matrices[nextIdx],
        lerp(query, times[prevIdx], times[nextIdx], 0.0, 1.0)
    )
}

internal fun lerpMatrixLookup(times: List<Double>, matrices: List<SimpleMatrix>, query: Double): SimpleMatrix =
    lerpMatrixLookup(times, matrices.map { Matrix(it) }, query).simple