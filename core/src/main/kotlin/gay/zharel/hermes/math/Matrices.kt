/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

@file:JvmName("Matrices")

package gay.zharel.hermes.math

import org.ejml.data.DMatrixRMaj
import org.ejml.dense.row.decomposition.chol.CholeskyDecompositionInner_DDRM
import org.ejml.dense.row.decomposition.chol.CholeskyDecompositionLDL_DDRM
import org.ejml.dense.row.decomposition.lu.LUDecompositionAlt_DDRM
import org.ejml.dense.row.decomposition.qr.QRDecompositionHouseholder_DDRM
import org.ejml.interfaces.decomposition.CholeskyDecomposition
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
class Matrix internal constructor(internal val simple: SimpleMatrix) {
    /**
     * Constructor to create a [Matrix] from a 2D array.
     */
    constructor(data: Array<DoubleArray>) : this(SimpleMatrix(data))

    /**
     * Constructor to create a [Matrix] from a list of lists.
     */
    constructor(data: Collection<Collection<Double>>) : this(data.map { it.toDoubleArray() }.toTypedArray())

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
     * The transpose of this matrix.
     */
    @get:JvmName("transpose")
    val transpose: Matrix
        get() = Matrix(simple.transpose())

    /**
     * Returns a copy of this matrix.
     */
    fun copy() = Matrix(simple.copy())

    /**
     * The inverse of this matrix.
     * The matrix must be square and invertible.
     * If the matrix is not square, use [pseudoInverse] instead.
     */
    @get:JvmName("inverse")
    val inverse: Matrix
        get() = Matrix(simple.invert())

    /**
     * The pseudo-inverse of this matrix.
     * The matrix must be square and invertible.
     */
    @get:JvmName("pseudoInverse")
    val pseudoInverse: Matrix
        get() = Matrix(simple.pseudoInverse())

    /**
     * Returns the Frobenius norm of this matrix.
     * The Frobenius norm is the square root of the sum of squares of all elements.
     */
    @get:JvmName("norm")
    val norm: Double
        get() = simple.normF()

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

    /**
     * Returns a submatrix of this matrix.
     * @param startRow First row to include in the submatrix, inclusive.
     * @param endRow Last row to include in the submatrix, exclusive.
     * @param startCol First column to include in the submatrix, inclusive.
     * @param endCol Last column to include in the submatrix, exclusive.
     */
    fun slice(startRow: Int, endRow: Int, startCol: Int, endCol: Int) = Matrix(
        simple.extractMatrix(startRow, endRow, startCol, endCol)
    )

    /**
     * Returns the LLT (Cholesky) decomposition of this matrix.
     * Only works for symmetric, positive-definite matrices.
     * Provides in-place rank-1 update/downdate methods.
     */
    fun llt(): LLTDecomposition {
        val chol = CholeskyDecompositionInner_DDRM(true)
        val mat = simple.ddrm.copy()
        require(chol.decompose(mat)) { "Matrix is not symmetric positive-definite" }
        return LLTDecomposition(chol, mat)
    }

    /**
     * Returns the LDLT decomposition of this matrix.
     * Only works for symmetric matrices.
     */
    fun ldlt(): LDLTDecomposition {
        val ldlt = CholeskyDecompositionLDL_DDRM()
        val mat = simple.ddrm.copy()
        require(ldlt.decompose(mat)) { "Matrix is not positive definite" }
        return LDLTDecomposition(ldlt, mat)
    }

    /**
     * Returns the LU decomposition of this matrix.
     */
    fun lu(): LUDecomposition {
        val lu = LUDecompositionAlt_DDRM()
        val mat = simple.ddrm.copy()
        require(lu.decompose(mat)) { "Matrix is singular or not square" }
        return LUDecomposition(lu, mat)
    }

    /**
     * Returns the QR decomposition of this matrix.
     */
    fun qr(): QRDecomposition {
        val qr = QRDecompositionHouseholder_DDRM()
        val mat = simple.ddrm.copy()
        require(qr.decompose(mat)) { "QR decomposition failed" }
        return QRDecomposition(qr, mat)
    }

    override fun toString(): String = (simple.toArray2()).contentDeepToString()

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        return other is Matrix && this.simple.isIdentical(other.simple, 1e-6)
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

/** Data class for LLT (Cholesky) decomposition, with rank-1 update/downdate. */
@ConsistentCopyVisibility
data class LLTDecomposition internal constructor(
    private val chol: CholeskyDecomposition<DMatrixRMaj>,
    private var mat: DMatrixRMaj
) {
    /** Lower-triangular matrix L such that mat = L*L^T */
    val L: Matrix get() = Matrix(SimpleMatrix(chol.getT(null)))

    /** In-place rank-1 update: mat := mat + x*x^T */
    fun update(x: DoubleArray) {
        // x must be column vector of correct size
        // Use EJML's rank-1 update if available, else manual
        // Here, we re-decompose for simplicity
        for (i in 0..mat.numRows) {
            for (j in 0..mat.numCols) {
                mat[i, j] += x[i] * x[j]
            }
        }
        require(chol.decompose(mat))
    }

    /** In-place rank-1 downdate: mat := mat - x*x^T */
    fun downdate(x: DoubleArray) {
        for (i in 0..mat.numRows) {
            for (j in 0..mat.numCols) {
                mat[i, j] -= x[i] * x[j]
            }
        }
        require(chol.decompose(mat))
    }
}

/** Data class for LDLT decomposition. */
@ConsistentCopyVisibility
data class LDLTDecomposition internal constructor(
    private val ldlt: CholeskyDecompositionLDL_DDRM,
    private val mat: DMatrixRMaj
) {
    val L: Matrix get() = Matrix(SimpleMatrix(ldlt.getL(null)))
    val D: Matrix get() = Matrix(SimpleMatrix(ldlt.getD(null)))
}

/** Data class for LU decomposition. */
@ConsistentCopyVisibility
data class LUDecomposition internal constructor(
    private val lu: LUDecompositionAlt_DDRM,
    private val mat: DMatrixRMaj
) {
    val L: Matrix get() = Matrix(SimpleMatrix(lu.getLower(null)))
    val U: Matrix get() = Matrix(SimpleMatrix(lu.getUpper(null)))
    val P: Matrix get() = Matrix(SimpleMatrix(lu.getRowPivot(null)))
}

/** Data class for QR decomposition. */
@ConsistentCopyVisibility
data class QRDecomposition internal constructor(
    private val qr: QRDecompositionHouseholder_DDRM,
    private val mat: DMatrixRMaj
) {
    val Q: Matrix get() = Matrix(SimpleMatrix(qr.getQ(null, false)))
    val R: Matrix get() = Matrix(SimpleMatrix(qr.getR(null, false)))
}
