/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes

import gay.zharel.hermes.math.Matrix
import io.kotest.matchers.Matcher
import io.kotest.matchers.MatcherResult
import io.kotest.matchers.doubles.ToleranceMatcher

operator fun ToleranceMatcher.contains(value: Double): Boolean = this.test(value).passed()
fun Matrix.hasNaN() = toArray2().any { row -> row.any { it.isNaN() } }
fun Matrix.hasInfinite() = toArray2().any { row -> row.any { it.isInfinite() } }

fun Matrix.isSymmetric() = toArray2().contentDeepEquals(this.transpose.toArray2())

fun beSymmetric() = Matcher<Matrix> { value ->
  MatcherResult(
    value.isSymmetric(),
    { "Matrix should be symmetric" },
    { "Matrix should not be symmetric" },
  )
}
