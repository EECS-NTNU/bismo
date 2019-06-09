// Copyright (c) 2018 Norwegian University of Science and Technology (NTNU)
// Copyright (c) 2019 Xilinx
//
// BSD v3 License
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of BISMO nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


// useful definitions for testing Chisel modules inside BISMO
object BISMOTestHelpers {
  // standard arguments to pass to chiselTest
  val stdArgs = Array("--genHarness", "--compile", "--test", "--backend", "c",
    "--targetDir", "build/test", "--vcd")

  // random number generation functions
  val r = scala.util.Random

  // treat given positive int as signed based on given num. of bits
  def treatAs2sCompl(in: Int, nBits: Int): Int = {
    val negOffset = (1 << (nBits - 1))
    return if (in >= negOffset) in - 2 * negOffset else in
  }

  // extract bit position at pos from number in
  def extractBitPos(in: Int, pos: Int, nBits: Int): Int = {
    val negOffset = (1 << (nBits - 1))
    if (in < 0) {
      return ((in + 2 * negOffset) & (1 << pos)) >> pos
    } else {
      return (in & (1 << pos)) >> pos
    }
  }

  // convert a sequence of integers into bit-serial form
  // e.g. [2, 0, 3] turns to [[0, 0, 1], [1, 0, 1]]
  def intVectorToBitSerial(in: Seq[Int], nBits: Int): Seq[Seq[Int]] = {
    for (i ← 0 to nBits - 1) yield in.map(x ⇒ extractBitPos(x, i, nBits))
  }

  // generate a random vector of len integers, each nBits wide, may be signed
  // if allowNeg is true
  def randomIntVector(len: Int, nBits: Int, allowNeg: Boolean): Seq[Int] = {
    val seq = for (i ← 1 to len) yield r.nextInt(1 << nBits)

    if (allowNeg) {
      return seq.map(i ⇒ treatAs2sCompl(i, nBits))
    } else {
      return seq
    }
  }

  // reshape a vector of integers into a matrix, treating the vector as a
  // row-major matrix
  def vectorToMatrix(in: Seq[Int], rows: Int, cols: Int): Seq[Seq[Int]] = {
    return for (i ← 0 to rows - 1) yield in.slice(i * cols, (i + 1) * cols)
  }

  // compute the dot product of two number sequences
  def dotProduct(a: Seq[Int], b: Seq[Int]): Int = {
    return (a zip b).map { case (x, y) ⇒ x * y }.reduce(_ + _)
  }

  // compute the product of two matrices a and b. assume a's first index returns
  // rows (row major), while b's first index returns columns (column major)
  // i.e. with sizes: a[m][k] x b[n][k] = c[m][n]
  def matrixProduct(a: Seq[Seq[Int]], b: Seq[Seq[Int]]): Seq[Seq[Int]] = {
    // ensure correct dimensions
    assert(a(0).size == b(0).size)
    // pairwise dot products
    val ret = for (i ← 0 to a.size - 1) yield for (j ← 0 to b.size - 1) yield dotProduct(a(i), b(j))
    return ret
  }

  // generate a matrix of random integers with desired size
  def randomIntMatrix(row: Int, col: Int, nBits: Int, allowNeg: Boolean): Seq[Seq[Int]] = {
    return vectorToMatrix(randomIntVector(row * col, nBits, allowNeg), row, col)
  }
  //helper funtction to quantize a given matrix with a given matrix of thresholds
  def quantizeMatrix(a: Seq[Seq[Int]], b: Seq[Seq[Int]]): Seq[Seq[Int]] = {
    val ret = for (i ← 0 to a.size - 1) yield for (j ← 0 to a(i).size - 1) yield {
      b(i).map((x: Int) ⇒ if (a(i)(j) > x) 1 else 0).reduce(_ + _)

    }
    return ret
  }

  //Printer helper function
  def printMatrix(a: Seq[Seq[Int]]) = {
    for (i ← 0 to a.size - 1) {
      for (j ← 0 to a(i).size - 1) {
        print(a(i)(j))
        print(" ")
      }
      println(" ")
    }
  }

  def printVector(a: Seq[Int]) = {
    for (i ← 0 to a.size - 1)
      print(Integer.toString(a(i), 2))
    println()
  }
}
