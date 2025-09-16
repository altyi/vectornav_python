// The MIT License (MIT)
// 
// VectorNav SDK (v0.22.0)
// Copyright (c) 2024 VectorNav Technologies, LLC
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef VN_LINEARLEASTSQUARES_HPP_
#define VN_LINEARLEASTSQUARES_HPP_

#include <cmath>

#include "vectornav/Debug.hpp"
#include "vectornav/EigenDecomposition.hpp"
#include "vectornav/LinearAlgebra.hpp"
#include "vectornav/TemplateLibrary/Matrix.hpp"

namespace VN
{
namespace Math
{

class LinearLeastSquares
{
public:
    enum class LlsError
    {
        None = 0,
        InsufficientData = 9,
        FailedEigenDecomposition = 11,
    };
    template <uint16_t n, typename T>
    struct LeastSquaresSolution
    {
        Matrix<n, 1, T> solution;
        Matrix<n, n, T> eigenVectors;
        Matrix<n, 1, T> eigenValuesReal;
        Matrix<n, 1, T> eigenValuesImag;
        T FOM;
        LlsError error;  // Replace with error struct
    };

    LinearLeastSquares() = default;

    template <uint16_t n, typename T>
    LeastSquaresSolution<n, T> solve(Matrix<n, n, T>& HTH, Matrix<n, 1, T>& HTy, T condition_threshold) noexcept
    {
        // Initialize structures
        LeastSquaresSolution<n, T> solution{Matrix<n, 1, T>(0.0), Matrix<n, n, T>(0.0), Matrix<n, 1, T>(0.0), Matrix<n, 1, T>(0.0), 0.0, LlsError::None};

        EigenDecomposition eigenDecomp;
        if (eigenDecomp.computeDecomposition(HTH, solution.eigenValuesReal, solution.eigenValuesImag, solution.eigenVectors))
        {
            solution.error = LlsError::FailedEigenDecomposition;
            return solution;
        }

        // Check if ill conditioned
        T minEigenValue = solution.eigenValuesReal(mini(solution.eigenValuesReal));
        if (minEigenValue < condition_threshold)
        {
            solution.error = LlsError::InsufficientData;
            return solution;
        }

        solution.FOM = 1.0 / std::sqrt(minEigenValue);

        // Compute LLS solution
        Matrix<n, n, T> tempMat = transpose(solution.eigenVectors);
        Matrix<n, 1, T> tempVec = tempMat * HTy;
        for (uint16_t i{0}; i < n; ++i)
        {
            for (uint16_t j{0}; j < n; ++j) { tempMat(i, j) = solution.eigenVectors(i, j) / solution.eigenValuesReal(j); }
        }

        solution.solution = tempMat * tempVec;

        return solution;
    }
};

template <uint16_t m, uint16_t n, typename T>
Matrix<n, 1, T> solveLinearLeastSquares(Matrix<m, n, T> A, const Matrix<m, 1, T>& y) noexcept
{
    return solveLinearSystemLU(transpose(A) * A, transpose(A) * y);
}

}  // namespace Math
}  // namespace VN

#endif  // VN_LINEARLEASTSQUARES_HPP_
