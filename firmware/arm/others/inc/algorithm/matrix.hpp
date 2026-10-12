/**
 * This file was implemented based on design ideas and algorithms referenced from:
 * SJTU-RoboMaster-Team/Matrix_and_Robotics_on_STM32
 * https://github.com/SJTU-RoboMaster-Team/Matrix_and_Robotics_on_STM32
 */

#pragma once

#define DEBUG_MATH 0
#if DEBUG_MATH == 1
#include <cassert>
#endif
#include <cstring>
#include <cmath>
#include "arm_math.h"

#define MATRIX_EPSILON 1e-6f
#define MATRIX_SINGULAR_EPSILON 1e-12f


// Matrix class
template <int _rows, int _cols>
class Matrixf {
	public:
	// Constructor without input data
	Matrixf() : arm_mat_{0, 0, nullptr}, rows_(_rows), cols_(_cols), data_{} {
		arm_mat_init_f32(&arm_mat_, _rows, _cols, this->data_);
	}
	// Constructor with input data
	explicit Matrixf(const float data[_rows * _cols]) : Matrixf() {
		memcpy(this->data_, data, _rows * _cols * sizeof(float));
	}
	// Copy constructor
	Matrixf(const Matrixf& mat) : Matrixf() {
		memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
	}
	// Destructor
	~Matrixf() = default;

	// Row size
	static int rows() { return _rows; }
	// Column size
	static int cols() { return _cols; }

	// Element
	float* operator[](const int& row) { return &this->data_[row * _cols]; }
	const float* operator[](const int& row) const { return &this->data_[row * _cols]; }  // const version

	// Operators
	Matrixf& operator=(const Matrixf& mat) {
		if (this == &mat) return *this;
		memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
		return *this;
	}
	Matrixf& operator+=(const Matrixf& mat) {
		arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
		return *this;
	}
	Matrixf& operator-=(const Matrixf& mat) {
		arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
		return *this;
	}
	Matrixf& operator*=(const float& val) {
		arm_mat_scale_f32(&this->arm_mat_, val, &this->arm_mat_);
		return *this;
	}
	Matrixf& operator/=(const float& val) {
		if (fabs(val) < MATRIX_EPSILON) {
			return *this;
		}
		arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &this->arm_mat_);
		return *this;
	}
	Matrixf operator+(const Matrixf& mat) const {
		Matrixf res;
		arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
		return res;
	}
	Matrixf operator-(const Matrixf& mat) const {
		Matrixf res;
		arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
		return res;
	}
	Matrixf operator*(const float& val) const {
		Matrixf res;
		arm_mat_scale_f32(&this->arm_mat_, val, &res.arm_mat_);
		return res;
	}
	friend Matrixf operator*(const float& val, const Matrixf& mat) {
		Matrixf res;
		arm_mat_scale_f32(&mat.arm_mat_, val, &res.arm_mat_);
		return res;
	}
	Matrixf operator/(const float& val) const {
		if (fabs(val) < MATRIX_EPSILON) {
			return *this;
		}
		Matrixf res;
		arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &res.arm_mat_);
		return res;
	}
	// Matrix multiplication
	template <int cols>
	friend Matrixf<_rows, cols> operator*(const Matrixf& mat1, const Matrixf<_cols, cols>& mat2) {
		Matrixf<_rows, cols> res;
#if DEBUG_MATH == 1
		arm_status s =
#endif
		arm_mat_mult_f32(&mat1.arm_mat_, &mat2.arm_mat_, &res.arm_mat_);
#if DEBUG_MATH == 1
		assert(s == ARM_MATH_SUCCESS);
#endif
		return res;
	}

	// Submatrix
	template <int rows, int cols>
	Matrixf<rows, cols> block(const int& start_row, const int& start_col) const & {
		Matrixf<rows, cols> res;
		for (int i = 0; i < rows; ++i) {
			memcpy(res[i], (*this)[start_row + i] + start_col, cols * sizeof(float));
		}
		return res;
	}

	// Specific row
	Matrixf<1, _cols> row(const int& row) const & { return block<1, _cols>(row, 0); }
	// Specific column
	Matrixf<_rows, 1> col(const int& col) const & { return block<_rows, 1>(0, col); }

	// Transpose
	Matrixf<_cols, _rows> trans() const {
		Matrixf<_cols, _rows> res;
		arm_mat_trans_f32(&arm_mat_, &res.arm_mat_);
		return res;
	}
	static constexpr int min_rc() noexcept { return _rows < _cols ? _rows : _cols; }
	// Trace
	float trace() const {
		float res = 0.f;
		for (int i = 0; i < min_rc(); ++i) {
			res += data_[i * _cols + i];
		}
		return res;
	}
	// Norm
	float norm() const
	{
		float sum = 0.f;
		arm_dot_prod_f32(this->data_, this->data_, _rows * _cols, &sum);
		return sqrtf(sum);
	}

	// arm matrix instance
	arm_matrix_instance_f32 arm_mat_;

	protected:
	// size
	int rows_, cols_;
	// data
	float data_[_rows * _cols];
};

// Matrix functions
namespace matrixf {

// Special Matrices
// Zero matrix
template <int _rows, int _cols>
Matrixf<_rows, _cols> zeros() {
	Matrixf<_rows, _cols> res;
	memset(res[0], 0, _rows * _cols * sizeof(float));
	return res;
}
// Ones matrix
template <int _rows, int _cols>
Matrixf<_rows, _cols> ones() {
	Matrixf<_rows, _cols> res;
	float* ptr = res[0];
	for (int i = 0; i < _rows * _cols; ++i) {
		ptr[i] = 1.0f;
	}
	return res;
}
// Identity matrix
template <int _rows, int _cols>
Matrixf<_rows, _cols> eye() {
	Matrixf<_rows, _cols> res = zeros<_rows, _cols>();
	constexpr int n = _rows < _cols ? _rows : _cols;
	for (int i = 0; i < n; ++i) {
		res[i][i] = 1.0f;
	}
	return res;
}
// Diagonal matrix
template <int _rows, int _cols>
Matrixf<_rows, _cols> diag(const Matrixf<_rows, 1>& vec) {
	Matrixf<_rows, _cols> res = zeros<_rows, _cols>();
	constexpr int n = _rows < _cols ? _rows : _cols;
	for (int i = 0; i < n; ++i) {
		res[i][i] = vec[i][0];
	}
	return res;
}

// Inverse
template <int _dim>
Matrixf<_dim, _dim> inv(const Matrixf<_dim, _dim>& mat) {
#if DEBUG_MATH == 1
	arm_status s;
#endif
	// ext_mat = [A | I]
	Matrixf<_dim, 2 * _dim> ext_mat = matrixf::zeros<_dim, 2 * _dim>();
	for (int i = 0; i < _dim; ++i) {  // Copy A
		memcpy(ext_mat[i], mat[i], _dim * sizeof(float));
		ext_mat[i][_dim + i] = 1.f;
	}
	for (int i = 0; i < _dim; ++i) {  // Elimination
		// find maximum absolute value in the first column in lower right block
		float abs_max = fabs(ext_mat[i][i]);
		int abs_max_row = i;
		for (int row = i + 1; row < _dim; ++row) {
			const float val = fabs(ext_mat[row][i]);
			if (val > abs_max) {
				abs_max = val;
				abs_max_row = row;
			}
		}
		if (abs_max < MATRIX_SINGULAR_EPSILON) {  // singular matrix
#if DEBUG_MATH == 1
			s = ARM_MATH_SINGULAR;
#endif
			return matrixf::zeros<_dim, _dim>();
		}
		if (abs_max_row != i) {
			float* ptr_i = ext_mat[i];
			float* ptr_max = ext_mat[abs_max_row];
			for(int k = 0; k < 2 * _dim; ++k) {  // swap rows
				const float temp = ptr_i[k];
				ptr_i[k] = ptr_max[k];
				ptr_max[k] = temp;
			}
		}
		// normalize pivot row
		float k = 1.f / ext_mat[i][i];
		for (int col = i; col < 2 * _dim; ++col) ext_mat[i][col] *= k;
		// eliminate other rows
		for (int row = 0; row < _dim; ++row) {
			if (row == i) continue;
			float factor = ext_mat[row][i];
			for (int col = i; col < 2 * _dim; ++col) {
				ext_mat[row][col] -= factor * ext_mat[i][col];
			}
		}
	}
#if DEBUG_MATH == 1
	s = ARM_MATH_SUCCESS;
#endif
	Matrixf<_dim, _dim> res;
	for (int i = 0; i < _dim; ++i) {
		memcpy(res[i], &ext_mat[i][_dim], _dim * sizeof(float));
	}
	return res;
}

}  // namespace matrixf

namespace vector3f {

// hat of vector
inline Matrixf<3, 3> hat(Matrixf<3, 1> vec) {
	Matrixf<3, 3> res;
	res[0][0] = 0;
	res[0][1] = -vec[2][0];
	res[0][2] = vec[1][0];
	res[1][0] = vec[2][0];
	res[1][1] = 0;
	res[1][2] = -vec[0][0];
	res[2][0] = -vec[1][0];
	res[2][1] = vec[0][0];
	res[2][2] = 0;
	return res;
}

// cross product
	inline Matrixf<3, 1> cross(const Matrixf<3, 1>& vec1, const Matrixf<3, 1>& vec2) {
	float data[3];
	data[0] = vec1[1][0] * vec2[2][0] - vec1[2][0] * vec2[1][0];
	data[1] = vec1[2][0] * vec2[0][0] - vec1[0][0] * vec2[2][0];
	data[2] = vec1[0][0] * vec2[1][0] - vec1[1][0] * vec2[0][0];
	return Matrixf<3, 1>(data);
}

}  // namespace vector3f
