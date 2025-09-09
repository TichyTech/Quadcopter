#ifndef ALGEBRA
#define ALGEBRA

#include "definitions.h"

/**
 * Create a diagonal matrix from a vector of values.
 */
template <typename Derived, int rows, typename dtype>
inline Matrix<rows, rows> diag_matrix(const MatrixBase<Derived, rows, 1, dtype>& entries){
  Matrix<rows, rows> A;
  A.Fill(0);
  for (int i = 0; i < rows; i++) A(i, i) = entries(i);
  return A;
}

/**
*Map angle from [-360, 360] to [-180, 180].
*/
inline float constrain_angle(float angle){
  return fmod(angle + 540, 360) - 180;  // signed angle difference
}

/**
* Dot product of two Vectors.
*/
template <typename Derived0, typename Derived1, int rows, typename dtype>
inline float dot(const MatrixBase<Derived0, rows, 1, dtype>& a, const MatrixBase<Derived1, rows, 1, dtype>& b){
  float sum = 0;
  for (int i = 0; i < rows; i++) sum += a(i, 0)*b(i, 0);
  return sum;
}

/**
* The norm of a Vector
*/
template <typename Derived, int rows, typename dtype>
inline float norm(const MatrixBase<Derived, rows, 1, dtype>& v) {
  return sqrt(dot(v,v));
}

/**
 * Sum of matrix elements
 */
template <typename Derived, int rows, int cols, typename dtype>
inline float sum(const MatrixBase<Derived, rows, cols, dtype>& v){
  float sum = 0;
  for (int i = 0; i < rows; i++){
    for (int j = 0; j < cols; j++) {
      sum += v(i, j);
    }
  }
  return sum;
}

/**
 * find the maximum entry in a matrix
 */
template <int Rows, int Cols>
inline float max_norm(const Matrix<Rows,Cols>& A){
  float max_float = 0;

  for (int i = 0; i < Rows; i++){
    for (int j = 0; j < Cols; j++){
      if (A(i,j) > max_float) max_float = A(i,j);
    }
  }

  return max_float;
}

/**
 * find the minimum entry in a matrix
 */
template <int Rows, int Cols>
inline float min_norm(const Matrix<Cols,Rows>& A){
  float min_float = 0;

  for (int i = 0; i < Cols; i++){
    for (int j = 0; j < Rows; j++){
      if (A(i,j) < min_float) min_float = A(i,j);
    }
  }

  return min_float;
}

/**
* The determinant of a 3x3 matrix.
*/
inline float det(const Matrix3& A){
  float sum = A(0,0)*(A(1,1)*A(2,2) - A(1,2)*A(2,1));
  sum += A(0,1)*(A(1,2)*A(2,0) - A(1,0)*A(2,2));
  sum += A(0,2)*(A(1,0)*A(2,1) - A(1,1)*A(2,0));
  return sum;
}

/**
* Divide a Vector by its norm.
*/
template <typename Derived, int rows, typename dtype>
inline Matrix<rows> normalize(const MatrixBase<Derived, rows, 1, dtype>& v){
  float mult = 1.0f/(norm(v) + 1e-6f);
  return v*mult;
}

/**
 * Return a skew symmetric matrix created from a Vector3 variable $[v]_x$.
 */
inline Matrix3 skew(const Vector3& v){  // skew symmetric matrix 
  Matrix3 S = {0, -v(2), v(1),
               v(2), 0, -v(0),
               -v(1), v(0), 0};
  return S;
}

/** 
 * Compute the cross product of two 3D vectors
 */
inline Vector3 cross(const Vector3& v1, const Vector3& v2){
  Vector3 prod;
  prod(0) = v1(1)*v2(2) - v1(2)*v2(1);
  prod(1) = v1(2)*v2(0) - v1(0)*v2(2);
  prod(2) = v1(0)*v2(1) - v1(1)*v2(0);
  return prod;
}

/**
 * Computes entry-wise product between two vectors
 */
template <typename Derived, int rows, typename dtype>
inline Matrix<rows> product(const MatrixBase<Derived, rows, 1, dtype>& v1, const MatrixBase<Derived, rows, 1, dtype>& v2){
  Matrix<rows> prod;
  for(int i = 0; i < rows; i++) prod(i) = v1(i) * v2(i);
  return prod;
}

/**
 * Clamps the vector values between two limiting floats
 */
template <typename Derived, int rows, typename dtype>
inline Matrix<rows> clamp(const MatrixBase<Derived, rows, 1, dtype>& v, float minimum, float maximum){
  Matrix<rows> out = v;
  for(int i = 0; i < rows; i++) out(i) = constrain(v(i), minimum, maximum);
  return out;
}

/**
 * Clamps the vector values between two limiting vectors
 */
template <typename Derived, int rows, typename dtype>
inline Matrix<rows> clamp(const MatrixBase<Derived, rows, 1, dtype>& v, 
                          const MatrixBase<Derived, rows, 1, dtype>& v1, 
                          const MatrixBase<Derived, rows, 1, dtype>& v2)
{
  Matrix<rows, 1> out = v;
  for(int i = 0; i < rows; i++){
    out(i) = constrain(v(i), v1(i), v2(i));
  }
  return out;
}

/**
 * Return the trace of a 3x3 Matrix
*/
inline float trace(const Matrix3& A){
  return A(0,0) + A(1,1) + A(2,2);
}

/**
 * Return the outer product $vv^T$ of a vector v.
 */
inline Matrix3 outer(const Vector3& v){  // skew symmetric matrix 
  Matrix3 vvT = v*(~v);
  return vvT;
}


/**
 * This function implements the Rodriguez formula for a conversion from the axis angle representation to a rotation matrix representation.
 */
inline Matrix3 rodriguez(const Vector3& axis, float angle){
  Matrix3 K = skew(axis);
  Matrix3 R = I_3 + K*sin(angle) + K*K*(1 - cos(angle));  // Rodriguez rotation formula
  return R;
}

/**
 * Normalize the columns of a 3x3 matrix.
 */
inline Matrix3 normalize_columns(const Matrix3& A){
  Matrix<3, 1> a1,a2,a3;
  a1 = A.Column(0);
  a2 = A.Column(1);
  a3 = A.Column(2);
  a1 = normalize(a1);
  a2 = normalize(a2);
  a3 = normalize(a3);
  return a1 || a2 || a3;
}

/**
* This implements some clever way to normalize a 3x3 matrix. 
*/
inline Matrix3 normalize_matrix(const Matrix3& A){
  Vector3 a1,a2,a3;
  float e = dot(A.Column(0), A.Column(1));
  a1 = A.Column(0) - A.Column(1)*e/2.0f;
  a2 = A.Column(1) - A.Column(0)*e/2.0f;
  a3 = skew(A.Column(0)) * A.Column(1);
  return normalize_columns(a1 || a2 || a3);
}

/**
 * Transform quaternion to the equivalent rotation matrix 
 */
inline Matrix3 quat2R(const Vector4& q){
  float q0 = q(0);  // scalar part
  Vector3 v = q.Submatrix<3,1>(1,0);  // vector part
  Matrix3 v_x = skew(v);  // skew symmetric from v
  Matrix3 R = (q0*q0)*I_3 + v*~v + (2.0f*q0)*v_x + v_x*v_x; 
  return R;
}

/**
 * Create the quaternion product of two quaternions q1q2
 */
inline Vector4 quat_mult(const Vector4& q1, const Vector4& q2){
  float w1 = q1(0);
  Vector3 v1 = q1.Submatrix<3,1>(1,0);
  float w2 = q2(0);
  Vector3 v2 = q2.Submatrix<3,1>(1,0);  

  Vector4 prod;
  prod(0) = w1 * w2 - dot(v1, v2);
  prod.Submatrix<3,1>(1,0) = w1*v2 + w2*v1 + cross(v1, v2);
  return prod;
}

/** 
 * Create a quaternion representation from roll pitch yaw angles
*/
inline Vector4 rpy2quat(const AttReference& att_ref){
  float roll = att_ref.roll * TO_RAD / 2;
  float pitch = att_ref.pitch * TO_RAD / 2;
  float yaw = att_ref.yaw * TO_RAD / 2;

  Vector4 q_r = {cos(roll), sin(roll), 0, 0};
  Vector4 q_p = {cos(pitch), 0, sin(pitch), 0};
  Vector4 q_y = {cos(yaw), 0, 0, sin(yaw)};
  return quat_mult(q_y, quat_mult(q_p, q_r));
}

#endif
