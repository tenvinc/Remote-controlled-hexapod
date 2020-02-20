#ifndef MATRIXT_H
#define MATRIXT_H

#define mtx_type double

#include "Arduino.h"

template <class Matrix>
struct Minor // class to avoid having to create matrices when recursing
{
  typedef typename Matrix::elem_t type_t;
  const Matrix parent;
  int i, j; // to store location of which row and col to skip

  Minor<Matrix>(const Matrix &obj, int row, int col) : parent(obj), i(row), j(col) {}

  type_t &operator()(int row, int col) const
  {
    if (row >= i)
      row++;
    if (col >= j)
      col++;

    return parent(row, col);
  }
};

template <int rows, int cols = 1, class ElemT = float>
class MatrixT
{
public:
  typedef ElemT elem_t;
  const static int Rows = rows;
  const static int Cols = cols;

  MatrixT<rows, cols, ElemT>() {}
  MatrixT<rows, cols, ElemT>(const MatrixT<rows, cols, ElemT> &obj) { *this = obj; }
  template <int rows2, int cols2>
  MatrixT<rows, cols, ElemT>(const Minor<MatrixT<rows2, cols2, ElemT>> m2)
  {
    for (int i = 0; i < rows; i++)
    {
      for (int j = 0; j < cols; j++)
      {
        (*this)(i, j) = m2(i, j);
      }
    }
  }
  template <typename... ARGS>
  MatrixT(ARGS... args) { FillRowMajor(args...); }
  template <typename... TAIL>
  void FillRowMajor(ElemT head, TAIL... tail);
  void FillRowMajor() {} // To stop the recursion

  // Assignment
  MatrixT<rows, cols, ElemT> &operator=(const MatrixT<rows, cols, ElemT> &obj);
  MatrixT<rows, cols, ElemT> &operator=(ElemT arr[rows][cols]);
  MatrixT<rows, cols, ElemT> &operator=(ElemT arr[rows * cols]);

  // Addition, Subtraction, Multiplication
  const MatrixT<rows, cols, ElemT> operator+(const MatrixT<rows, cols, ElemT> &obj) const;
  MatrixT<rows, cols, ElemT> &operator+=(const MatrixT<rows, cols, ElemT> &obj);
  const MatrixT<rows, cols, ElemT> operator+(ElemT scalar) const;
  MatrixT<rows, cols, ElemT> &operator+=(ElemT scalar);

  const MatrixT<rows, cols, ElemT> operator-(const MatrixT<rows, cols, ElemT> &obj) const;
  MatrixT<rows, cols, ElemT> &operator-=(const MatrixT<rows, cols, ElemT> &obj);
  const MatrixT<rows, cols, ElemT> operator-(const ElemT scalar) const;
  MatrixT<rows, cols, ElemT> &operator-=(ElemT scalar);

  template <int operandCols>
  const MatrixT<rows, operandCols, ElemT> operator*(const MatrixT<cols, operandCols, ElemT> &obj) const;
  MatrixT<rows, cols, ElemT> &operator*=(const MatrixT<rows, cols, ElemT> &obj); // strictly for square matrices
  const MatrixT<rows, cols, ElemT> operator*(const ElemT scalar) const;
  MatrixT<rows, cols, ElemT> &operator*=(ElemT scalar);

  MatrixT<rows, cols, ElemT> &operator-();
  // Inverse
  MatrixT<rows, cols, ElemT> invert(int *res) const;
  // Determinant
  ElemT det(void) const;
  // Transpose
  MatrixT<cols, rows, ElemT> transpose() const;

  // Indexing
  ElemT &operator()(int row, int col) const;
  void print() const;
  void setNull() { isNull = true; }

private:
  mutable elem_t m[rows * cols] = {0};
  bool isNull = false;
};

template <int dim, class ElemT = float>
const MatrixT<dim, dim, ElemT> eye()
{
  MatrixT<dim, dim, ElemT> ret;
  for (int n = 0; n < dim; n++)
  {
    ret(n, n) = 1;
  }
  return ret;
}

///////////////////////////////////////////////////////// Implementation ///////////////////////////////////////////////////////////////////////////////

template <int rows, int cols, class ElemT>
template <typename... TAIL>
void MatrixT<rows, cols, ElemT>::FillRowMajor(ElemT head, TAIL... tail)
{
  static_assert(rows * cols > sizeof...(TAIL), "Too many arguments passed to FillRowMajor");
  (*this)((rows * cols - sizeof...(TAIL) - 1) / cols, (rows * cols - sizeof...(TAIL) - 1) % cols) = head;
  FillRowMajor(tail...);
}

template <int rows, int cols, class ElemT>
ElemT &MatrixT<rows, cols, ElemT>::operator()(int row, int col) const
{
  // if (row > Rows || col > Cols || row < 0 || col < 0) return;
  return m[row * Cols + col];
}

// Assignment
template <int rows, int cols, class ElemT>
MatrixT<rows, cols, ElemT> &MatrixT<rows, cols, ElemT>::operator=(const MatrixT<rows, cols, ElemT> &obj)
{
  for (int i = 0; i < rows; i++)
    for (int j = 0; j < cols; j++)
      (*this)(i, j) = obj(i, j);

  return *this;
}

template <int rows, int cols, class ElemT>
MatrixT<rows, cols, ElemT> &MatrixT<rows, cols, ElemT>::operator=(ElemT arr[rows][cols])
{
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      (*this)(i, j) = arr[i][j];
    }
  }
  return *this;
}

template <int rows, int cols, class ElemT>
MatrixT<rows, cols, ElemT> &MatrixT<rows, cols, ElemT>::operator=(ElemT arr[rows * cols])
{
  for (int n = 0; n < rows * cols; n++)
  {
    int i = n / cols;
    int j = n % cols;
    (*this)(i, j) = arr[n];
  }
}

///////////////////////////////////////////////////// Addition //////////////////////////////////////////////////////////////
template <int rows, int cols, class ElemT>
const MatrixT<rows, cols, ElemT> MatrixT<rows, cols, ElemT>::operator+(const MatrixT<rows, cols, ElemT> &obj) const
{
  MatrixT<rows, cols, ElemT> res = (*this);
  res += obj;
  return res;
}

template <int rows, int cols, class ElemT>
MatrixT<rows, cols, ElemT> &MatrixT<rows, cols, ElemT>::operator+=(const MatrixT<rows, cols, ElemT> &obj)
{
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      (*this)(i, j) += obj(i, j);
    }
  }
  return *this;
}

template <int rows, int cols, class ElemT>
const MatrixT<rows, cols, ElemT> MatrixT<rows, cols, ElemT>::operator+(ElemT scalar) const
{
  MatrixT<rows, cols, ElemT> res;
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      res(i, j) = (*this)(i, j) + scalar;
    }
  }
  return res;
}

template <int rows, int cols, class ElemT>
MatrixT<rows, cols, ElemT> &MatrixT<rows, cols, ElemT>::operator+=(ElemT scalar)
{
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      (*this)(i, j) += scalar;
    }
  }
  return *this;
}

///////////////////////////////////////////////////// Subtraction //////////////////////////////////////////////////////////////

template <int rows, int cols, class ElemT>
const MatrixT<rows, cols, ElemT> MatrixT<rows, cols, ElemT>::operator-(const MatrixT<rows, cols, ElemT> &obj) const
{
  MatrixT<rows, cols, ElemT> res = (*this);
  res -= obj;
  return res;
}

template <int rows, int cols, class ElemT>
MatrixT<rows, cols, ElemT> &MatrixT<rows, cols, ElemT>::operator-=(const MatrixT<rows, cols, ElemT> &obj)
{
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      (*this)(i, j) -= obj(i, j);
    }
  }
  return *this;
}

template <int rows, int cols, class ElemT>
const MatrixT<rows, cols, ElemT> MatrixT<rows, cols, ElemT>::operator-(ElemT scalar) const
{
  MatrixT<rows, cols, ElemT> res;
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      res(i, j) = (*this)(i, j) - scalar;
    }
  }
  return res;
}

template <int rows, int cols, class ElemT>
MatrixT<rows, cols, ElemT> &MatrixT<rows, cols, ElemT>::operator-=(ElemT scalar)
{
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      (*this)(i, j) -= scalar;
    }
  }
  return *this;
}

///////////////////////////////////////////////////// Multiplication //////////////////////////////////////////////////////////////
template <int rows, int cols, class ElemT>
template <int operandCols>
const MatrixT<rows, operandCols, ElemT> MatrixT<rows, cols, ElemT>::operator*(const MatrixT<cols, operandCols, ElemT> &obj) const
{
  MatrixT<rows, operandCols, ElemT> res;
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < operandCols; j++)
    {
      for (int k = 0; k < cols; k++)
      {
        res(i, j) = res(i, j) + ((*this)(i, k) * obj(k, j));
      }
    }
  }
  return res; // gonna return a copy since the matrices we are dealing with are small
}

template <int rows, int cols, class ElemT>
// strictly for square matrices
MatrixT<rows, cols, ElemT> &MatrixT<rows, cols, ElemT>::operator*=(const MatrixT<rows, cols, ElemT> &obj)
{
  if (rows != cols)
  {
    (*this).setNull();
    return (*this);
  }
  int dim = rows;
  MatrixT<rows, cols, ElemT> res;
  for (int i = 0; i < dim; i++)
  {
    for (int j = 0; j < dim; j++)
    {
      for (int k = 0; k < dim; k++)
      {
        res(i, j) += ((*this)(i, k) * obj(k, j));
      }
    }
  }
  *this = res;
  return (*this);
}

template <int rows, int cols, class ElemT>
const MatrixT<rows, cols, ElemT> MatrixT<rows, cols, ElemT>::operator*(ElemT scalar) const
{
  MatrixT<rows, cols, ElemT> res;
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      res(i, j) = (*this)(i, j) * scalar;
    }
  }
  return res;
}

template <int rows, int cols, class ElemT>
MatrixT<rows, cols, ElemT> &MatrixT<rows, cols, ElemT>::operator*=(ElemT scalar)
{
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      (*this)(i, j) *= scalar;
    }
  }
  return (*this);
}

///////////////////////////////////////////////////// Negation /////////////////////////////////////////////////////////////
template <int rows, int cols, class ElemT>
MatrixT<rows, cols, ElemT> &MatrixT<rows, cols, ElemT>::operator-()
{
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      (*this)(i, j) = -(*this)(i, j);
    }
  }
  return (*this);
}
///////////////////////////////////////////////////// Inverse //////////////////////////////////////////////////////////////
template <int rows, int cols, class ElemT>
MatrixT<rows, cols, ElemT> MatrixT<rows, cols, ElemT>::invert(int *res) const
{
  MatrixT<rows, cols, ElemT> ret(*this);
  return Invert(ret, res);
}

// Taken from Tom Steward
// Matrix Inversion Routine - modified from code written by Charlie Matlack: http://playground.arduino.cc/Code/MatrixMath
// This function inverts a matrix based on the Gauss Jordan method. Specifically, it uses partial pivoting to improve numeric stability.
// The algorithm is drawn from those presented in NUMERICAL RECIPES: The Art of Scientific Computing.
template <int dim, class ElemT>
MatrixT<dim, dim, ElemT> &Invert(MatrixT<dim, dim, ElemT> &A, int *res)
{
  int pivrow;
  int pivrows[dim]; // keeps track of current pivot row and row swaps
  int i, j, k;
  ElemT tmp; // used for finding max value and making column swaps

  for (k = 0; k < dim; k++)
  {
    // find pivot row, the row with biggest entry in current column
    tmp = 0;
    for (i = k; i < dim; i++)
    {
      if (fabs(A(i, k)) >= tmp)
      {
        tmp = fabs(A(i, k));
        pivrow = i;
      }
    }

    // check for singular matrix
    if (A(pivrow, k) == 0.0f)
      if (res)
        *res = 1;

    // Execute pivot (row swap) if needed
    if (pivrow != k)
    {
      // swap row k with pivrow
      for (j = 0; j < dim; j++)
      {
        tmp = A(k, j);
        A(k, j) = A(pivrow, j);
        A(pivrow, j) = tmp;
      }
    }
    pivrows[k] = pivrow; // record row swap (even if no swap happened)

    tmp = 1.0f / A(k, k); // invert pivot element
    A(k, k) = 1.0f;       // This element of input matrix becomes result matrix

    // Perform row reduction (divide every element by pivot)
    for (j = 0; j < dim; j++)
      A(k, j) = A(k, j) * tmp;

    // Now eliminate all other entries in this column
    for (i = 0; i < dim; i++)
    {
      if (i != k)
      {
        tmp = A(i, k);
        A(i, k) = 0.0f; // The other place where in matrix becomes result mat

        for (j = 0; j < dim; j++)
          A(i, j) = A(i, j) - A(k, j) * tmp;
      }
    }
  }

  // Done, now need to undo pivot row swaps by doing column swaps in reverse order
  for (k = dim - 1; k >= 0; k--)
  {
    if (pivrows[k] != k)
    {
      for (i = 0; i < dim; i++)
      {
        tmp = A(i, k);
        A(i, k) = A(i, pivrows[k]);
        A(i, pivrows[k]) = tmp;
      }
    }
  }

  if (res)
    *res = 0;

  return A;
}

///////////////////////////////////////////////////// Transpose //////////////////////////////////////////////////////////////
template <int rows, int cols, class ElemT>
MatrixT<cols, rows, ElemT> MatrixT<rows, cols, ElemT>::transpose() const
{
  MatrixT<cols, rows, ElemT> ret;
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      ret(j, i) = (*this)(i, j);
    }
  }
  return ret;
}

///////////////////////////////////////////////////// Determinant //////////////////////////////////////////////////////////////
template <int rows, int cols, class ElemT>
ElemT MatrixT<rows, cols, ElemT>::det(void) const
{
  return Determinant(*this);
}

template <int dim, class ElemT>
ElemT Determinant(const MatrixT<dim, dim, ElemT> &A)
{
  ElemT det = 0;

  // Add the determinants of all the minors
  for (int i = 0; i < dim; i++)
  {
    Minor<MatrixT<dim, dim, ElemT>> tmp(A, i, 0);
    MatrixT<dim - 1, dim - 1, ElemT> m(tmp);

    if (i % 2)
      det -= Determinant(m) * A(i, 0);
    else
      det += Determinant(m) * A(i, 0);
    Serial.print(dim);
    Serial.print(", ");
    Serial.print(i);
    Serial.print(", ");
    Serial.println(det);
  }

  return det;
}

template <class ElemT>
ElemT Determinant(MatrixT<2, 2, ElemT> &A)
{
  return A(0, 0) * A(1, 1) - A(1, 0) * A(0, 1);
}

// Debugging
template <int rows, int cols, class ElemT>
void MatrixT<rows, cols, ElemT>::print() const
{
  Serial.print("(");
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      Serial.print((*this)(i, j), 10);
      Serial.print(", ");
    }
    Serial.println("");
  }
  Serial.println(")");
}

#endif
