/// Copyright (c) 2016 AutelRobotics. All Rights Reserved
///
/// \file data_define.h
///
/// \Definition : Many classes are defined in this file, such as AuMat.
///
///  \author XipengCui
///
#ifndef AUTEL_CV_PACKAGE_DATA_DEFINE_H_
#define AUTEL_CV_PACKAGE_DATA_DEFINE_H_

#include <cstdlib>
#include <cfloat>
#include <cstdio>
#include <cmath>
#include <cstring>

namespace autel{

namespace computer_vision{

typedef unsigned char uchar;
typedef unsigned short ushort;
#define AuMax(a,b)    (((a) > (b)) ? (a) : (b))
#define AuMin(a,b)    (((a) < (b)) ? (a) : (b))
#define AU_PI   3.1415926535897932384626433832795

/// \desc Define the primitive data types
enum AuFormat{
  //unsigned char
  AU_8U  = 0,
  // char
  AU_8S  = 1,
  // unsigned short
  AU_16U = 2,
  // short
  AU_16S = 3,
  // int
  AU_32S = 4,
  // float
  AU_32F = 5,
  // double
  AU_64F = 6
};

///
/// \class Define 2-dimensional point
///
template <typename T>
class AuPoint{
public:
  T x_, y_;
};

/// 
/// size of two dimension
///
class AuSize{
public:
  int width_, height_;
};

///
/// rect class
///
template <typename T>
class AuRect{
public:
  //< the top-left corner, as well as width and height of the rectangle
  T x_, y_, width_, height_;
};

///
/// range class
///
class AuRange{
public:
  int start_, end_;
};

///
/// complex class
///
template <typename T>
class AuComplex{
public:
  //< the real and the imaginary parts
  T real_, imaginary_;
};

///
/// \class AuMat is a fundamental class structure. 
///
class AuMat{
public:
  //channes
  int channels_;
  //data type
  int depth_;
  //dimension
  int dims_;
  //height, width
  int rows_, cols_;
  //pointer to the data
  unsigned char* data_;
  //[0]: width step; [1]: Bytes of every pixel
  unsigned int step_[2];
  // no used
  unsigned int size_[2];
  // whether it gets memory in constructor function or copy function
  char is_getMem_;

  ///
  /// Default constructor.
  ///
  AuMat();

  ///
  /// Constructor.
  ///
  /// \param[in] rows rows of matrix
  /// \param[in] cols column of matrix
  /// \param[in] cn channels of matrix
  /// \param[in] depth data type: AU_8U, AU_8S, AU_16U, AU_16S, AU_32S, AU_32F, AU_64F
  ///
  AuMat(int rows, int cols, int cn, int depth);

  ///
  /// Constructor.
  ///
  /// \param[in] rows rows of matrix
  /// \param[in] cols column of matrix
  /// \param[in] cn channels of matrix
  /// \param[in] depth data type: AU_8U, AU_8S, AU_16U, AU_16S, AU_32S, AU_32F, AU_64F
  /// \param[in] data pointer to data
  ///
  AuMat(int rows, int cols, int cn, int depth, unsigned char*_data);

  ///
  /// Default destructor.
  ///
  ~AuMat();

  ///
  /// Copy constructor.
  ///
  /// \param[in] kmat matrix is copyed
  ///
  AuMat(const AuMat& kMat);

  ///
  /// Split an image into three different images, such as: RGBRGB --> RR...GG...BB...
  ///
  AuMat Split();

  ///
  /// Merge three images in one channel into one with three channels, such as: RR...GG...BB... ---> RGB...RGB
  ///
  AuMat Merge();

  ///
  /// Matrix multiplication.
  ///
  /// \param[in] kMatB right operator: such as: AuMat mat_c = mat_a.Mul(kMatB);
  ///
  AuMat Mul(const AuMat& kMatB);

  ///
  /// matrix division, it need inverse function.
  ///
  /// \param[in] kmat_b right operator: such as: AuMat mat_c = mat_a.Div(mat_b);
  ///
  AuMat Div(AuMat& mat_b);

  ///
  /// matrix subtraction.
  ///
  /// \param[in] kMatB right operator: such as: AuMat mat_c = mat_a.Sub(kMatB);
  ///
  AuMat Sub(const AuMat& kMatB);

  ///
  /// matrix addition.
  ///
  /// \param[in] kMatB right operator: such as: AuMat mat_c = mat_a.Add(kMatB);
  ///
  AuMat Add(const AuMat& kMatB);

  ///
  /// Compute the inverse of a matrix.
  ///
  AuMat Inv();

  ///
  /// Compute the determinant of a matrix. The matrix should be a
  /// square matrix, otherwise return 0;
  ///
  double Det();

  ///
  /// Compute the norm of matrix. such as: L1: sum of absolute value of every pixel; L2: sum of square.
  ///
  /// \param[in] p p norm: p >= 1
  ///
  double Norm(int p = 2);

  ///
  /// Compute the transpose of matrix.
  ///
  AuMat Trans();

  ///
  /// Get maximum element in matrix.
  ///
  /// \param[in] point coordinate of max element
  ///
  double Max(AuPoint<int>* point = NULL);

  ///
  /// Get minimum element in matrix.
  ///
  /// \param[in] point coordinate of minimum element
  ///
  double Min(AuPoint<int>* point = NULL);

  ///
  /// Operator '+', it is same with Add function.
  ///
  /// \param[in] kMatB right operator: such as: AuMat mat_c = mat_a + kMatB;
  ///
  AuMat operator+(const AuMat& kMatB);

  ///
  /// Operator '-', it is same with Sub function
  ///
  /// \param[in] kMatB right operator: such as: AuMat mat_c = mat_a - kMatB;
  ///
  AuMat operator-(const AuMat& kMatB);

  ///
  /// Operator '*' based on element-wise.
  ///
  /// \param[in] kMatB right operator: such as: AuMat mat_c = mat_a * kMatB;
  ///
  AuMat operator*(const AuMat& kMatB);

  ///
  /// Operator '/' based on element-wise.
  ///
  /// \param[in] kMatB right operator: such as: AuMat mat_c = mat_a / kMatB;
  ///
  AuMat operator/(const AuMat& kMatB);

  ///
  /// Operator '='.
  ///
  /// \param[in] kMatB assigment matrix
  ///
  AuMat& operator=(const AuMat& kMatB);

};

///
/// class TermCriteria
///
class AuTermCriteria{
  // the maximum number of iterations/elements
  int maxCount_;
  // the desired accuracy
  double epsilon_;
} ;

///
/// Locates matrix header within a parent matrix.
///
/// \param[in] m matrix
/// \param[out] wholeSize Output parameter that contains the size of the whole matrix containing m as a part
/// \param[out] ofs Output parameter that contains an offset of m inside the whole matrix
///
void locateROI(AuMat*m, AuSize* wholeSize, AuPoint<int>* ofs);

///
/// Get roi from image, don't copy any data.
///
/// \param[in] m matrix
/// \param[in] roi patch of image
/// \param[out] dst roi of image
///
AuMat getROI(AuMat m, AuRect<int>* roi);

///
/// Moves/resizes the current matrix ROI inside the parent matrix.
///
/// \param[in] m matrix
/// \param[in] dtop Shift of the top submatrix boundary upwards.
/// \param[in] dbottom Shift of the bottom submatrix boundary downwards.
/// \param[in] dleft Shift of the left submatrix boundary to the left.
/// \param[in] dright Shift of the right submatrix boundary to the right.
///
void adjustROI(AuMat* m, int dtop, int dbottom, int dleft, int dright);

} // namespace computer_vision

}// namespace autel

#endif

