/// Copyright (c) 2016 AutelRobotics. All Rights Reserved
///
/// \file data_define.cpp
///
/// \Definition : Many classes are defined in this file, such as AuMat.
///
///  \author XipengCui
///  \Date : Created on: Feb 20, 2017
///
#include "../include/data_define.h"
#include <algorithm>
#include <limits>

namespace autel{

namespace computer_vision{

#define Lambda 0.000001

AuMat::AuMat()
{
  this->rows_ = 0;
  this->cols_ = 0;
  this->channels_ = 0;
  this->depth_ = 0;
  this->data_ = 0;
  this->is_getMem_ = 0;
  this->step_[1] = 0;
  this->step_[0] = 0;
  this->dims_ = 0;
}

AuMat::AuMat(int rows, int cols, int cn, int depth, uchar* data)
{
  this->rows_ = rows;
  this->cols_ = cols;
  this->channels_ = cn;
  this->dims_ = 2;
  this->depth_ = depth;
  this->data_ = data;
  this->is_getMem_ = 0;
  this->step_[1] = this->channels_ * this->depth_;
  this->step_[0] = this->step_[1] * this->cols_;
}

AuMat::AuMat(int rows, int cols, int cn, int depth)
{
  this->rows_ = rows;
  this->cols_ = cols;
  this->channels_ = cn; //channels
  this->dims_ = 2;
  this->depth_ = depth;//bytes of every pixel;
  if (depth <= 1)//char or unsigned char
  {
    this->data_ = new unsigned char[rows*cols*this->channels_];
    this->step_[1] = this->channels_;
  }
  else if (depth <= 3) // short or unsigned short
  {
    this->data_ = new unsigned char[rows*cols*this->channels_*sizeof(short)];
    this->step_[1] = this->channels_*sizeof(short);
  }
  else if (depth <= 5) // int or float
  {
    this->data_ = new unsigned char[rows*cols*this->channels_*sizeof(int)];
    this->step_[1] = this->channels_*sizeof(int);
  }
  else // double
  {
    this->data_ = new unsigned char[rows*cols*this->channels_*sizeof(double)];
    this->step_[1] = this->channels_*sizeof(double);
  }
  this->is_getMem_ = 1; //if using new function to get memory, this parameter is 1;
  this->step_[0] = this->step_[1] * this->cols_;
}
AuMat::~AuMat()
{
  if (this->is_getMem_ && this->data_ != NULL)
    delete [] this->data_;
}

AuMat::AuMat(const AuMat& kMat)
{
  this->channels_ = kMat.channels_;
  this->cols_ = kMat.cols_;
  this->rows_ = kMat.rows_;
  this->depth_ = kMat.depth_;
  this->dims_ = kMat.dims_;
  this->step_[0] = kMat.step_[0];
  this->step_[1] = kMat.step_[1];
  this->size_[0] = kMat.size_[0];
  this->size_[1] = kMat.size_[1];

  int depth = kMat.depth_;
  int rows = kMat.rows_;
  int cols = kMat.cols_;
  if (depth <= 1)//char or unsigned char
  {
    this->data_ = new unsigned char[rows*cols*this->channels_];
    memcpy(this->data_, kMat.data_, rows*cols*this->channels_);
  }
  else if (depth <= 3) // short or unsigned short
  {
    this->data_ = new unsigned char[rows*cols*this->channels_*sizeof(short)];
    memcpy(this->data_, kMat.data_, rows*cols*this->channels_*sizeof(short));
  }
  else if (depth <= 5) // int or float
  {
    this->data_ = new unsigned char[rows*cols*this->channels_*sizeof(int)];
    memcpy(this->data_, kMat.data_, rows*cols*this->channels_*sizeof(int));
  }
  else // double
  {
    this->data_ = new unsigned char[rows*cols*this->channels_*sizeof(double)];
    memcpy(this->data_, kMat.data_, rows*cols*this->channels_*sizeof(double));
  }
  this->is_getMem_ = 1; //if using new function to get memory, this parameter is 1;
}
///
/// Split one layout into three layout
///
/// \param[in] data_a input data: RGB...RGB...RGB
/// \param[in] w column of data
/// \param[in] h rows of data
/// \param[in] cn channels: RGB
/// \param[out] data_b output data
///
template <typename T>
inline void SplitData(T* data_a, T* data_b, int w, int h, int cn)
{
  

  if(data_a == NULL || data_b == NULL)
  {
    printf("NULL pointer in SplitData!\n");
    return;
  }
  int x,y,c;
  for(y = 0; y < h; ++y)
  {
    for(x = 0; x < w; ++x)
    {
      for(c = 0; c < cn; ++c)
      {
        data_b[y*w+x+c*w*h] = data_a[(y*w+x)*cn+c];
      }
    }
  }
}

AuMat AuMat::Split()
{
  AuMat mat_zero(0, 0, 0, 0, 0);
  if (this->cols_ <= 0 || this->rows_ <= 0 || this->data_ == NULL)
    return mat_zero;

  if(this->channels_ == 1)
    return *this;

  int h = this->rows_;
  int w = this->cols_;
  int cn = this->channels_;
  AuMat mat_c(h, w, cn, this->depth_);
  switch (this->depth_)
  {
  case 0:
  {
    //AU_8U: unsigned char
    uchar* ptr_a = this->data_;
    uchar* ptr_c = mat_c.data_;
    SplitData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  case 1:
  {
    //AU_8S: char
    char* ptr_a = (char*)this->data_;
    char* ptr_c = (char*)mat_c.data_;
    SplitData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  case 2:
  {
    //AU_16U: unsigned short
    unsigned short* ptr_a = (unsigned short*)this->data_;
    unsigned short* ptr_c = (unsigned short*)mat_c.data_;
    SplitData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  case 3:
  {
    //AU_16S: short
    short* ptr_a = (short*)this->data_;
    short* ptr_c = (short*)mat_c.data_;
    SplitData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  case 4:
  {
    //AU_32S: int
    int* ptr_a = (int*)this->data_;
    int* ptr_c = (int*)mat_c.data_;
    SplitData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  case 5:
  {
    //AU_32F: float
    float* ptr_a = (float*)this->data_;
    float* ptr_c = (float*)mat_c.data_;
    SplitData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  case 6:
  {
    //AU_64F: double
    double* ptr_a = (double*)this->data_;
    double* ptr_c = (double*)mat_c.data_;
    SplitData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  }
  return mat_c;
}

///
/// Merge three layout into one layout
///
/// \param[in] data_a input data: RGB...RGB...RGB
/// \param[in] w column of data
/// \param[in] h rows of data
/// \param[in] cn channels: RGB
/// \param[out] data_b output data
///
template <typename T>
inline void MergeData(T* data_a, T* data_b, int w, int h, int cn)
{
  if(data_a == NULL || data_b == NULL)
  {
    printf("NULL pointer in MergeData!\n");
    return;
  }
  int x,y,c;
  for(y = 0; y < h; ++y)
  {
    for(x = 0; x < w; ++x)
    {
      for(c = 0; c < cn; ++c)
      {
        data_b[(y*w+x)*cn+c] = data_a[y*w+x+c*w*h];
      }
    }
  }
}

AuMat AuMat::Merge()
{
  AuMat mat_zero(0, 0, 0, 0, 0);
  if (this->cols_ <= 0 || this->rows_ <= 0 || this->data_ == NULL)
    return mat_zero;

  if(this->channels_ == 1)
    return *this;

  int h = this->rows_;
  int w = this->cols_;
  int cn = this->channels_;
  AuMat mat_c(h, w, cn, this->depth_);
  switch (this->depth_)
  {
  case 0:
  {
    //AU_8U: unsigned char
    uchar* ptr_a = this->data_;
    uchar* ptr_c = mat_c.data_;
    MergeData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  case 1:
  {
    //AU_8S: char
    char* ptr_a = (char*)this->data_;
    char* ptr_c = (char*)mat_c.data_;
    MergeData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  case 2:
  {
    //AU_16U: unsigned short
    unsigned short* ptr_a = (unsigned short*)this->data_;
    unsigned short* ptr_c = (unsigned short*)mat_c.data_;
    MergeData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  case 3:
  {
    //AU_16S: short
    short* ptr_a = (short*)this->data_;
    short* ptr_c = (short*)mat_c.data_;
    MergeData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  case 4:
  {
    //AU_32S: int
    int* ptr_a = (int*)this->data_;
    int* ptr_c = (int*)mat_c.data_;
    MergeData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  case 5:
  {
    //AU_32F: float
    float* ptr_a = (float*)this->data_;
    float* ptr_c = (float*)mat_c.data_;
    MergeData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  case 6:
  {
    //AU_64F: double
    double* ptr_a = (double*)this->data_;
    double* ptr_c = (double*)mat_c.data_;
    MergeData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  }
  return mat_c;
}

///
/// Process dot product
///
/// \param[in] v_a data of matrix
/// \param[in] v_b data of matrix
/// \param[in] a_w column of v_a
/// \param[in] a_h rows of v_a
/// \param[in] b_w column of v_b
/// \param[in] b_h rows of v_b
/// \param[in] cn channels
/// \param[out] v_c result of dot product, whose size is a_h x b_w
///
template <typename T>
inline void DotProductVector(T* v_a, T* v_b, T* v_c, int a_w, int a_h, int b_w, int b_h, int cn)
{
  if(v_a == NULL || v_b == NULL || v_c == NULL)
  {
	printf("Null pointer in DotProductVector!!!\n");
	return;
  }
  int h = a_h; //height for v_c
  int w = b_w; //width for v_c
  int x, y, c, loop;

  for (y = 0; y < h; ++y)
  {
    for (x = 0; x < w; ++x)
    {
      for (c = 0; c < cn; ++c)
      {
        // calculate every point in v_c
        double sum_value = 0;
        for (loop = 0; loop < a_w; ++loop)
        {
          sum_value += v_a[(y*a_w + loop)*cn + c] * v_b[(loop*b_w + x)*cn + c];
        }
        v_c[(y*w + x)*cn + c] = sum_value;
      }// for(c = 0; c < cn; ++c)
    }//for (x = 0; x < w; ++x)
  }//(y = 0; y < h; ++y)
}

AuMat AuMat::Mul(const AuMat& kMatB)
{
  AuMat mat_zero(0, 0, 0, 0, 0);
  if (this->cols_ != kMatB.rows_ || this->data_ == NULL || kMatB.data_ == NULL ||
    this->depth_ != kMatB.depth_)
    return mat_zero;

  // (mxn).(nxm) = (mxm)
  int h = this->rows_;
  int w = kMatB.cols_;
  int cn = kMatB.channels_;
  AuMat mat_c(h, w, cn, kMatB.depth_);
  switch (this->depth_)
  {
  case 0:
  {
    //AU_8U: unsigned char
    uchar* ptr_a = this->data_;
    uchar* ptr_b = kMatB.data_;
    uchar* ptr_c = mat_c.data_;
    DotProductVector(ptr_a, ptr_b, ptr_c, this->cols_, this->rows_, kMatB.cols_, kMatB.rows_, cn);
    break;
  }
  case 1:
  {
    //AU_8S: char
    char* ptr_a = (char*)this->data_;
    char* ptr_b = (char*)kMatB.data_;
    char* ptr_c = (char*)mat_c.data_;
    DotProductVector(ptr_a, ptr_b, ptr_c, this->cols_, this->rows_, kMatB.cols_, kMatB.rows_, cn);
    break;
  }
  case 2:
  {
    //AU_16U: unsigned short
    unsigned short* ptr_a = (unsigned short*)this->data_;
    unsigned short* ptr_b = (unsigned short*)kMatB.data_;
    unsigned short* ptr_c = (unsigned short*)mat_c.data_;
    DotProductVector(ptr_a, ptr_b, ptr_c, this->cols_, this->rows_, kMatB.cols_, kMatB.rows_, cn);
    break;
  }
  case 3:
  {
    //AU_16S: short
    short* ptr_a = (short*)this->data_;
    short* ptr_b = (short*)kMatB.data_;
    short* ptr_c = (short*)mat_c.data_;
    DotProductVector(ptr_a, ptr_b, ptr_c, this->cols_, this->rows_, kMatB.cols_, kMatB.rows_, cn);
    break;
  }
  case 4:
  {
    //AU_32S: int
    int* ptr_a = (int*)this->data_;
    int* ptr_b = (int*)kMatB.data_;
    int* ptr_c = (int*)mat_c.data_;
    DotProductVector(ptr_a, ptr_b, ptr_c, this->cols_, this->rows_, kMatB.cols_, kMatB.rows_, cn);
    break;
  }
  case 5:
  {
    //AU_32F: float
    float* ptr_a = (float*)this->data_;
    float* ptr_b = (float*)kMatB.data_;
    float* ptr_c = (float*)mat_c.data_;
    DotProductVector(ptr_a, ptr_b, ptr_c, this->cols_, this->rows_, kMatB.cols_, kMatB.rows_, cn);
    break;
  }
  case 6:
  {
    //AU_64F: double
    double* ptr_a = (double*)this->data_;
    double* ptr_b = (double*)kMatB.data_;
    double* ptr_c = (double*)mat_c.data_;
    DotProductVector(ptr_a, ptr_b, ptr_c, this->cols_, this->rows_, kMatB.cols_, kMatB.rows_, cn);
    break;
  }
  }
  return mat_c;
}

AuMat AuMat::Div(AuMat& mat_b)
{
  AuMat mat_b_inv = mat_b.Inv();
  return this->Mul(mat_b_inv);
}
///
/// Matrix addition
///
/// \param[in] data_a data of matrix
/// \param[in] data_b data of matrix
/// \param[in] w column of matrix
/// \param[in] h rows of matrix
/// \param[in] cn channels
/// \param[out] data_c result of matrix addition
///
template <typename T>
inline void AddNumber(T* data_a, T* data_b, T* data_c, int w, int h, int cn)
{
  if(data_a == NULL || data_b == NULL || data_c == NULL)
  {
	printf("NULL pointer in AddNumber function!!!");
	return;
  }
  int x, y, c;
  for (y = 0; y < h; ++y)
  {
    for (x = 0; x < w; ++x)
    {
      for (c = 0; c < cn; ++c)
      {
        int idx = (y*w + x)*cn + c;
        data_c[idx] = data_a[idx] + data_b[idx];
      }
    }
  }
}

AuMat AuMat::Add(const AuMat& kMatB)
{
  AuMat mat_zero(0, 0, 0, 0, 0);
  if (this->rows_ != kMatB.rows_ || this->cols_ != kMatB.cols_ ||
    this->data_ == NULL || kMatB.data_ == NULL ||
    this->depth_ != kMatB.depth_)
    return mat_zero;

  int w = kMatB.cols_;
  int h = kMatB.rows_;
  int cn = kMatB.channels_;
  AuMat mat_c(h, w, cn, kMatB.depth_);
  switch (this->depth_)
  {
  case 0:
  {
    //AU_8U
    uchar* ptr_a = this->data_;
    uchar* ptr_b = kMatB.data_;
    uchar* ptr_c = mat_c.data_;
    AddNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 1:
  {
    //AU_8S
    char* ptr_a = (char*)this->data_;
    char* ptr_b = (char*)kMatB.data_;
    char* ptr_c = (char*)mat_c.data_;
    AddNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 2:
  {
    //AU_16U
    unsigned short* ptr_a = (unsigned short*)this->data_;
    unsigned short* ptr_b = (unsigned short*)kMatB.data_;
    unsigned short* ptr_c = (unsigned short*)mat_c.data_;
    AddNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 3:
  {
    //AU_16S
    short* ptr_a = (short*)this->data_;
    short* ptr_b = (short*)kMatB.data_;
    short* ptr_c = (short*)mat_c.data_;
    AddNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 4:
  {
    //AU_32S
    int* ptr_a = (int*)this->data_;
    int* ptr_b = (int*)kMatB.data_;
    int* ptr_c = (int*)mat_c.data_;
    AddNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 5:
  {
    //AU_32F
    float* ptr_a = (float*)this->data_;
    float* ptr_b = (float*)kMatB.data_;
    float* ptr_c = (float*)mat_c.data_;
    AddNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 6:
  {
    //AU_64F
    double* ptr_a = (double*)this->data_;
    double* ptr_b = (double*)kMatB.data_;
    double* ptr_c = (double*)mat_c.data_;
    AddNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  }
  return mat_c;
}
///
/// Matrix subtraction
///
/// \param[in] data_a data of matrix
/// \param[in] data_b data of matrix
/// \param[in] w column of matrix
/// \param[in] h rows of matrix
/// \param[in] cn channels
/// \param[out] data_c result of matrix subtraction
///
template <typename T>
inline void SubNumber(T* data_a, T* data_b, T* data_c, int w, int h, int cn)
{
  if(data_a == NULL || data_b == NULL || data_c == NULL)
  {
	printf("NULL pointer in AddNumber function!!!");
	return;
  }
  int x, y, c;
  for (y = 0; y < h; ++y)
  {
    for (x = 0; x < w; ++x)
    {
      for (c = 0; c < cn; ++c)
      {
        int idx = (y*w + x)*cn + c;
        data_c[idx] = data_a[idx] - data_b[idx];
      }
    }
  }
}

AuMat AuMat::Sub(const AuMat& kMatB)
{
  AuMat mat_zero(0, 0, 0, 0, 0);
  if (this->rows_ != kMatB.rows_ || this->cols_ != kMatB.cols_ ||
    this->data_ == NULL || kMatB.data_ == NULL ||
    this->depth_ != kMatB.depth_)
    return mat_zero;

  int w = kMatB.cols_;
  int h = kMatB.rows_;
  int cn = kMatB.channels_;
  AuMat mat_c(h, w, cn, kMatB.depth_);
  switch (this->depth_)
  {
  case 0:
  {
    uchar* ptr_a = this->data_;
    uchar* ptr_b = kMatB.data_;
    uchar* ptr_c = mat_c.data_;
    SubNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 1:
  {
    char* ptr_a = (char*)this->data_;
    char* ptr_b = (char*)kMatB.data_;
    char* ptr_c = (char*)mat_c.data_;
    SubNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 2:
  {
    unsigned short* ptr_a = (unsigned short*)this->data_;
    unsigned short* ptr_b = (unsigned short*)kMatB.data_;
    unsigned short* ptr_c = (unsigned short*)mat_c.data_;
    SubNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 3:
  {
    short* ptr_a = (short*)this->data_;
    short* ptr_b = (short*)kMatB.data_;
    short* ptr_c = (short*)mat_c.data_;
    SubNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 4:
  {
    int* ptr_a = (int*)this->data_;
    int* ptr_b = (int*)kMatB.data_;
    int* ptr_c = (int*)mat_c.data_;
    SubNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 5:
  {
    float* ptr_a = (float*)this->data_;
    float* ptr_b = (float*)kMatB.data_;
    float* ptr_c = (float*)mat_c.data_;
    SubNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 6:
  {
    double* ptr_a = (double*)this->data_;
    double* ptr_b = (double*)kMatB.data_;
    double* ptr_c = (double*)mat_c.data_;
    SubNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  }
  return mat_c;
}
///
/// LU decomposition
///
template<typename _Tp> static inline int
LUImpl(_Tp* A, size_t astep, int m, _Tp* b, size_t bstep, int n)
{
  int i, j, k, p = 1;
  astep /= sizeof(A[0]);
  bstep /= sizeof(b[0]);

  for (i = 0; i < m; i++)
  {
    k = i;

    for (j = i + 1; j < m; j++)
      if (std::abs(A[j*astep + i]) > std::abs(A[k*astep + i]))
        k = j;

    if (std::abs(A[k*astep + i]) < std::numeric_limits<_Tp>::epsilon())
      return 0;

    if (k != i)
    {
      for (j = i; j < m; j++)
        std::swap(A[i*astep + j], A[k*astep + j]);
      if (b)
        for (j = 0; j < n; j++)
          std::swap(b[i*bstep + j], b[k*bstep + j]);
      p = -p;
    }

    _Tp d = -1 / A[i*astep + i];

    for (j = i + 1; j < m; j++)
    {
      _Tp alpha = A[j*astep + i] * d;

      for (k = i + 1; k < m; k++)
        A[j*astep + k] += alpha*A[i*astep + k];

      if (b)
        for (k = 0; k < n; k++)
          b[j*bstep + k] += alpha*b[i*bstep + k];
    }

    A[i*astep + i] = -d;
  }

  if (b)
  {
    for (i = m - 1; i >= 0; i--)
      for (j = 0; j < n; j++)
      {
        _Tp s = b[i*bstep + j];
        for (k = i + 1; k < m; k++)
          s -= A[i*astep + k] * b[k*bstep + j];
        b[i*bstep + j] = s*A[i*astep + i];
      }
  }

  return p;
}

int LU(float* A, size_t astep, int m, float* b, size_t bstep, int n)
{
  return LUImpl(A, astep, m, b, bstep, n);
}

int LU(double* A, size_t astep, int m, double* b, size_t bstep, int n)
{
  return LUImpl(A, astep, m, b, bstep, n);
}

///
/// define brief functions
///
#define Sf( y, x ) ((float*)(srcdata + y*srcstep))[x]
#define Sd( y, x ) ((double*)(srcdata + y*srcstep))[x]
#define Df( y, x ) ((float*)(dstdata + y*dststep))[x]
#define Dd( y, x ) ((double*)(dstdata + y*dststep))[x]

#define det_2(m)   ((double)m(0,0)*m(1,1) - (double)m(0,1)*m(1,0))
#define det_3(m)   (m(0,0)*((double)m(1,1)*m(2,2) - (double)m(1,2)*m(2,1)) -  \
                    m(0,1)*((double)m(1,0)*m(2,2) - (double)m(1,2)*m(2,0)) +  \
                    m(0,2)*((double)m(1,0)*m(2,1) - (double)m(1,1)*m(2,0)))

///
/// Copy data form src to dst, whose size is w x h x cn
///
template <typename T>
inline void CopyData(T* src, float* dst, int w, int h, int cn)
{
  if(src == NULL || dst == NULL)
  {
	printf("NULL pointer in SetIdentityMat \n");
	return;
  }
  int x, y, c;
  for (y = 0; y < h; ++y)
  {
    for (x = 0; x < w; ++x)
    {
      for (c = 0; c < cn; ++c)
      {
        int idx = (y*w + x)*cn + c;
        dst[idx] = (float)src[idx];
      }
    }
  }
}

///
/// Set identity for matrix
///
template <typename T>
inline void SetIdentityMat(T* data, int w, int h, int cn)
{
  if(data == NULL)
  {
	printf("NULL pointer in SetIdentityMat \n");
	return;
  }
  int x, y, c;
  for (y = 0; y < h; ++y)
  {
    for (x = 0; x < w; ++x)
    {
      for (c = 0; c < cn; ++c)
      {
        if (x == y)
        {
          data[(y*w + x)*cn + c] = 1;
        }
        else
        {
          data[(y*w + x)*cn + c] = 0;
        }
      }
    }
  }
}

AuMat AuMat::Inv()
{
  AuMat mat_zero(0, 0, 0, 0, 0);
  if (this->rows_ == 0 || this->cols_ == 0 || this->data_ == NULL)
	return mat_zero;

  bool result = false;
  int cn = 1;
  int m = this->rows_, n = this->cols_;
  int depth = 5;
  if (this->depth_ == 6)
    depth = 6;
  AuMat dst(m, n, cn, depth); // output matrix

  AuMat src(m, n, cn, depth);// this->data_ --> src
  if (this->depth_ <= 5)
  {
    float* ptr_out = (float*)src.data_;
    switch (this->depth_)
    {
    case 0:
    {
      uchar* ptr_in = this->data_;
      CopyData(ptr_in, ptr_out, n, m, cn);
      break;
    }
    case 1:
    {
      char* ptr_in = (char*)this->data_;
      CopyData(ptr_in, ptr_out, n, m, cn);
      break;
    }
    case 2:
    {
      unsigned short* ptr_in = (unsigned short*)this->data_;
      CopyData(ptr_in, ptr_out, n, m, cn);
      break;
    }
    case 3:
    {
      short* ptr_in = (short*)this->data_;
      CopyData(ptr_in, ptr_out, n, m, cn);
      break;
    }
    case 4:
    {
      int* ptr_in = (int*)this->data_;
      CopyData(ptr_in, ptr_out, n, m, cn);
      break;
    }
    case 5:
    {
      float* ptr_in = (float*)this->data_;
      CopyData(ptr_in, ptr_out, n, m, cn);
      break;
    }
    }
  }
  else
  {
    int x, y, c;
    double* ptr_in = (double*)this->data_;
    double* ptr_out = (double*)src.data_;
    for (y = 0; y < m; ++y)
    {
      for (x = 0; x < n; ++x)
      {
        for (c = 0; c < cn; ++c)
        {
          int idx = (y*n + x)*cn + c;
          ptr_out[idx] = ptr_in[idx];
        }
      }
    }
  }

  if (n <= 3)
  {
    uchar* srcdata = src.data_;
    uchar* dstdata = dst.data_;
    size_t srcstep = src.step_[0];
    size_t dststep = dst.step_[0];

    if (n == 2)
    {
      if (this->depth_ <= 5)
      {
        double d = det_2(Sf);
        if (d != 0.)
        {
          result = true;
          d = 1. / d;

          double t0, t1;
          t0 = Sf(0, 0)*d;
          t1 = Sf(1, 1)*d;
          Df(1, 1) = (float)t0;
          Df(0, 0) = (float)t1;
          t0 = -Sf(0, 1)*d;
          t1 = -Sf(1, 0)*d;
          Df(0, 1) = (float)t0;
          Df(1, 0) = (float)t1;

        }
      }
      else if (this->depth_ == 6)
      {
        double d = det_2(Sd);
        if (d != 0.)
        {
          result = true;
          d = 1. / d;
          {
            double t0, t1;
            t0 = Sd(0, 0)*d;
            t1 = Sd(1, 1)*d;
            Dd(1, 1) = t0;
            Dd(0, 0) = t1;
            t0 = -Sd(0, 1)*d;
            t1 = -Sd(1, 0)*d;
            Dd(0, 1) = t0;
            Dd(1, 0) = t1;
          }
        }
      }
    }
    else if (n == 3)
    {
      if (this->depth_ <= 5)
      {
        double d = det_3(Sf);

        if (d != 0.)
        {
          double t[12];

          result = true;
          d = 1. / d;
          t[0] = (((double)Sf(1, 1) * Sf(2, 2) - (double)Sf(1, 2) * Sf(2, 1)) * d);
          t[1] = (((double)Sf(0, 2) * Sf(2, 1) - (double)Sf(0, 1) * Sf(2, 2)) * d);
          t[2] = (((double)Sf(0, 1) * Sf(1, 2) - (double)Sf(0, 2) * Sf(1, 1)) * d);

          t[3] = (((double)Sf(1, 2) * Sf(2, 0) - (double)Sf(1, 0) * Sf(2, 2)) * d);
          t[4] = (((double)Sf(0, 0) * Sf(2, 2) - (double)Sf(0, 2) * Sf(2, 0)) * d);
          t[5] = (((double)Sf(0, 2) * Sf(1, 0) - (double)Sf(0, 0) * Sf(1, 2)) * d);

          t[6] = (((double)Sf(1, 0) * Sf(2, 1) - (double)Sf(1, 1) * Sf(2, 0)) * d);
          t[7] = (((double)Sf(0, 1) * Sf(2, 0) - (double)Sf(0, 0) * Sf(2, 1)) * d);
          t[8] = (((double)Sf(0, 0) * Sf(1, 1) - (double)Sf(0, 1) * Sf(1, 0)) * d);

          Df(0, 0) = (float)t[0]; Df(0, 1) = (float)t[1]; Df(0, 2) = (float)t[2];
          Df(1, 0) = (float)t[3]; Df(1, 1) = (float)t[4]; Df(1, 2) = (float)t[5];
          Df(2, 0) = (float)t[6]; Df(2, 1) = (float)t[7]; Df(2, 2) = (float)t[8];
        }
      }
      else
      {
        double d = det_3(Sd);
        if (d != 0.)
        {
          result = true;
          d = 1. / d;
          double t[9];

          t[0] = (Sd(1, 1) * Sd(2, 2) - Sd(1, 2) * Sd(2, 1)) * d;
          t[1] = (Sd(0, 2) * Sd(2, 1) - Sd(0, 1) * Sd(2, 2)) * d;
          t[2] = (Sd(0, 1) * Sd(1, 2) - Sd(0, 2) * Sd(1, 1)) * d;

          t[3] = (Sd(1, 2) * Sd(2, 0) - Sd(1, 0) * Sd(2, 2)) * d;
          t[4] = (Sd(0, 0) * Sd(2, 2) - Sd(0, 2) * Sd(2, 0)) * d;
          t[5] = (Sd(0, 2) * Sd(1, 0) - Sd(0, 0) * Sd(1, 2)) * d;

          t[6] = (Sd(1, 0) * Sd(2, 1) - Sd(1, 1) * Sd(2, 0)) * d;
          t[7] = (Sd(0, 1) * Sd(2, 0) - Sd(0, 0) * Sd(2, 1)) * d;
          t[8] = (Sd(0, 0) * Sd(1, 1) - Sd(0, 1) * Sd(1, 0)) * d;

          Dd(0, 0) = t[0]; Dd(0, 1) = t[1]; Dd(0, 2) = t[2];
          Dd(1, 0) = t[3]; Dd(1, 1) = t[4]; Dd(1, 2) = t[5];
          Dd(2, 0) = t[6]; Dd(2, 1) = t[7]; Dd(2, 2) = t[8];
        }
      }
    }
    else
    {
      if (this->depth_ <= 5)
      {
        double d = Sf(0, 0);
        if (d != 0.)
        {
          result = true;
          Df(0, 0) = (float)(1. / d);
        }
      }
      else
      {
        double d = Sd(0, 0);
        if (d != 0.)
        {
          result = true;
          Dd(0, 0) = 1. / d;
        }
      }
    }
    if (!result)
    {
      if (this->depth_ <= 5)
        memset(dst.data_, 1, m*n*sizeof(float));
      else if (this->depth_ == 6)
        memset(dst.data_, 1, m*n*sizeof(double));
    }
    return dst;
  }
  //set identity and call LU
  if (this->depth_ <= 5)
  {
    float* ptr_dst = (float*)dst.data_;
    SetIdentityMat(ptr_dst, n, m, cn);
    result = LU((float*)src.data_, src.step_[0], n, ptr_dst, dst.step_[0], n) != 0;
  }
  else if (this->depth_ == 6)
  {
    double* ptr_dst = (double*)dst.data_;
    SetIdentityMat(ptr_dst, n, m, cn);
    result = LU((double*)src.data_, src.step_[0], n, ptr_dst, dst.step_[0], n) != 0;
  }

  if (!result)
  {
    if (this->depth_ <= 5)
      memset(dst.data_, 1, m*n*sizeof(float));
    else if (this->depth_ == 6)
      memset(dst.data_, 1, m*n*sizeof(double));
  }
  return dst;
}

double AuMat::Det()
{
  if (this->rows_ == 0 || this->cols_ == 0 || this->data_ == NULL)
	return 0;
  double result = 0;
  if (this->cols_ != this->rows_)
    return 0;
  size_t step = this->step_[1];
  uchar* m = this->data_;
  int rows = this->rows_;
  int cols = this->cols_;
  int cn = 1;

  #define Mf(y, x) ((float*)(m + y*step))[x]
  #define Md(y, x) ((double*)(m + y*step))[x]

  if (this->depth_ <= 5)
  {
    AuMat a(rows, cols, cn, AU_32F);
    if (rows == 2)
    {
      result = det_2(Mf);
    }
    else if (rows == 3)
    {
      result = det_3(Mf);
    }
    else if (rows == 1)
    {
      result = Mf(0,0);
    }
    else
    {
      float* dst = (float*)a.data_;
      switch (this->depth_)
      {
      case 0:
      {
        uchar* src = this->data_;
        CopyData(src, dst, cols, rows, cn);
        break;
      }
      case 1:
      {
        char* src = (char*)this->data_;
        CopyData(src, dst, cols, rows, cn);
        break;
      }
      case 2:
      {
        unsigned short* src = (unsigned short*)this->data_;
        CopyData(src, dst, cols, rows, cn);
        break;
      }
      case 3:
      {
        short* src = (short*)this->data_;
        CopyData(src, dst, cols, rows, cn);
        break;
      }
      case 4:
      {
        int* src = (int*)this->data_;
        CopyData(src, dst, cols, rows, cn);
        break;
      }
      case 5:
      {
        float* src = (float*)this->data_;
        CopyData(src, dst, cols, rows, cn);
        break;
      }
      }

      result = LU((float*)a.data_, a.step_[0], rows, 0, 0, 0);
      if (result)
      {
        for (int i = 0; i < rows; i++)
          result *= ((const float*)(a.data_ + a.step_[0]*i))[i];
        result = 1. / result;
      }
    }

  }
  else if (this->depth_ == AU_64F)
  {
    AuMat a(rows, cols, cn, AU_64F);
    if (rows == 2)
    {
      result = det_2(Md);
    }
    else if (rows == 3)
    {
      result = det_3(Md);
    }
    else if (rows == 1)
    {
      result = Md(0, 0);
    }
    else
    {
      int x, y, c;
      double* src = (double*)this->data_;
      double* dst = (double*)a.data_;
      for (y = 0; y < rows; ++y)
      {
        for (x = 0; x < cols; ++x)
        {
          for (c = 0; c < cn; ++c)
          {
            int idx = (y*cols + x)*cn + c;
            dst[idx] = src[idx];
          }
        }
      }
      result = LU((double*)a.data_, a.step_[0], rows, 0, 0, 0);
      if (result)
      {
        for (int i = 0; i < rows; i++)
          result *= ((const double*)(a.data_ + a.step_[0] * i))[i];
        result = 1. / result;
      }

    }
  }
  return result;
}

///
/// Calculated norm function: L1, L2, Lp and so on
///
template <typename T>
double NormCalculate(T* data_a, int w, int h, int cn, int p)
{
  int x, y, c;
  double sum_value = 0;
  for (y = 0; y < h; ++y)
  {
    for (x = 0; x < w; ++x)
    {
      for (c = 0; c < cn; ++c)
      {
        double value = data_a[(y*w + x)*cn + c];
        if (p == 1)
        {
          sum_value += abs(value);
        }
        else
        {
          sum_value += pow(value , p);
        }
      }
    }
  }
  if (p == 1)
  {
    return sum_value;
  }
  else
  {
    return pow(sum_value, 1.0/p);
  }
}

double AuMat::Norm(int p)
{
  if (this->rows_ == 0 || this->cols_ == 0 || this->data_ == NULL)
    return -1;

  int w = this->cols_;
  int h = this->rows_;
  int cn = this->channels_;
  switch (this->depth_)
  {
  case 0:
  {
    uchar* ptr_a = this->data_;
    return NormCalculate(ptr_a, w, h, cn, p);
  }
  case 1:
  {
    char* ptr_a = (char*)this->data_;
    return NormCalculate(ptr_a, w, h, cn, p);
  }
  case 2:
  {
    unsigned short* ptr_a = (unsigned short*)this->data_;
    return NormCalculate(ptr_a, w, h, cn, p);
  }
  case 3:
  {
    short* ptr_a = (short*)this->data_;
    return NormCalculate(ptr_a, w, h, cn, p);
  }
  case 4:
  {
    int* ptr_a = (int*)this->data_;
    return NormCalculate(ptr_a, w, h, cn, p);
  }
  case 5:
  {
    float* ptr_a = (float*)this->data_;
    return NormCalculate(ptr_a, w, h, cn, p);
  }
  case 6:
  {
    double* ptr_a = (double*)this->data_;
    return NormCalculate(ptr_a, w, h, cn, p);
  }
  }
  return -1;
}

///
/// Transpose matrix: rows <--> cols
///
template <typename T>
inline void TransData(T* data_a, T* data_c, int w, int h, int cn)
{
  int x, y, c;
  for (y = 0; y < w; ++y)
  {
    for (x = 0; x < h; ++x)
    {
      for (c = 0; c < cn; ++c)
      {
        data_c[(y*h + x)*cn + c] = data_a[(x*w+y)*cn+c];
      }
    }
  }
}

AuMat AuMat::Trans()
{
  AuMat mat_zero(0, 0, 0, 0, 0);
  if (this->rows_ == 0 || this->cols_ == 0 || this->data_ == NULL)
    return mat_zero;

  int w = this->cols_;
  int h = this->rows_;
  int cn = this->channels_;
  AuMat mat_c(w, h, cn, this->depth_);
  switch (this->depth_)
  {
  case 0:
  {
    uchar* ptr_a = this->data_;
    uchar* ptr_c = mat_c.data_;
    TransData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  case 1:
  {
    char* ptr_a = (char*)this->data_;
    char* ptr_c = (char*)mat_c.data_;
    TransData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  case 2:
  {
    unsigned short* ptr_a = (unsigned short*)this->data_;
    unsigned short* ptr_c = (unsigned short*)mat_c.data_;
    TransData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  case 3:
  {
    short* ptr_a = (short*)this->data_;
    short* ptr_c = (short*)mat_c.data_;
    TransData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  case 4:
  {
    int* ptr_a = (int*)this->data_;
    int* ptr_c = (int*)mat_c.data_;
    TransData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  case 5:
  {
    float* ptr_a = (float*)this->data_;
    float* ptr_c = (float*)mat_c.data_;
    TransData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  case 6:
  {
    double* ptr_a = (double*)this->data_;
    double* ptr_c = (double*)mat_c.data_;
    TransData(ptr_a, ptr_c, w, h, cn);
    break;
  }
  }
  return mat_c;
}

///
/// Find maximum or minimum value from matrix
///
template <typename T>
double MaxMin(T* data_a, int w, int h, int cn, AuPoint<int>* point, int is_max)
{
  int x, y, c;
  double max_value = 0;
  double min_value = 1000000;
  AuPoint<int> point_max, point_min;
  for (y = 0; y < h; ++y)
  {
    for (x = 0; x < w; ++x)
    {
      for (c = 0; c < cn; ++c)
      {
        T value = data_a[(y*w + x)*cn + c];
        if (value > max_value)
        {
          max_value = value;
          point_max.x_ = x;
          point_max.y_ = y;
        }
        if (value < min_value)
        {
          min_value = value;
          point_min.x_ = x;
          point_min.y_ = y;
        }
      }
    }
  }
  if (is_max)
  {
    if (point != NULL){
      point->x_ = point_max.x_;
      point->y_ = point_max.y_;
    }
    return max_value;
  }
  else
  {
    if (point != NULL){
      point->x_ = point_min.x_;
      point->y_ = point_min.y_;
    }
    return min_value;
  }
}

double AuMat::Max(AuPoint<int>* point)
{
  if (this->rows_ == 0 || this->cols_ == 0 || this->data_ == NULL)
    return -1;

  int w = this->cols_;
  int h = this->rows_;
  int cn = this->channels_;
  int is_max = 1;
  switch (this->depth_)
  {
  case 0:
  {
    uchar* ptr_a = this->data_;
    return MaxMin(ptr_a, w, h, cn, point, is_max);
  }
  case 1:
  {
    char* ptr_a = (char*)this->data_;
    return MaxMin(ptr_a, w, h, cn, point, is_max);
  }
  case 2:
  {
    unsigned short* ptr_a = (unsigned short*)this->data_;
    return MaxMin(ptr_a, w, h, cn, point, is_max);
  }
  case 3:
  {
    short* ptr_a = (short*)this->data_;
    return MaxMin(ptr_a, w, h, cn, point, is_max);
  }
  case 4:
  {
    int* ptr_a = (int*)this->data_;
    return MaxMin(ptr_a, w, h, cn, point, is_max);
  }
  case 5:
  {
    float* ptr_a = (float*)this->data_;
    return MaxMin(ptr_a, w, h, cn, point, is_max);
  }
  case 6:
  {
    double* ptr_a = (double*)this->data_;
    return MaxMin(ptr_a, w, h, cn, point, is_max);
  }
  }
  return -1;
}

double AuMat::Min(AuPoint<int>* point)
{
  if (this->rows_ == 0 || this->cols_ == 0 || this->data_ == NULL)
    return -1;

  int w = this->cols_;
  int h = this->rows_;
  int cn = this->channels_;
  int is_max = 0;
  switch (this->depth_)
  {
  case 0:
  {
    uchar* ptr_a = this->data_;
    return MaxMin(ptr_a, w, h, cn, point, is_max);
  }
  case 1:
  {
    char* ptr_a = (char*)this->data_;
    return MaxMin(ptr_a, w, h, cn, point, is_max);
  }
  case 2:
  {
    unsigned short* ptr_a = (unsigned short*)this->data_;
    return MaxMin(ptr_a, w, h, cn, point, is_max);
  }
  case 3:
  {
    short* ptr_a = (short*)this->data_;
    return MaxMin(ptr_a, w, h, cn, point, is_max);
  }
  case 4:
  {
    int* ptr_a = (int*)this->data_;
    return MaxMin(ptr_a, w, h, cn, point, is_max);
  }
  case 5:
  {
    float* ptr_a = (float*)this->data_;
    return MaxMin(ptr_a, w, h, cn, point, is_max);
  }
  case 6:
  {
    double* ptr_a = (double*)this->data_;
    return MaxMin(ptr_a, w, h, cn, point, is_max);
  }
  }
  return -1;
}

AuMat AuMat::operator+(const AuMat& kMatB)
{
  return this->Add(kMatB);
}

AuMat AuMat::operator-(const AuMat& kMatB)
{
  return this->Sub(kMatB);
}

template <typename T>
inline void MulNumber(T* data_a, T* data_b, T* data_c, int w, int h, int cn)
{
  int x,y,c;
  for (y = 0; y < h; ++y)
  {
    for (x = 0; x < w; ++x)
    {
      for (c = 0; c < cn; ++c)
      {
        int idx = (y*w + x)*cn + c;
        data_c[idx] = data_a[idx] * data_b[idx];
      }
    }
  }
}

AuMat AuMat::operator*(const AuMat& kMatB)
{
  AuMat mat_zero(0, 0, 0, 0, 0);
  if (this->rows_ != kMatB.rows_ || this->cols_ != kMatB.cols_ ||
    this->data_ == NULL || kMatB.data_ == NULL ||
    this->depth_ != kMatB.depth_)
    return mat_zero;

  int w = kMatB.cols_;
  int h = kMatB.rows_;
  int cn = kMatB.channels_;
  AuMat mat_c(h, w, cn, kMatB.depth_);

  switch (this->depth_)
  {
  case 0:
  {
    uchar* ptr_a = this->data_;
    uchar* ptr_b = kMatB.data_;
    uchar* ptr_c = mat_c.data_;
    MulNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 1:
  {
    char* ptr_a = (char*)this->data_;
    char* ptr_b = (char*)kMatB.data_;
    char* ptr_c = (char*)mat_c.data_;
    MulNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 2:
  {
    unsigned short* ptr_a = (unsigned short*)this->data_;
    unsigned short* ptr_b = (unsigned short*)kMatB.data_;
    unsigned short* ptr_c = (unsigned short*)mat_c.data_;
    MulNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 3:
  {
    short* ptr_a = (short*)this->data_;
    short* ptr_b = (short*)kMatB.data_;
    short* ptr_c = (short*)mat_c.data_;
    MulNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 4:
  {
    int* ptr_a = (int*)this->data_;
    int* ptr_b = (int*)kMatB.data_;
    int* ptr_c = (int*)mat_c.data_;
    MulNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 5:
  {
    float* ptr_a = (float*)this->data_;
    float* ptr_b = (float*)kMatB.data_;
    float* ptr_c = (float*)mat_c.data_;
    MulNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 6:
  {
    double* ptr_a = (double*)this->data_;
    double* ptr_b = (double*)kMatB.data_;
    double* ptr_c = (double*)mat_c.data_;
    MulNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  }
  return mat_c;
}

template <typename T>
inline void DivNumber(T* data_a, T* data_b, T* data_c, int w, int h, int cn)
{
  int x, y, c;
  for (y = 0; y < h; ++y)
  {
    for (x = 0; x < w; ++x)
    {
      for (c = 0; c < cn; ++c)
      {
        int idx = (y*w + x)*cn + c;
        T value = AuMax(data_b[idx], Lambda);
        if (value == 0)
          value = 1;
        data_c[idx] = data_a[idx] / value;
      }
    }
  }
}

AuMat AuMat::operator/(const AuMat& kMatB)
{
  AuMat mat_zero(0, 0, 0, 0, 0);
  if (this->rows_ != kMatB.rows_ || this->cols_ != kMatB.cols_ ||
    this->data_ == NULL || kMatB.data_ == NULL ||
    this->depth_ != kMatB.depth_)
    return mat_zero;

  int w = kMatB.cols_;
  int h = kMatB.rows_;
  int cn = kMatB.channels_;
  AuMat mat_c(h, w, cn, kMatB.depth_);

  switch (this->depth_)
  {
  case 0:
  {
    uchar* ptr_a = this->data_;
    uchar* ptr_b = kMatB.data_;
    uchar* ptr_c = mat_c.data_;
    DivNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 1:
  {
    char* ptr_a = (char*)this->data_;
    char* ptr_b = (char*)kMatB.data_;
    char* ptr_c = (char*)mat_c.data_;
    DivNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 2:
  {
    unsigned short* ptr_a = (unsigned short*)this->data_;
    unsigned short* ptr_b = (unsigned short*)kMatB.data_;
    unsigned short* ptr_c = (unsigned short*)mat_c.data_;
    DivNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 3:
  {
    short* ptr_a = (short*)this->data_;
    short* ptr_b = (short*)kMatB.data_;
    short* ptr_c = (short*)mat_c.data_;
    DivNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 4:
  {
    int* ptr_a = (int*)this->data_;
    int* ptr_b = (int*)kMatB.data_;
    int* ptr_c = (int*)mat_c.data_;
    DivNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 5:
  {
    float* ptr_a = (float*)this->data_;
    float* ptr_b = (float*)kMatB.data_;
    float* ptr_c = (float*)mat_c.data_;
    DivNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  case 6:
  {
    double* ptr_a = (double*)this->data_;
    double* ptr_b = (double*)kMatB.data_;
    double* ptr_c = (double*)mat_c.data_;
    DivNumber(ptr_a, ptr_b, ptr_c, w, h, cn);
    break;
  }
  }
  return mat_c;
}

AuMat& AuMat::operator=(const AuMat& kMatB)
{
  int w = kMatB.cols_;
  int h = kMatB.rows_;
  int cn = kMatB.channels_;

  cols_ = kMatB.cols_;
  rows_ = kMatB.rows_;
  channels_ = kMatB.channels_;
  depth_ = kMatB.depth_;
  dims_ = kMatB.dims_;
  step_[1] = kMatB.step_[1];
  step_[0] = kMatB.step_[0];

  if(this->data_ == NULL)
  {
    switch (kMatB.depth_)
    {
    case 0:
    {
      uchar* ptr_b = kMatB.data_;
      int len = w*h*cn;
      data_ = new unsigned char[len];
      memcpy(data_, ptr_b, len);
      is_getMem_ = 1;
      break;
    }
    case 1:
    {
      char* ptr_b = (char*)kMatB.data_;
      int len = w*h*cn;
      data_ = new unsigned char[len];
      memcpy(data_, (unsigned char*)ptr_b, len);
      is_getMem_ = 1;
      break;
    }
    case 2:
    {
      unsigned short* ptr_b = (unsigned short*)kMatB.data_;
      int len = w*h*cn*sizeof(unsigned short);
      data_ = new unsigned char[len];
      memcpy(data_, (unsigned char*)ptr_b, len);
      is_getMem_ = 1;
      break;
    }
    case 3:
    {
      short* ptr_b = (short*)kMatB.data_;
      int len = w*h*cn*sizeof(short);
      data_ = new unsigned char[len];
      memcpy(data_, (unsigned char*)ptr_b, len);
      is_getMem_ = 1;
      break;
    }
    case 4:
    {
      int* ptr_b = (int*)kMatB.data_;
      int len = w*h*cn*sizeof(int);
      data_ = new unsigned char[len];
      memcpy(data_, (unsigned char*)ptr_b, len);
      is_getMem_ = 1;
      break;
    }
    case 5:
    {
      float* ptr_b = (float*)kMatB.data_;
      int len = w*h*cn*sizeof(float);
      data_ = new unsigned char[len];
      memcpy(data_, (unsigned char*)ptr_b, len);
      is_getMem_ = 1;
      break;
    }
    case 6:
    {
      double* ptr_b = (double*)kMatB.data_;
      int len = w*h*cn*sizeof(double);
      data_ = new unsigned char[len];
      memcpy(data_, (unsigned char*)ptr_b, len);
      is_getMem_ = 1;
      break;
    }
    }
  }
  else
  {
    data_ = kMatB.data_;
    is_getMem_ = 0;
  }
  return *this;
}

void locateROI(AuMat*m, AuSize* wholeSize, AuPoint<int>* ofs )
{
	if(m->step_[0] == 0 || m->data_ == NULL)
		return;
    size_t esz = m->step_[m->dims_-1];
    size_t minstep;
    float delta1 = 0;
    float delta2 = m->cols_ * m->rows_;

    if( delta1 == 0 )
        ofs->x_ = ofs->y_ = 0;
    else
    {
        ofs->y_ = (int)(delta1/m->step_[0]);
        ofs->x_ = (int)((delta1 - m->step_[0]*ofs->y_)/esz);
    }

    minstep = (ofs->x_ + m->cols_)*esz;
    wholeSize->height_ = (int)((delta2 - minstep)/m->step_[0] + 1);
    wholeSize->height_ = AuMax(wholeSize->height_, ofs->y_ + m->rows_);
    wholeSize->width_ = (int)((delta2 - m->step_[0]*(wholeSize->height_-1))/esz);
    wholeSize->width_ = AuMax(wholeSize->width_, ofs->x_ + m->cols_);

}

void adjustROI(AuMat* m, int dtop, int dbottom, int dleft, int dright )
{
	if(m->step_[0] == 0 || m->data_ == NULL)
		return;
    AuSize wholeSize; AuPoint<int> ofs;
    size_t esz = m->step_[m->dims_-1];
    locateROI(m, &wholeSize, &ofs );
    int row1 = AuMax(ofs.y_ - dtop, 0), row2 = AuMin(ofs.y_ + m->rows_ + dbottom, wholeSize.height_);
    int col1 = AuMax(ofs.x_ - dleft, 0), col2 = AuMin(ofs.x_ + m->cols_ + dright, wholeSize.width_);
    m->data_ += (row1 - ofs.y_)*m->step_[0] + (col1 - ofs.x_)*esz;
    m->rows_ = row2 - row1; m->cols_ = col2 - col1;
    m->size_[0] = m->rows_;
    m->size_[1] = m->cols_;
}

AuMat getROI(AuMat m, AuRect<int>* roi)
{
    AuMat dst;
    dst.dims_ = 2;
    dst.rows_ = roi->height_;
    dst.cols_ = roi->width_;
    dst.size_[0] = dst.rows_;
    dst.size_[1] = dst.cols_;
    dst.step_[0] = m.step_[0];
    dst.step_[1] = m.step_[1];
    dst.channels_ = m.channels_;
    dst.depth_ = m.depth_;

    dst.data_ = m.data_ + roi->y_*m.step_[0] + roi->x_*m.step_[1];

    if( m.rows_ <= 0 || m.cols_ <= 0 )
    {
        dst.rows_ = dst.cols_ = 0;
    }
    return dst;

}

}//namespace computer_vision

}//namespace autel
