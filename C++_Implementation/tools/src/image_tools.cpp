/// Copyright (c) 2016 AutelRobotics. All Rights Reserved
///
/// \image_tools.h
///
/// \Definition : Image fundamental functions are defined here;
///
///  \author XipengCui
///  \Date : Created on: Jan 1, 2017
///
#include "../include/image_tools.h"
#include <string.h>
#include <math.h>
#include <string>
#include <typeinfo>
#include "../include/wrappers.h"

void Resize(unsigned char* src, unsigned char* dst, AuSize src_size, int src_step, int cn, AuSize dst_size, int dst_step, uchar* memoryPara)
{
    unsigned char* data = src;
    if(src == dst)
    {
        data = memoryPara;
        memcpy(data, src, src_size.width_*src_size.height_*3);
    }
    else if(dst == NULL)
    {
        dst = memoryPara;
    }
    //˫���²�ֵ
    int w = dst_size.width_;
    int h = dst_size.height_;

    //ʱ����Լ���һ��
    int s = 10;
    int s_sum = 1<<s;

    int x, y;
    for(y = 0; y < h; y++)
    {
        long test = (long)(y*src_size.height_)<<s;
        long y1 = test/h;
        test = y1>>s;
        int a = y1 - (test<<s);
        y1 = y1>>s;
        for(x = 0; x < w; x++)
        {
            test = (long)(x*src_size.width_)<<s;
            long x1 = test/w;
            test = x1>>s;
            int b = x1 - (test<<s);
            x1 = x1>>s;

            {
                int dst_id = (y*dst_step+x)*cn;
                long data_1 = data[(y1*src_step+x1)*cn];
                long data_2 = data[(y1*src_step+x1+1)*cn];
                long data_3 = data[((y1+1)*src_step+x1+1)*cn];
                long data_4 = data[((y1+1)*src_step+x1)*cn];
                dst[dst_id] =(unsigned char)((data_1*a*b + data_2*a*(s_sum-b)+data_3*(s_sum-a)*(s_sum-b) + data_4*(s_sum-a)*b)>>(s+s));

                if(cn > 1)
                {
                    data_1 = data[(y1*src_step+x1)*3+1];
                    data_2 = data[(y1*src_step+x1+1)*3+1];
                    data_3 = data[((y1+1)*src_step+x1+1)*3+1];
                    data_4 = data[((y1+1)*src_step+x1)*3+1];
                    dst[dst_id+1] =(unsigned char)((data_1*a*b + data_2*a*(s_sum-b)+data_3*(s_sum-a)*(s_sum-b) + data_4*(s_sum-a)*b)>>(s+s));

                    data_1 = data[(y1*src_step+x1)*3+2];
                    data_2 = data[(y1*src_step+x1+1)*3+2];
                    data_3 = data[((y1+1)*src_step+x1+1)*3+2];
                    data_4 = data[((y1+1)*src_step+x1)*3+2];
                    dst[dst_id+2] =(unsigned char)((data_1*a*b + data_2*a*(s_sum-b)+data_3*(s_sum-a)*(s_sum-b) + data_4*(s_sum-a)*b)>>(s+s));
                }
            }
        }
    }
}

///
void ResizeROI_whynotuse(unsigned char* src, unsigned char* dst, AuRect<int> roi, AuSize src_size, int src_step, int cn, AuSize dst_size, int dst_step)
{
    //bilinearity resample
    int w = dst_size.width_;
    int h = dst_size.height_;

    //float --> int
    int s = 10;
    int s_sum = 1<<s;

    int x, y;
    for(y = 0; y < h; y++)
    {
        long test = (long)(y*src_size.height_)<<s;
        long y1 = test/h;
        test = y1>>s;
        int a = y1 - (test<<s);
        y1 = y1>>s;
        y1 += roi.y_;
        for(x = 0; x < w; x++)
        {
            test = (long)(x*src_size.width_)<<s;
            long x1 = test/w;
            test = x1>>s;
            int b = x1 - (test<<s);
            x1 = x1>>s;
            x1 += roi.x_;
            {
                int dst_id = (y*dst_step+x)*cn;
                long data_1 = src[(y1*src_step+x1)*cn];
                long data_2 = src[(y1*src_step+x1+1)*cn];
                long data_3 = src[((y1+1)*src_step+x1+1)*cn];
                long data_4 = src[((y1+1)*src_step+x1)*cn];
                dst[dst_id] =(unsigned char)((data_1*a*b + data_2*a*(s_sum-b)+data_3*(s_sum-a)*(s_sum-b) + data_4*(s_sum-a)*b)>>(s+s));

                if(cn > 1)
                {
                    data_1 = src[(y1*src_step+x1)*3+1];
                    data_2 = src[(y1*src_step+x1+1)*3+1];
                    data_3 = src[((y1+1)*src_step+x1+1)*3+1];
                    data_4 = src[((y1+1)*src_step+x1)*3+1];
                    dst[dst_id+1] =(unsigned char)((data_1*a*b + data_2*a*(s_sum-b)+data_3*(s_sum-a)*(s_sum-b) + data_4*(s_sum-a)*b)>>(s+s));

                    data_1 = src[(y1*src_step+x1)*3+2];
                    data_2 = src[(y1*src_step+x1+1)*3+2];
                    data_3 = src[((y1+1)*src_step+x1+1)*3+2];
                    data_4 = src[((y1+1)*src_step+x1)*3+2];
                    dst[dst_id+2] =(unsigned char)((data_1*a*b + data_2*a*(s_sum-b)+data_3*(s_sum-a)*(s_sum-b) + data_4*(s_sum-a)*b)>>(s+s));
                }
            }
        }
    }
}
///

void ResizeYUVToYUV(unsigned char* src_y, unsigned char* src_uv, AuRect<int> roi, unsigned char* dst_y, unsigned char* dst_uv,
    AuSize src_size, int src_width, int cn, AuSize dst_size, int dst_width, int data_format)
{
    //bilinearity resample
    int w = dst_size.width_;
    int h = dst_size.height_;

    int x, y;
    for(y = 0; y < h; y++)
    {
        float test = y*src_size.height_;
        float y1 = test/h;
        float a = (int)(y1+1)-y1;
        y1 += roi.y_;
        for(x = 0; x < w; x++)
        {
            test = x*src_size.width_;
            float x1 = test/w;
            float b = (int)(x1+1) - x1;
            x1 += roi.x_;

            int data_y[4];
            int data_u[4];
            int data_v[4];
            int x11, y11, idx = 0;
            for(y11 = y1; y11 <= y1+1; ++y11)
            {
                for(x11 = x1; x11 <= x1+1; ++x11)
                {
                  int U,V;
                  data_y[idx] = src_y[y11*src_width+x11];
                  idx++;
                }
            }

            int y_id = (y*dst_width+x)*cn;
            int uv_id = (y*dst_width+x)*cn;
            dst_y[y_id] = (unsigned char)(data_y[0]*a*b + data_y[1]*a*(1-b) + data_y[2]*(1-a)*b + data_y[3]*(1-a)*(1-b));

            if(0 == data_format)
            {
              uv_id = (y/2*dst_width+x)*cn;
              y11 = y1/2;
              x11 = x1;
              if(x%2 == 0)
                dst_uv[uv_id] = (unsigned char)(src_uv[y11*src_width+x11]);
              else
                dst_uv[uv_id] = (unsigned char)(src_uv[y11*src_width+x11+1]);
            }
            else
            {
              y11 = y1;
              x11 = x1;
              if(x%2 == 0)
                dst_uv[uv_id] = (unsigned char)(src_uv[y11*src_width+x11]);
              else
                dst_uv[uv_id] = (unsigned char)(src_uv[y11*src_width+x11+1]);
            }
        }
    }
}

///
/// img becomes larger by filling data
/// param[in]: img: old image
/// param[in]: p: size of img: img.cols_ + p[2]+p[3], img.rows_+p[0]+p[1];
/// param[in]: style: how to fill data
/// param[out]: img: new big image.
///
void ImPad(AuMat& img, int* p, char* style)
{
  //the data memory will become bigger, so no use memory of copy constructor function
  AuMat image;
  image.channels_ = img.channels_;
  image.cols_ = img.cols_;
  image.rows_ = img.rows_;
  image.depth_ = img.depth_;
  image.dims_ = img.dims_;
  image.step_[1] = image.channels_;
  image.step_[0] = image.cols_*image.channels_;
  int sum = (p[0]+p[1])*image.cols_ + (p[2]+p[3])*(image.rows_+p[0]+p[1]);
  unsigned char* memory = new unsigned char[(image.cols_*image.rows_+sum)*image.channels_];
  image.data_ = memory + (p[0]*(p[2]+image.cols_+p[3])+p[2])*image.channels_;
  memcpy(image.data_, img.data_, img.rows_*img.step_[0]);

  if (!strcmp(style, "symmetric"))
  {
    unsigned char* ptr_dst;
    unsigned char* ptr_src;
    int idx = 0;

    //bottom
    ptr_src = image.data_ + (image.rows_ - 1) * image.step_[0];
    ptr_dst = image.data_ + image.rows_ * image.step_[0];
    for (idx = 0; idx < p[1]; idx++)
    {
      memcpy(ptr_dst, ptr_src, image.step_[0]);
      ptr_src -= image.step_[0];
      ptr_dst += image.step_[0];
    }

    //top
    ptr_src = image.data_;
    ptr_dst = image.data_ - image.step_[0];
    for (idx = 0; idx < p[0]; idx++)
    {
      memcpy(ptr_dst, ptr_src, image.step_[0]);
      ptr_src += image.step_[0];
      ptr_dst -= image.step_[0];
    }
    image.data_ = ptr_dst + image.step_[0];
    image.rows_ += p[0] + p[1];

    img.step_[0] = image.step_[0] + (p[2] + p[3])*image.channels_;
    img.rows_ = image.rows_;
    img.cols_ = image.cols_ + p[2] + p[3];

    //it is hard to copy image data to left or right side.
    int y = 0;
    for (y = 0; y < image.rows_; y++)
    {
      ptr_src = image.data_ + y*image.step_[0]+(p[2]-1)*image.channels_;
      ptr_dst = img.data_ + y*img.step_[0];
      for (idx = 0; idx < p[2] + p[3] + image.cols_; idx++)
      {
        //left
        if (idx < p[2])
        {
          memcpy(ptr_dst, ptr_src, image.channels_);
          ptr_src -= image.channels_;
          ptr_dst += image.channels_;
        }
        else if (idx >= p[2] + image.cols_)//right
        {
          ptr_src -= image.channels_;
          memcpy(ptr_dst, ptr_src, image.channels_);
          ptr_dst += image.channels_;
        }
        else{

          ptr_src = image.data_ + y*image.step_[0];
          memcpy(ptr_dst, ptr_src, image.step_[0]);
          ptr_dst += image.step_[0];
          ptr_src += image.step_[0];
          idx += image.cols_-1;
        }
      }
    }
  }
  delete [] memory;
}

// Constants for rgb2luv conversion and lookup table for y-> l conversion
float* rgb2luv_setup(float z, float *mr, float *mg, float *mb,
  float &minu, float &minv, float &un, float &vn)
{
  // set constants for conversion
  const float y0 = (float)((6.0 / 29)*(6.0 / 29)*(6.0 / 29));
  const float a = (float)((29.0 / 3)*(29.0 / 3)*(29.0 / 3));
  un = (float) 0.197833; vn = (float) 0.468331;
  mr[0] = (float) 0.430574*z; mr[1] = (float) 0.222015*z; mr[2] = (float) 0.020183*z;
  mg[0] = (float) 0.341550*z; mg[1] = (float) 0.706655*z; mg[2] = (float) 0.129553*z;
  mb[0] = (float) 0.178325*z; mb[1] = (float) 0.071330*z; mb[2] = (float) 0.939180*z;
  float maxi = (float) 1.0 / 270;
  minu = -88 * maxi;
  minv = -134 * maxi;
  // build (padded) lookup table for y->l conversion assuming y in [0,1]
  static float lTable[1064];
  static bool lInit = false;
  if (lInit)
    return lTable;
  float y, l;
  for (int i = 0; i<1025; i++) {
    y = (float)(i / 1024.0);
    l = y>y0 ? 116 * (float)pow((double)y, 1.0 / 3.0) - 16 : y*a;
    lTable[i] = l*maxi;
  }
  for (int i = 1025; i < 1064; i++) {
    lTable[i] = lTable[i - 1];
  }
  lInit = true;
  return lTable;
}

///
/// Convert RGB image to other color spaces
/// param[in]: I: RGB image
/// param[in]: n: number of image pixels
/// param[in]: nrm: normalizing coefficient
/// param[out]: J: other color spaces
///
void RGB2LUV(AuMat& I, AuMat* J, int n, float nrm) {
  float minu, minv, un, vn, mr[3], mg[3], mb[3];
  float *lTable = rgb2luv_setup(nrm, mr, mg, mb, minu, minv, un, vn);
  float *Luv = (float*)(J->data_);
  unsigned char *R = I.data_;

  for (int i = 0; i<n; i++) {
    float r, g, b, x, y, z, l;
    b = *R++; g = *R++; r = *R++;
    x = mr[0] * r + mg[0] * g + mb[0] * b;
    y = mr[1] * r + mg[1] * g + mb[1] * b;
    z = mr[2] * r + mg[2] * g + mb[2] * b;
    l = lTable[(int)(y * 1024)];
    *(Luv++) = l; z = 1 / (x + 15 * y + 3 * z + (float)1e-35);
    *(Luv++) = l * (13 * 4 * x*z - 13 * un) - minu;
    *(Luv++) = l * (13 * 9 * y*z - 13 * vn) - minv;
  }
}

///
/// Normalize image
/// param[in]: I: image
/// param[in]: n: number of image pixels
/// param[in]: nrm: normalizing coefficient
/// param[out]: J: normalization image
///
void NormalizeImage(AuMat& I, AuMat* J, int n, float nrm) {
  float* ptr = (float*)J->data_;
  //for (int i = 0; i < n; i++)
  //{
  //  *(ptr++) = (I.data_[i])*nrm;
  //}
  int x, y;
  int w = I.cols_;
  int h = I.rows_;
  int cn = I.channels_;
  for (y = 0; y < h; ++y)
  {
    for (x = 0; x < w; ++x)
    {
      if (cn > 1)
      {
        ptr[y*w + x] = I.data_[(y*w + x)*cn] * nrm;
        ptr[y*w + x+w*h] = I.data_[(y*w + x)*cn + 1] * nrm;
        ptr[y*w + x+w*h*2] = I.data_[(y*w + x)*cn + 2] * nrm;
      }
    }
  }

}

///
/// Down sample for image
/// param[in]: src: inputting image, which the data is float
/// param[out]: dst: resampled image
///
void ImResample(AuMat& src, AuMat* dst)
{
  float* data = (float*)src.data_;
  float* dst_data = (float*)dst->data_;
  //bilinearity insert
  int w = dst->cols_;
  int h = dst->rows_;
  int src_step = src.cols_;
  int dst_step = dst->cols_;
  int x, y;
  for (y = 0; y < h; y++)
  {
    float a = (float)y * src.rows_ / h;
    int y1 = (int)a;
    a = y1 + 1 - a;

    for (x = 0; x < w; x++)
    {
      float b = (float)x * src.cols_ / w;
      int x1 = (int)b;
      b = x1 + 1 - b;
      if (src.channels_ == 3)
      {
        /// image is divided into three layers
        int dst_id = (y*dst_step + x);
        float data_1 = data[(y1*src_step + x1) * 3];
        float data_2 = data[(y1*src_step + x1 + 1) * 3];
        float data_3 = data[((y1 + 1)*src_step + x1 + 1) * 3];
        float data_4 = data[((y1 + 1)*src_step + x1) * 3];
        dst_data[dst_id] = data_1*a*b + data_2*a*(1 - b) + data_3*(1 - a)*(1 - b) + data_4*(1 - a)*b;

        dst_id += w*h;
        data_1 = data[(y1*src_step + x1) * 3 + 1];
        data_2 = data[(y1*src_step + x1 + 1) * 3 + 1];
        data_3 = data[((y1 + 1)*src_step + x1 + 1) * 3 + 1];
        data_4 = data[((y1 + 1)*src_step + x1) * 3 + 1];
        dst_data[dst_id] = data_1*a*b + data_2*a*(1 - b) + data_3*(1 - a)*(1 - b) + data_4*(1 - a)*b;

        dst_id += w*h;
        data_1 = data[(y1*src_step + x1) * 3 + 2];
        data_2 = data[(y1*src_step + x1 + 1) * 3 + 2];
        data_3 = data[((y1 + 1)*src_step + x1 + 1) * 3 + 2];
        data_4 = data[((y1 + 1)*src_step + x1) * 3 + 2];
        dst_data[dst_id] = data_1*a*b + data_2*a*(1 - b) + data_3*(1 - a)*(1 - b) + data_4*(1 - a)*b;
      }
      else if (src.channels_ == 1)
      {
        int dst_id = y*dst_step + x;
        float data_1 = data[y1*src_step + x1];
        float data_2 = data[y1*src_step + x1 + 1];
        float data_3 = data[(y1 + 1)*src_step + x1 + 1];
        float data_4 = data[(y1 + 1)*src_step + x1];
        dst_data[dst_id] = data_1*a*b + data_2*a*(1 - b) + data_3*(1 - a)*(1 - b) + data_4*(1 - a)*b;
      }
    }
  }
}

///
/// Down sample for image
/// param[in]: src: inputting image, which the data is unsigned char
/// param[out]: dst: resampled image
///
void ImResample_char(AuMat& src, AuMat* dst)
{
  unsigned char* data = src.data_;
  unsigned char* dst_data = dst->data_;
  //bilinearity insert
  int w = dst->cols_;
  int h = dst->rows_;
  int src_step = src.cols_;
  int dst_step = dst->cols_;
  int x, y;
  for (y = 0; y < h; y++)
  {
    float a = (float)y * src.rows_ / h;
    int y1 = (int)a;
    a = y1 + 1 - a;

    for (x = 0; x < w; x++)
    {
      float b = (float)x * src.cols_ / w;
      int x1 = (int)b;
      b = x1 + 1 - b;
      {
        int dst_id = (y*dst_step + x) * 3;
        int data_1 = data[(y1*src_step + x1) * 3];
        int data_2 = data[(y1*src_step + x1 + 1) * 3];
        int data_3 = data[((y1 + 1)*src_step + x1 + 1) * 3];
        int data_4 = data[((y1 + 1)*src_step + x1) * 3];
        dst_data[dst_id] = data_1*a*b + data_2*a*(1 - b) + data_3*(1 - a)*(1 - b) + data_4*(1 - a)*b;

        data_1 = data[(y1*src_step + x1) * 3 + 1];
        data_2 = data[(y1*src_step + x1 + 1) * 3 + 1];
        data_3 = data[((y1 + 1)*src_step + x1 + 1) * 3 + 1];
        data_4 = data[((y1 + 1)*src_step + x1) * 3 + 1];
        dst_data[dst_id + 1] = data_1*a*b + data_2*a*(1 - b) + data_3*(1 - a)*(1 - b) + data_4*(1 - a)*b;

        data_1 = data[(y1*src_step + x1) * 3 + 2];
        data_2 = data[(y1*src_step + x1 + 1) * 3 + 2];
        data_3 = data[((y1 + 1)*src_step + x1 + 1) * 3 + 2];
        data_4 = data[((y1 + 1)*src_step + x1) * 3 + 2];
        dst_data[dst_id + 2] = data_1*a*b + data_2*a*(1 - b) + data_3*(1 - a)*(1 - b) + data_4*(1 - a)*b;
      }
    }
  }
}

//// compute interpolation values for single column for resapling
//void resampleCoef(int ha, int hb, int &n, int *&yas,
//  int *&ybs, float *&wts, int bd[2], int pad = 0)
//{
//  const float s = float(hb) / float(ha), sInv = 1 / s; float wt, wt0 = float(1e-3)*s;
//  bool ds = ha>hb; int nMax; bd[0] = bd[1] = 0;
//  if (ds) { n = 0; nMax = ha + (pad>2 ? pad : 2)*hb; }
//  else { n = nMax = hb; }
//  // initialize memory
//  wts = (float*)alMalloc(nMax*sizeof(float), 16);
//  yas = (int*)alMalloc(nMax*sizeof(int), 16);
//  ybs = (int*)alMalloc(nMax*sizeof(int), 16);
//  if (ds) for (int yb = 0; yb<hb; yb++) {
//    // create coefficients for downsampling
//    float ya0f = yb*sInv, ya1f = ya0f + sInv, W = 0;
//    int ya0 = int(ceil(ya0f)), ya1 = int(ya1f), n1 = 0;
//    for (int ya = ya0 - 1; ya<ya1 + 1; ya++) {
//      wt = s; if (ya == ya0 - 1) wt = (ya0 - ya0f)*s; else if (ya == ya1) wt = (ya1f - ya1)*s;
//      if (wt>wt0 && ya >= 0) { ybs[n] = yb; yas[n] = ya; wts[n] = wt; n++; n1++; W += wt; }
//    }
//    if (W>1) for (int i = 0; i<n1; i++) wts[n - n1 + i] /= W;
//    if (n1>bd[0]) bd[0] = n1;
//    while (n1<pad) { ybs[n] = yb; yas[n] = yas[n - 1]; wts[n] = 0; n++; n1++; }
//  }
//  else for (int yb = 0; yb<hb; yb++) {
//    // create coefficients for upsampling
//    float yaf = (float(.5) + yb)*sInv - float(.5); int ya = (int)floor(yaf);
//    wt = 1; if (ya >= 0 && ya<ha - 1) wt = 1 - (yaf - ya);
//    if (ya<0) { ya = 0; bd[0]++; } if (ya >= ha - 1) { ya = ha - 1; bd[1]++; }
//    ybs[yb] = yb; yas[yb] = ya; wts[yb] = wt;
//  }
//}
//
//// resample A using bilinear interpolation and and store result in B
//void resample(float *A, float *B, int ha, int hb, int wa, int wb, int d, float r) {
//  int hn, wn, x, x1, y, z, xa, xb, ya; float *A0, *A1, *A2, *A3, *B0, wt, wt1;
//  float *C = (float*)alMalloc((ha + 4)*sizeof(float), 16); for (y = ha; y<ha + 4; y++) C[y] = 0;
//  bool sse = (typeid(float) == typeid(float)) && !(size_t(A) & 15) && !(size_t(B) & 15);
//  // get coefficients for resampling along w and h
//  int *xas, *xbs, *yas, *ybs; float *xwts, *ywts; int xbd[2], ybd[2];
//  resampleCoef(wa, wb, wn, xas, xbs, xwts, xbd, 0);
//  resampleCoef(ha, hb, hn, yas, ybs, ywts, ybd, 4);
//  if (wa == 2 * wb) r /= 2; if (wa == 3 * wb) r /= 3; if (wa == 4 * wb) r /= 4;
//  r /= float(1 + 1e-6); for (y = 0; y<hn; y++) ywts[y] *= r;
//  // resample each channel in turn
//  for (z = 0; z<d; z++) for (x = 0; x<wb; x++) {
//    if (x == 0) x1 = 0; xa = xas[x1]; xb = xbs[x1]; wt = xwts[x1]; wt1 = 1 - wt; y = 0;
//    A0 = A + z*ha*wa + xa*ha; A1 = A0 + ha, A2 = A1 + ha, A3 = A2 + ha; B0 = B + z*hb*wb + xb*hb;
//    // variables for SSE (simple casts to float)
//    float *Af0, *Af1, *Af2, *Af3, *Bf0, *Cf, *ywtsf, wtf, wt1f;
//    Af0 = (float*)A0; Af1 = (float*)A1; Af2 = (float*)A2; Af3 = (float*)A3;
//    Bf0 = (float*)B0; Cf = (float*)C;
//    ywtsf = (float*)ywts; wtf = (float)wt; wt1f = (float)wt1;
//    // resample along x direction (A -> C)
//#define FORs(X) if(sse) for(; y<ha-4; y+=4) STR(Cf[y],X);
//#define FORr(X) for(; y<ha; y++) C[y] = X;
//    if (wa == 2 * wb) {
//      FORs(ADD(LDu(Af0[y]), LDu(Af1[y])));
//      FORr(A0[y] + A1[y]); x1 += 2;
//    }
//    else if (wa == 3 * wb) {
//      FORs(ADD(LDu(Af0[y]), LDu(Af1[y]), LDu(Af2[y])));
//      FORr(A0[y] + A1[y] + A2[y]); x1 += 3;
//    }
//    else if (wa == 4 * wb) {
//      FORs(ADD(LDu(Af0[y]), LDu(Af1[y]), LDu(Af2[y]), LDu(Af3[y])));
//      FORr(A0[y] + A1[y] + A2[y] + A3[y]); x1 += 4;
//    }
//    else if (wa>wb) {
//      int m = 1; while (x1 + m<wn && xb == xbs[x1 + m]) m++; float wtsf[4];
//      for (int x0 = 0; x0<(m<4 ? m : 4); x0++) wtsf[x0] = float(xwts[x1 + x0]);
//#define U(x) MUL( LDu(*(Af ## x + y)), SET(wtsf[x]) )
//#define V(x) *(A ## x + y) * xwts[x1+x]
//      if (m == 1) { FORs(U(0));                     FORr(V(0)); }
//      if (m == 2) { FORs(ADD(U(0), U(1)));           FORr(V(0) + V(1)); }
//      if (m == 3) { FORs(ADD(U(0), U(1), U(2)));      FORr(V(0) + V(1) + V(2)); }
//      if (m >= 4) { FORs(ADD(U(0), U(1), U(2), U(3))); FORr(V(0) + V(1) + V(2) + V(3)); }
//#undef U
//#undef V
//      for (int x0 = 4; x0<m; x0++) {
//        A1 = A0 + x0*ha; wt1 = xwts[x1 + x0]; Af1 = (float*)A1; wt1f = float(wt1); y = 0;
//        FORs(ADD(LD(Cf[y]), MUL(LDu(Af1[y]), SET(wt1f)))); FORr(C[y] + A1[y] * wt1);
//      }
//      x1 += m;
//    }
//    else {
//      bool xBd = x<xbd[0] || x >= wb - xbd[1]; x1++;
//      if (xBd) memcpy(C, A0, ha*sizeof(float));
//      if (!xBd) FORs(ADD(MUL(LDu(Af0[y]), SET(wtf)), MUL(LDu(Af1[y]), SET(wt1f))));
//      if (!xBd) FORr(A0[y] * wt + A1[y] * wt1);
//    }
//#undef FORs
//#undef FORr
//    // resample along y direction (B -> C)
//    if (ha == hb * 2) {
//      float r2 = r / 2; int k = ((~((size_t)B0) + 1) & 15) / 4; y = 0;
//      for (; y<k; y++)  B0[y] = (C[2 * y] + C[2 * y + 1])*r2;
//      if (sse) for (; y<hb - 4; y += 4) STR(Bf0[y], MUL((float)r2, _mm_shuffle_ps(ADD(
//        LDu(Cf[2 * y]), LDu(Cf[2 * y + 1])), ADD(LDu(Cf[2 * y + 4]), LDu(Cf[2 * y + 5])), 136)));
//      for (; y<hb; y++) B0[y] = (C[2 * y] + C[2 * y + 1])*r2;
//    }
//    else if (ha == hb * 3) {
//      for (y = 0; y<hb; y++) B0[y] = (C[3 * y] + C[3 * y + 1] + C[3 * y + 2])*(r / 3);
//    }
//    else if (ha == hb * 4) {
//      for (y = 0; y<hb; y++) B0[y] = (C[4 * y] + C[4 * y + 1] + C[4 * y + 2] + C[4 * y + 3])*(r / 4);
//    }
//    else if (ha>hb) {
//      y = 0;
//      //if( sse && ybd[0]<=4 ) for(; y<hb; y++) // Requires SSE4
//      //  STR1(Bf0[y],_mm_dp_ps(LDu(Cf[yas[y*4]]),LDu(ywtsf[y*4]),0xF1));
//#define U(o) C[ya+o]*ywts[y*4+o]
//      if (ybd[0] == 2) for (; y<hb; y++) { ya = yas[y * 4]; B0[y] = U(0) + U(1); }
//      if (ybd[0] == 3) for (; y<hb; y++) { ya = yas[y * 4]; B0[y] = U(0) + U(1) + U(2); }
//      if (ybd[0] == 4) for (; y<hb; y++) { ya = yas[y * 4]; B0[y] = U(0) + U(1) + U(2) + U(3); }
//      if (ybd[0]>4)  for (; y<hn; y++) { B0[ybs[y]] += C[yas[y]] * ywts[y]; }
//#undef U
//    }
//    else {
//      for (y = 0; y<ybd[0]; y++) B0[y] = C[yas[y]] * ywts[y];
//      for (; y<hb - ybd[1]; y++) B0[y] = C[yas[y]] * ywts[y] + C[yas[y] + 1] * (r - ywts[y]);
//      for (; y<hb; y++)        B0[y] = C[yas[y]] * ywts[y];
//    }
//  }
//  alFree(xas); alFree(xbs); alFree(xwts); alFree(C);
//  alFree(yas); alFree(ybs); alFree(ywts);
//}
//
//float src_1[1000 * 1000 * 3];
//
/////
///// Fast bilinear image downsampling/upsampling, using SSE
///// param[in]: src: inputting image, which the data is unsigned char
///// param[out]: dst: resampled image
/////
//void ImResample_mex(AuMat& src, AuMat* dst)
//{
//  float* ptr = (float*)src.data_;
//  int w = src.cols_;
//  int h = src.rows_;
//  int d = src.channels_;
//  float norm = 1.0;
//  if (d > 1)
//  {
//    int y, x;
//    for (y = 0; y < h; y++)
//    {
//      for (x = 0; x < w; x++)
//      {
//        src_1[(y*w + x)] = ptr[(y*w + x) * 3]; // first layout
//        src_1[(y*w + x) + w*h] = ptr[(y*w + x) * 3 + 1]; // second layout
//        src_1[(y*w + x) + w*h * 2] = ptr[(y*w + x) * 3 + 2]; // third layout
//      }
//    }
//    resample(src_1, (float*)dst->data_, src.cols_, dst->cols_, src.rows_, dst->rows_, d, norm);
//  }
//  else
//  {
//    resample((float*)src.data_, (float*)dst->data_, src.cols_, dst->cols_, src.rows_, dst->rows_, d, norm);
//  }
//}
