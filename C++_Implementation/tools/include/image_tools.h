/// Copyright (c) 2016 AutelRobotics. All Rights Reserved
///
/// \image_tools.h
///
/// \Definition : Image fundamental functions are defined here;
///
///  \author XipengCui
///  \Date : Created on: Jan 1, 2017
///
#ifndef _IMAGE_TOOLS_H_
#define _IMAGE_TOOLS_H_

#include "data_define.h"
using namespace autel;
using namespace computer_vision;

void Resize(unsigned char* src, unsigned char* dst, AuSize src_size, int src_step, int cn, AuSize dst_size, int dst_step, uchar* memoryPara);
void ResizeROI_whynotuse(unsigned char* src, unsigned char* dst, AuRect<int> roi, AuSize src_size, int src_step, int cn, AuSize dst_size, int dst_step);

///
/// Resize size of YUV image to any size.
///
/// \param[in] src_y: Y data of YUV image
/// \param[in] src_uv: U and V data of YUV image
/// \param[in] roi: patch in YUV image, here it is not used.
/// \param[in] src_size: width and height of source image(YUV)
/// \param[in] src_width: width of source image
/// \param[in] cn: channels
/// \param[in] dst_size: width and height of output image
/// \param[in] dst_width: width of output image
/// \param[in] data_format: data type
/// \param[out] dst_y: Y data of output image
/// \param[out] dst_uv: U and V data of output image
///
void ResizeYUVToYUV(unsigned char* src_y, unsigned char* src_uv, AuRect<int> roi, unsigned char* dst_y, unsigned char* dst_uv,
    AuSize src_size, int src_width, int cn, AuSize dst_size, int dst_width, int data_format);
///
/// Pad an image along its four boundaries
/// param[in]: img: old image
/// param[in]: p: size of img: img.cols + p[2]+p[3], img.rows+p[0]+p[1];
/// param[in]: style: how to fill data
/// param[out]: img: padded image
///
void ImPad(AuMat& img, int* p, char* style);

///
/// Convert RGB image to other color spaces
/// param[in]: I: RGB image
/// param[in]: n: number of image pixels
/// param[in]: nrm: normalizing coefficient
/// param[out]: J: other color spaces
///
void RGB2LUV(AuMat& I, AuMat* J, int n, float nrm);

///
/// Normalize image
/// param[in]: I: image
/// param[in]: n: number of image pixels
/// param[in]: nrm: normalizing coefficient
/// param[out]: J: normalization image
///
void NormalizeImage(AuMat& I, AuMat* J, int n, float nrm);

///
/// bilinear image downsampling/upsampling
/// param[in]: src: inputting image, which the data is float
/// param[out]: dst: resampled image
///
void ImResample(AuMat& src, AuMat* dst);
///
/// bilinear image downsampling/upsampling
/// param[in]: src: inputting image, which the data is unsigned char
/// param[out]: dst: resampled image
///
void ImResample_char(AuMat& src, AuMat* dst);

/// Fast bilinear image downsampling/upsampling, using SSE
/// param[in]: src: inputting image, which the data is unsigned char
/// param[out]: dst: resampled image
///
void ImResample_mex(AuMat& src, AuMat* dst);

#endif
