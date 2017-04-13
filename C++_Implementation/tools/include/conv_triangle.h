/// Copyright (c) 2016 AutelRobotics. All Rights Reserved
///
/// \conv_triangle.h
///
/// \Definition : Execute convolution with triangle filter.
///
///  \author XipengCui
///  \Date : Created on: Jan 1, 2017
///
#ifndef _CONV_TRIANGLE_H_
#define _CONV_TRIANGLE_H_

///
/// Extremely fast 2D image convolution with a triangle filter.
///
/// \param[in] src inputting data, such as: image after normalizing
/// \param[in] r integer filter radius
/// \param[in] w width of src
/// \param[in] h height of src
/// \param[in] d channels of src
/// \param[out] dst smoothed image
///
void ConvTriangle(float* src, float* dst, int r, int w, int h, int d);

// convolve I by a [1 p 1] filter (uses SSE)
void ConvTri1(float *I, float *O, int h, int w, int d, float p, int s);

// convolve I by a 2rx1 triangle filter (uses SSE)
void ConvTri(float *I, float *O, int h, int w, int d, int r, int s);

#endif
