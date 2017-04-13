/// Copyright (c) 2016 AutelRobotics. All Rights Reserved
///
/// \gradient.h
///
/// \Definition : Calculate gradient magnitude and gradient hist;
///
///  \author XipengCui
///  \Date : Created on: Jan 1, 2017
///
#ifndef _GRADIENT_H_
#define _GRADIENT_H_
#include "data_define.h"
#include "conv_triangle.h"
#include "wrappers.h"
#include "string.h"
#include <math.h>
#include "sse.h"

using namespace autel;
using namespace computer_vision;

///
/// Compute gradient magnitude and orientation at each image location.
///
/// \param[in] img inputting image
/// \param[in] channel if>0 color channel to use for gradient computation
/// \param[in] norm_rad normalization radius (no normalization if 0)
/// \param[in] norm_const normalization constant
/// \param[in] is_layout if true, the image will be divided into several layers
/// \param[out] M gradient magnitude at each location
/// \param[out] O approximate gradient orientation modulo PI
///
void GradientMag(AuMat& img, float* M, float* O, int channel, int norm_rad, float norm_const, char is_layout);

///
/// Compute oriented gradient histograms.
///
/// \param[in] M gradient magnitude at each location
/// \param[in] O gradient orientation in range defined by param flag
/// \param[in] bin spatial bin size
/// \param[in] nOrients number of orientation bins
/// \param[in] softBin set soft binning (odd: spatial=soft, >=0: orient=soft)
/// \param[out] H gradient histograms
///
void GradientHist(float *M, float *O, float *H, int h, int w,
  int bin, int nOrients, int softBin);

///
/// Compute numerical gradients along x and y directions.
///
/// \param[in] I inputting image
/// \param[in] w width of E
/// \param[in] h height of E
/// \param[in] d channel of E
/// \param[out] G_x x-gradient (horizontal)
/// \param[out] G_y y-gradient (vertical)
///
void Gradient2(float* I, float* G_x, float* G_y, int w, int h, int d);

#endif
