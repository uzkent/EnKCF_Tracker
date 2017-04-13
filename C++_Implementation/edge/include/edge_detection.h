/// Copyright (c) 2016 AutelRobotics. All Rights Reserved
///
/// \edge_detection.h
///
/// \Definition : detect edge.
///
///  \author XipengCui
///  \Date : Created on: Jan 1, 2017
///
#ifndef _EDGE_DETECTION_H_
#define _EDGE_DETECTION_H_
#include "../../tools/include/data_define.h"
#include "../include/edge_boxes_interface.h"

///
/// Detect edges in image.
///
/// \param[in] image img data after ImPadding
/// \param[in] chns regular output channels
/// \param[in] chnsSs self-similarity output channels
/// \param[in] model_edgebox parameters of model
/// \param[out] E edge map
/// \param[out] E_w height of E
/// \param[out] E_h width of E
///
void EdgeDetect(AuMat& image, float* E, int* E_w, int* E_h, float* chns, float* chnsSs,
    model_opts model_edgebox, float* thrs, unsigned int* fids, unsigned int* child,
    unsigned char* segs, unsigned char* nSegs, unsigned short* eBins, unsigned int* eBnds);

#endif
