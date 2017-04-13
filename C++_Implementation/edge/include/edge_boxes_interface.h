/// Copyright (c) 2016 AutelRobotics. All Rights Reserved
///
/// \edge_boxes_interface.h
///
/// \Definition : This is interface function of edge box.
///
///  \author XipengCui
///  \Date : Created on: Jan 1, 2017
///
#ifndef _EDGE_BOXES_INTERFACE_H_
#define _EDGE_BOXES_INTERFACE_H_

#include "../../tools/include/data_define.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>

using namespace autel;
using namespace computer_vision;

typedef struct {
  float alpha;
  float beta;
  float eta;
  float minScore;
  float maxBoxes;
  float edgeMinMag;
  float edgeMergeThr;
  float clusterMinMag;
  float maxAspectRatio;
  float minBoxArea;
  float gamma;
  float kappa;
} edge_box_parameters;

typedef struct{
  int imWidth;
  int gtWidth;
  int nPos;
  int nNeg;
  int nImgs;
  int nTrees;
  float fracFtrs;
  int minCount;
  int minChild;
  int maxDepth;
  char discretize[128];
  int nSamples;
  int nClasses;
  char split[128];
  int nOrients;
  int grdSmooth;
  int chnSmooth;
  int simSmooth;
  int normRad;
  int shrink;
  int nCells;
  int rgbd;
  int stride;
  int multiscale;
  int sharpen;
  int nTreesEval;
  int nThreads;
  int nms;
  int seed;
  int useParfor;
  char modelDir[128];
  char modelFnm[128];
  char bsdsDir[128];
  int nChns;
  int nChnFtrs;
  int nSimFtrs;
  int nTotFtrs;
} model_opts;

///
/// It is interface of edge box algorithm.
///
/// \param[in] img image data
/// \param[in] box input box
/// \param[in] memory_ptr temporary memory
/// \param[out] best box
///
std::vector<cv::Vec4i> EdgeBoxInterface(AuMat& img);

///
/// Initialize many parameters for edge box.
///
/// \param[in] model_path the path of model_train.txt
///
void InitEdgeBox(char* model_path);

///
/// Get best box from edge box algorithm.
///
/// \param[in] image image data
/// \param[out] rect best box.
///
std::vector<cv::Vec4i> GetEdgeBoxImg(AuMat& img);

///
/// Compute features for structured edge detection.
///
/// \param[in] img image data
/// \param[in] g_modelEdgeBox structured edge model options
/// \param[out] chns_reg [h x w x nChannel] regular output channels
/// \param[out] chns_sim [h x w x nChannel] self-similarity output channels
///
void GetEdgeChnsFeatures(AuMat& img);

///
/// Perform Non-Maximal Suppression(NMS).
///
/// \param[in]: E_output original edge map
/// \param[in]: O orientation map
/// \param[in]: r radius for nms supr
/// \param[in]: s radius for supr boundaries
/// \param[in]: m multiplier for conservative supr
/// \param[int]: nThreads number of threads for evaluation
/// \param[out]: E edge probability map
///
void EdgesNMS(float* E0, float* _O, int r, int s, float m, int nThreads, int h, int w, float* E);


#endif
