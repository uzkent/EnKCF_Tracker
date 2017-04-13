/// Copyright (c) 2016 AutelRobotics. All Rights Reserved
///
/// \edge_box.h
///
/// \Definition : Find many boxes in the image.
///
///  \author XipengCui
///  \Date : Created on: Jan 1, 2017
///
#ifndef _EDGE_BOX_GENERATOR_H_
#define _EDGE_BOX_GENERATOR_H_

#include "edge_boxes_interface.h"
#include <math.h>
#include <algorithm>
#include <vector>
using namespace std;
#define PI 3.1415926535897f

///
/// v is in the [a, b], return v
///
static int clamp(int v, int a, int b)
{
  return v<a ? a : v>b ? b : v;
}

// trivial array class encapsulating pointer arrays
template <class T> class Array
{
public:
  Array() { _h = _w = 0; _x = 0; _free = 0; }
  virtual ~Array() { clear(); }
  void clear() { if (_free) delete[] _x; _h = _w = 0; _x = 0; _free = 0; }
  void init(int h, int w) { clear(); _h = h; _w = w; _x = new T[h*w](); _free = 1; }
  T& val(size_t c, size_t r) { return _x[c*_h + r]; }
  int _h, _w; T *_x; bool _free;
};

// convenient typedefs
typedef vector<float> vectorf;
typedef vector<int> vectori;
typedef Array<float> arrayf;
typedef Array<int> arrayi;

// bounding box data structures and routines
typedef struct { int c, r, w, h; float s; } Box;
typedef vector<Box> Boxes;
static bool boxesCompare(const Box &a, const Box &b)
{
  return a.s<b.s;
}

float boxesOverlap(Box &a, Box &b);
void boxesNms(Boxes &boxes, float thr, float eta, int maxBoxes);

// main class for generating edge boxes
class EdgeBoxGenerator
{
public:
  // method parameters (must be manually set)
  float _alpha, _beta, _eta, _minScore; int _maxBoxes;
  float _edgeMinMag, _edgeMergeThr, _clusterMinMag;
  float _maxAspectRatio, _minBoxArea, _gamma, _kappa;

  // main external routine (set parameters first)
  void generate(Boxes &boxes, arrayf &E, arrayf &O, arrayf &V);

private:
  // edge segment information (see clusterEdges)
  int h, w;                         // image dimensions
  int _segCnt;                      // total segment count
  arrayi _segIds;                   // segment ids (-1/0 means no segment)
  vectorf _segMag;                  // segment edge magnitude sums
  vectori _segR, _segC;             // segment lower-right pixel
  vector<vectorf> _segAff;          // segment affinities
  vector<vectori> _segAffIdx;       // segment neighbors

  // data structures for efficiency (see prepDataStructs)
  arrayf _segIImg, _magIImg; arrayi _hIdxImg, _vIdxImg;
  vector<vectori> _hIdxs, _vIdxs; vectorf _scaleNorm;
  float _scStep, _arStep, _rcStepRatio;

  // data structures for efficiency (see scoreBox)
  arrayf _sWts; arrayi _sDone, _sMap, _sIds; int _sId;

  // helper routines
  void clusterEdges(arrayf &E, arrayf &O, arrayf &V);
  void prepDataStructs(arrayf &E);
  void scoreAllBoxes(Boxes &boxes);
  void scoreBox(Box &box);
  void refineBox(Box &box);
  void drawBox(Box &box, arrayf &E, arrayf &V);
};

///
/// Generate Edge Boxes object proposals in given image.
///
/// \param[in] E edge probability map
/// \param[in] _O approximate orientation
/// \param[in] opt Structured Edge model trained with edgesTrain(in Matlab)
/// \param[in] h height of E
/// \param[in] w width of E
/// \param[out] bbs [nx5] array containing proposal bbs [x y w h score]
/// \param[out] num number of bbs
///
void EdgeBoxes(float* E, float* _O, int h, int w, edge_box_parameters opt, float* bbs, int* num);



#endif
