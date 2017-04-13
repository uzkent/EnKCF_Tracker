/// Copyright (c) 2016 AutelRobotics. All Rights Reserved
///
/// \edge_boxes_interface.cpp
///
/// \Definition : This is interface function of edge box, include InitEdgeBox, EdgeBoxInterface
///
///  \author XipengCui
///  \Date : Created on: Jan 1, 2017
///
#include "../include/edge_boxes_interface.h"
#include "../../tools/include/image_tools.h"
#include "../../tools/include/conv_triangle.h"
#include "../../tools/include/gradient.h"
#include "../include/edge_detection.h"
#include "../include/edge_box.h"
#include <iostream>

edge_box_parameters g_opts;
model_opts g_modelEdgeBox;

// memory of loading trained data, about 210M
float g_thrs[79861*8];
unsigned int g_fids[79861*8];
unsigned int g_child[79861 * 8];
unsigned char g_segs[16*16*79861*8];
unsigned char g_nSegs[79861 * 8];
unsigned short g_eBins[19306620];
unsigned int g_eBnds[1916665];

AuRect<float> g_inputBox;

/// memory for program
float* memory_ptr;
float* chns_reg;
float* chns_sim;

void InitEdgeBox(char* model_path) {

  chns_reg = NULL;
  chns_sim = NULL;
  // init edge box parameters
  g_opts.alpha = 0.65;
  g_opts.beta = 0.75;
  g_opts.eta = 1;
  g_opts.minScore = 0.01;
  g_opts.maxBoxes = 10000;
  g_opts.edgeMinMag = 0.1;
  g_opts.edgeMergeThr = 0.5;
  g_opts.clusterMinMag = 0.5;
  g_opts.maxAspectRatio = 3;
  g_opts.minBoxArea = 1000;
  g_opts.gamma = 2;
  g_opts.kappa = 1.5;

  // init model parameters
  g_modelEdgeBox.imWidth = 32;
  g_modelEdgeBox.gtWidth = 16;
  g_modelEdgeBox.nPos = 500000;
  g_modelEdgeBox.nNeg = 500000;
  g_modelEdgeBox.nTrees = 8;
  g_modelEdgeBox.fracFtrs = 0.25;
  g_modelEdgeBox.minCount = 1;
  g_modelEdgeBox.minChild = 8;
  g_modelEdgeBox.maxDepth = 64;
  g_modelEdgeBox.nSamples = 256;
  g_modelEdgeBox.nClasses = 2;
  g_modelEdgeBox.nOrients = 4;
  g_modelEdgeBox.grdSmooth = 0;
  g_modelEdgeBox.chnSmooth = 2;
  g_modelEdgeBox.simSmooth = 8;
  g_modelEdgeBox.normRad = 4;
  g_modelEdgeBox.shrink = 2;
  g_modelEdgeBox.nCells = 5;
  g_modelEdgeBox.rgbd = 0;
  g_modelEdgeBox.stride = 2;
  g_modelEdgeBox.multiscale = 0;
  g_modelEdgeBox.sharpen = 2;
  g_modelEdgeBox.nTreesEval = 4;
  g_modelEdgeBox.nThreads = 4;
  g_modelEdgeBox.nms = 0;
  g_modelEdgeBox.seed = 1;
  g_modelEdgeBox.useParfor = 0;
  g_modelEdgeBox.nChns = 13;
  g_modelEdgeBox.nChnFtrs = 3328;
  g_modelEdgeBox.nSimFtrs = 3900;
  g_modelEdgeBox.nTotFtrs = 7228;

  //read data from model_train which is trained in MATLAB
  FILE* fp = fopen(model_path, "rb+");
  if (fp == NULL)
  {
    char model_path_tmp[256] = "./model_train.txt";
    fp = fopen(model_path_tmp, "rb+");
    if(fp == NULL)
    {
      fp = fopen("/tmp/SD0/model_train.txt", "rb+");
      if(fp == NULL)
      {
        printf("model_train.txt open failed!\n");
        return;
      }
    }
  }
  unsigned int h = 0, w = 0, d = 0;
  long long len = 0;
  //1.read thrs, which type is float
  fread(&h, sizeof(unsigned int), 1, fp);
  fread(&w, sizeof(unsigned int), 1, fp);
  fread(&d, sizeof(unsigned int), 1, fp);
  len = h*w*d;
  fread(g_thrs, sizeof(float), len, fp);
  //2.read fids, which type is unsigned int
  fread(&h, sizeof(unsigned int), 1, fp);
  fread(&w, sizeof(unsigned int), 1, fp);
  fread(&d, sizeof(unsigned int), 1, fp);
  len = h*w*d;
  fread(g_fids, sizeof(int), len, fp);
  //3.read child, which type is unsigned int
  fread(&h, sizeof(unsigned int), 1, fp);
  fread(&w, sizeof(unsigned int), 1, fp);
  fread(&d, sizeof(unsigned int), 1, fp);
  len = h*w*d;
  fread(g_child, sizeof(int), len, fp);
  //4.read segs, which type is unsigned char
  fread(&h, sizeof(unsigned int), 1, fp);
  fread(&w, sizeof(unsigned int), 1, fp);
  fread(&d, sizeof(unsigned int), 1, fp);
  len = h*w*d;
  fread(g_segs, sizeof(char), len, fp);
  //5.read nSegs, which type is unsigned char
  fread(&h, sizeof(unsigned int), 1, fp);
  fread(&w, sizeof(unsigned int), 1, fp);
  fread(&d, sizeof(unsigned int), 1, fp);
  len = h*w*d;
  fread(g_nSegs, sizeof(char), len, fp);
  //6.read eBins, which type is unsigned short
  fread(&h, sizeof(unsigned int), 1, fp);
  fread(&w, sizeof(unsigned int), 1, fp);
  fread(&d, sizeof(unsigned int), 1, fp);
  len = h*w*d;
  fread(g_eBins, sizeof(short), len, fp);
  //7.read eBnds, which type is unsigned short
  fread(&h, sizeof(unsigned int), 1, fp);
  fread(&w, sizeof(unsigned int), 1, fp);
  fread(&d, sizeof(unsigned int), 1, fp);
  len = h*w*d;
  fread(g_eBnds, sizeof(int), len, fp);

  fclose(fp);
}

std::vector<cv::Vec4i> EdgeBoxInterface(AuMat& image) {

  std::vector<cv::Vec4i> boxes = GetEdgeBoxImg(image);

  return boxes;
}

std::vector<cv::Vec4i> GetEdgeBoxImg(AuMat& img) {
  
  g_modelEdgeBox.nms = 0;
  AuSize img_size;
  img_size.width_ = img.cols_;
  img_size.height_ = img.rows_;

  //edge detection department
  if (g_modelEdgeBox.nTrees < g_modelEdgeBox.nTreesEval)
  {
    g_modelEdgeBox.nTreesEval = g_modelEdgeBox.nTrees;
  }
  if (g_modelEdgeBox.shrink > g_modelEdgeBox.stride)
  {
    g_modelEdgeBox.stride = g_modelEdgeBox.shrink;
  }

  //multiscale
  if (g_modelEdgeBox.multiscale){
    // No multiscale in edge box algorithm
  }
  else
  {
    int r = g_modelEdgeBox.imWidth / 2;
    int p[4] = { r, r, r, r }; //T, B, L, R
    p[1] += (4 - (img.rows_ + 2 * r) % 4) % 4;
    p[3] += (4 - (img.cols_ + 2 * r) % 4) % 4;

    char fill_style[128] = "symmetric";

    // ImPad(img, p, fill_style);

    GetEdgeChnsFeatures(img);


    float* memory_mag = new float[img.cols_*img.rows_*img.channels_];
    if(memory_ptr == NULL)
      memory_ptr = new float[img_size.width_*img_size.height_];

    if (g_modelEdgeBox.sharpen > 0)
    {
      //
      AuMat image;
      image.channels_ = img.channels_;
      image.cols_ = img.cols_;
      image.rows_ = img.rows_;
      image.depth_ = AU_32F;
      image.dims_ = img.dims_;
      image.step_[1] = image.channels_;
      image.step_[0] = image.cols_*image.channels_;
      float* temp_ptr = new float[image.cols_*image.rows_*image.channels_];
      image.data_ = (unsigned char*)temp_ptr;

      // every element multiply 1.0/255;
      NormalizeImage(img, &image, img.rows_*img.step_[0], 1.0 / 255);

      // Extremely fast 2D image convolution with a triangle filter.
      ConvTriangle(temp_ptr, memory_mag, 1, image.cols_, image.rows_, image.channels_);
      delete [] temp_ptr;
    }

    AuMat image;
    image.channels_ = img.channels_;
    image.cols_ = img.cols_;
    image.rows_ = img.rows_;
    image.depth_ = AU_32F;
    image.dims_ = img.dims_;
    image.step_[1] = image.channels_;
    image.step_[0] = image.cols_*image.channels_;
    image.data_ = (unsigned char*)memory_mag;
    int E_h = 0;
    int E_w = 0;

    float* memory_edge = new float[(image.cols_+g_modelEdgeBox.gtWidth)*(image.rows_+g_modelEdgeBox.gtWidth)];
    memset(memory_edge, 0, (image.cols_+g_modelEdgeBox.gtWidth)*(image.rows_+g_modelEdgeBox.gtWidth)*sizeof(float));

    EdgeDetect(image, memory_edge, &E_w, &E_h, chns_reg, chns_sim, g_modelEdgeBox, g_thrs, g_fids, g_child, g_segs, g_nSegs, g_eBins, g_eBnds);
    delete [] memory_mag;
    int tmp = E_w;
    E_w = E_h; // exchange
    E_h = E_w;

    float t = g_modelEdgeBox.stride*g_modelEdgeBox.stride;
    t /= (g_modelEdgeBox.gtWidth*g_modelEdgeBox.gtWidth);
    t /= g_modelEdgeBox.nTreesEval;
    r = g_modelEdgeBox.gtWidth / 2;

    if (g_modelEdgeBox.sharpen == 0)
      t = t * 2.0;
    else if (g_modelEdgeBox.sharpen == 1)
      t = t * 1.8;
    else
      t = t * 1.66;

    //// E_w, E_h ----> img_size.width_, img_size.height_
    int x, y;
    for (y = r; y < img_size.height_ + r; ++y)
    {
      for (x = r; x < img_size.width_ + r; ++x)
      {
        memory_edge[(y - r)*img_size.width_ + x - r] = memory_edge[y*E_w + x] * t;
      }
    }
    // Extremely fast 2D image convolution with a triangle filter.
    ConvTriangle(memory_edge, memory_ptr, 1, img_size.width_, img_size.height_, 1);
    delete [] memory_edge;
  }

  float* memory_ori = chns_sim;;
  if (g_modelEdgeBox.nms != -1)
  {
    int w = img_size.width_;
    int h = img_size.height_;

    float* Ox = chns_reg;
    float* Oy = chns_sim;
    float* Oxx = new float[w*h];
    float* Oyy = new float[w*h];
    float* Oxy = new float[w*h];

    float* memory_edge = Oxy;
    ConvTriangle(memory_ptr, memory_edge, 4, img_size.width_, img_size.height_, 1);

    // Compute numerical gradients along x and y directions
    Gradient2(memory_edge, Ox, Oy, img_size.width_, img_size.height_, 1);
    Gradient2(Ox, Oxx, Oyy, img_size.width_, img_size.height_, 1);
    Gradient2(Oy, Oxy, Oyy, img_size.width_, img_size.height_, 1);

    int x, y;
    for (y = 0; y < img_size.height_; ++y)
    {
      for (x = 0; x < img_size.width_; ++x)
      {
        float o_yy = Oyy[y*img_size.width_ + x];
        float o_xy = Oxy[y*img_size.width_ + x];
        float o_xx = Oxx[y*img_size.width_ + x];

        if (o_xy > 0.00001)
          o_xy = -1;
        else if (o_xy < -0.00001)
          o_xy = 1;

        o_yy *= o_xy;
        o_yy /= (o_xx + 0.00001);
        o_yy = atan(o_yy);

        // calculating mod function in Matlab
        if (o_yy < 0.00001 && o_yy > -0.00001)
        {
          memory_ori[y*img_size.width_ + x] = o_yy;
        }
        else
        {
          float s = o_yy / PI;
          int n = (int)s;
          if (n > s)
            n -= 1;
          memory_ori[y*img_size.width_ + x] = o_yy - n*PI;
        }
      }
    }//end matrix
    delete [] Oxx; delete [] Oyy; delete [] Oxy;
  }

  ////////// end edge detection
  float* ptr_edge = chns_reg;
  EdgesNMS(memory_ptr, memory_ori, 2, 0, 1, g_modelEdgeBox.nThreads, img_size.width_, img_size.height_, ptr_edge); //w, h --> h, w

  int box_num = 0;
  EdgeBoxes(ptr_edge, memory_ori, img_size.width_, img_size.height_, g_opts, memory_ptr, &box_num); //w, h --> h, w
  
  // Return All Boxes Back
  int idx = 0;
  float* ptr = memory_ptr;
  std::cout << box_num << std::endl;
  std::vector<cv::Vec4i> boxes_final;
  for (int i = 0; i < box_num; i++){
     boxes_final.push_back(cv::Vec4i(ptr[i*5+1],ptr[i*5+0],ptr[i*5+3],ptr[i*5+2]));
  } 
  
  // Delete Temporary Memory
  delete [] memory_ptr;
  memory_ptr = NULL;

  return boxes_final;

}

void GetEdgeChnsFeatures(AuMat& _img)
{
  float shrink = g_modelEdgeBox.shrink;
  int w = _img.cols_/shrink;
  int h = _img.rows_/shrink;
  if(chns_sim == NULL)
  {
    chns_sim = new float[w*h*sizeof(float)*13];
    chns_reg = new float[w*h*sizeof(float)*13];
  }
  float* memory_chns = new float[w*h*sizeof(float)*13];
  memset(memory_chns+w*h * 3*sizeof(float), 0, w*h*4*10);
  // rgb -> luv
  AuMat image;
  image.channels_ = _img.channels_;
  image.cols_ = _img.cols_;
  image.rows_ = _img.rows_;
  image.depth_ = _img.depth_;
  image.dims_ = _img.dims_;
  image.step_[1] = image.channels_;
  image.step_[0] = image.cols_*image.channels_;
  image.data_ = (unsigned char*)(memory_chns+4*w*h);

  // RGB -> LUV
  RGB2LUV(_img, &image, _img.rows_*_img.cols_, 1.0/255);

  AuMat img;
  img.channels_ = _img.channels_;
  img.depth_ = _img.depth_;
  img.dims_ = _img.dims_;
  img.cols_ = _img.cols_ / shrink;// shrink value is: 2
  img.rows_ = _img.rows_ / shrink;
  img.step_[1] = img.channels_;
  img.step_[0] = img.cols_ * img.step_[1];
  img.data_ = (unsigned char*)chns_reg;

  ImResample(image, &img);

  float* ptr = (float*)img.data_;

  memcpy(memory_chns, ptr, w*h * 3*sizeof(float));

  int index_chns = w*h*3;
  ///// calcute chns; 0-2: img; 3: M; 4-7: H; 8: M; 9-12: H. two processing: gradient --> sample; sample --> gradient.
  int idx = 0;
  for (idx = 0; idx < 2; idx++)
  {
    if (idx == 0)// initialization size
    {
      float* ptr_mag = chns_sim;
      float* ptr_ori = memory_chns + 9*w*h;
      GradientMag(image, ptr_mag, ptr_ori, 0, g_modelEdgeBox.normRad, 0.01, 1);

      float* H_1 = memory_chns + index_chns + w*h;
      GradientHist(ptr_mag, ptr_ori, H_1, image.rows_, image.cols_, 2, g_modelEdgeBox.nOrients, 0);//ok

      // don't use copy constructor, because it will allocat memory and release memory out of this scope
      AuMat image_m;
      image_m.channels_ = 1;
      image_m.cols_ = image.cols_;
      image_m.rows_ = image.rows_;
      image_m.depth_ = AU_32F;
      image_m.dims_ = image.dims_;
      image_m.step_[1] = image_m.channels_;
      image_m.step_[0] = image_m.cols_*image_m.channels_;
      image_m.data_ = (unsigned char*)ptr_mag;

      AuMat image_output;
      image_output.channels_ = 1;
      image_output.depth_ = AU_32F;
      image_output.dims_ = image.dims_;
      image_output.cols_ /= 2;
      image_output.rows_ /= 2;
      image_output.step_[1] = image_output.channels_;
      image_output.step_[0] = image_output.cols_ * image_output.step_[1];
      image_output.data_ = (unsigned char*)(memory_chns+index_chns);
      //ImResample_mex(image_m, &image_output);
      ImResample(image_m, &image_output);
      index_chns += 5*w*h;
    }
    else
    {
      float* ptr_m = memory_chns + index_chns;
      GradientMag(img, ptr_m, chns_sim, 0, g_modelEdgeBox.normRad, 0.01, 0); // M point to eighth plane of chns;
      float* H_1 = memory_chns + index_chns + w*h;
      GradientHist(ptr_m, chns_sim, H_1, h, w, 1, g_modelEdgeBox.nOrients, 0); // H point to ninth to twelfth plan of chns
    }
  }
  ///// end
  float chn_sm = g_modelEdgeBox.chnSmooth / (float)shrink;
  if (chn_sm > 1)
  {
    chn_sm = (int)(chn_sm + 0.5f);
  }
  float sim_sm = g_modelEdgeBox.simSmooth / (float)shrink;
  if (sim_sm > 1)
  {
    sim_sm = (int)(sim_sm + 0.5f);
  }

  ConvTriangle(memory_chns, chns_reg, chn_sm, w, h, 13);
  ConvTriangle(memory_chns, chns_sim, sim_sm, w, h, 13);

  //release memory
  delete [] memory_chns;
}

// return I[x,y] via bilinear interpolation
inline float interp(float *I, int h, int w, float x, float y) {
  x = x<0 ? 0 : (x>w - 1.001 ? w - 1.001 : x);
  y = y<0 ? 0 : (y>h - 1.001 ? h - 1.001 : y);
  int x0 = int(x), y0 = int(y), x1 = x0 + 1, y1 = y0 + 1;
  float dx0 = x - x0, dy0 = y - y0, dx1 = 1 - dx0, dy1 = 1 - dy0;
  return I[x0*h + y0] * dx1*dy1 + I[x1*h + y0] * dx0*dy1 +
    I[x0*h + y1] * dx1*dy0 + I[x1*h + y1] * dx0*dy0;
}

void EdgesNMS(float* E0, float* _O, int r, int s, float m, int nThreads, int h, int w, float* E) {

  // suppress edges where edge is stronger in orthogonal direction
  for (int x = 0; x<w; x++) for (int y = 0; y<h; y++) {
    float e = E[x*h + y] = E0[x*h + y];
    if (!e)
      continue;
    e *= m;
    float coso = cos(_O[x*h + y]), sino = sin(_O[x*h + y]);
    for (int d = -r; d <= r; d++) if (d) {
      float e0 = interp(E0, h, w, x + d*coso, y + d*sino);
      if (e < e0) {
        E[x*h + y] = 0;
        break;
      }
    }
  }

  // suppress noisy edge estimates near boundaries
  s = s>w / 2 ? w / 2 : s;
  s = s>h / 2 ? h / 2 : s;
  for (int x = 0; x<s; x++) for (int y = 0; y<h; y++) {
    E[x*h + y] *= x / float(s);
    E[(w - 1 - x)*h + y] *= x / float(s);
  }
  for (int x = 0; x<w; x++) for (int y = 0; y<s; y++) {
    E[x*h + y] *= y / float(s);
    E[x*h + (h - 1 - y)] *= y / float(s);
  }


}














