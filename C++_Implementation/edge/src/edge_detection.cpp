/// Copyright (c) 2016 AutelRobotics. All Rights Reserved
///
/// \edge_detection.cpp
///
/// \Definition : detect edge.
///
///  \author XipengCui
///  \Date : Created on: Jan 1, 2017
///
#include "../include/edge_detection.h"
#include <cstring>

typedef unsigned int uint32;
typedef unsigned short uint16;
typedef unsigned char uint8;

// construct lookup array for mapping fids to channel indices
uint32* buildLookup(int *dims, int w) {
  int c, r, z, n = w*w*dims[2];
  uint32 *cids = new uint32[n];
  n = 0;
  for (z = 0; z < dims[2]; z++) {
    for (c = 0; c < w; c++) {
      for (r = 0; r < w; r++) {
        cids[n++] = z*dims[0] * dims[1] + c*dims[0] + r;
      }
    }
  }
  return cids;
}

// construct lookup arrays for mapping fids for self-similarity channel
void buildLookupSs(uint32 *&cids1, uint32 *&cids2, int *dims, int w, int m) {
  int i, j, z, z1, c, r;
  int locs[1024];
  int m2 = m*m, n = m2*(m2 - 1) / 2 * dims[2], s = int(w / m / 2.0 + .5);
  cids1 = new uint32[n];
  cids2 = new uint32[n];
  n = 0;
  for (i = 0; i < m; i++) {
    locs[i] = uint32((i + 1)*(w + 2 * s - 1) / (m + 1.0) - s + .5);
  }
  for (z = 0; z < dims[2]; z++) {
    for (i = 0; i < m2; i++) {
      for (j = i + 1; j < m2; j++) {
        z1 = z*dims[0] * dims[1]; n++;
        r = i%m; c = (i - r) / m; cids1[n - 1] = z1 + locs[c] * dims[0] + locs[r];
        r = j%m; c = (j - r) / m; cids2[n - 1] = z1 + locs[c] * dims[0] + locs[r];
      }
    }
  }
}

void EdgeDetect(AuMat& image, float* E, int* E_w, int* E_h, float* chns, float* chnsSs,
    model_opts model_edgebox, float* thrs, unsigned int* fids, unsigned int* child,
    unsigned char* segs, unsigned char* nSegs, unsigned short* eBins, unsigned int* eBnds)
{
  float *I = (float*)image.data_;
  const int shrink = model_edgebox.shrink;
  const int imWidth = model_edgebox.imWidth;
  const int gtWidth = model_edgebox.gtWidth;
  const int nChns = model_edgebox.nChns;
  const int nCells = model_edgebox.nCells;
  const uint32 nChnFtrs = model_edgebox.nChnFtrs;
  const int stride = model_edgebox.stride;
  const int nTreesEval = model_edgebox.nTreesEval;
  int sharpen = model_edgebox.sharpen;
  int nThreads = model_edgebox.nThreads;
  const int nBnds = 3;// 1916665.0 / (79861.0 * 8.0);
  if (sharpen>nBnds - 1) {
    sharpen = nBnds - 1;
  }

  uint32* ind = new uint32[1000 * 1000 * 4];
  // get dimensions and constants
  const int w = image.rows_;
  const int h = image.cols_;
  const int Z = image.channels_;
  const int nTreeNodes = 79861;// (int)fidsSize[0];
  const int nTrees = 8;// (int)fidsSize[1];
  const int h1 = (int)ceil(double(h - imWidth) / stride);
  const int w1 = (int)ceil(double(w - imWidth) / stride);
  const int h2 = h1*stride + gtWidth;
  const int w2 = w1*stride + gtWidth;
  const int imgDims[3] = { h, w, Z };
  const int chnDims[3] = { h / shrink, w / shrink, nChns };
  const int indDims[3] = { h1, w1, nTreesEval };
  const int outDims[3] = { h2, w2, 1 };
  const int segDims[5] = { gtWidth, gtWidth, h1, w1, nTreesEval };

  // construct lookup tables
  uint32 *iids, *eids, *cids, *cids1, *cids2;
  iids = buildLookup((int*)imgDims, gtWidth);
  eids = buildLookup((int*)outDims, gtWidth);
  cids = buildLookup((int*)chnDims, imWidth / shrink);
  buildLookupSs(cids1, cids2, (int*)chnDims, imWidth / shrink, nCells);

  *E_w = w2;
  *E_h = h2;
  // apply forest to all patches and store leaf inds
  for (int c = 0; c<w1; c++) for (int t = 0; t<nTreesEval; t++) {
    for (int r0 = 0; r0<2; r0++) for (int r = r0; r<h1; r += 2) {
      int o = (r*stride / shrink) + (c*stride / shrink)*h / shrink;
      // select tree to evaluate
      int t1 = ((r + c) % 2 * nTreesEval + t) % nTrees;
      uint32 k = t1*nTreeNodes;
      while (child[k]) {
        // compute feature (either channel or self-similarity feature)
        uint32 f = fids[k];
        float ftr;
        if (f < nChnFtrs) {
          ftr = chns[cids[f] + o];
        }
        else {
          ftr = chnsSs[cids1[f - nChnFtrs] + o] - chnsSs[cids2[f - nChnFtrs] + o];
        }
        // compare ftr to threshold and move left or right accordingly
        if (ftr < thrs[k]) {
          k = child[k] - 1;
        }
        else {
          k = child[k];
        }
        k += t1*nTreeNodes;
      }
      // store leaf index and update edge maps
      ind[r + c*h1 + t*h1*w1] = k;
    }
  }

  // compute edge maps (avoiding collisions from parallel executions)
  if (!sharpen) {
    for (int c0 = 0; c0 < gtWidth / stride; c0++) {
      for (int c = c0; c < w1; c += gtWidth / stride) {
        for (int r = 0; r < h1; r++) {
          for (int t = 0; t < nTreesEval; t++) {
            uint32 k = ind[r + c*h1 + t*h1*w1];
            float *E1 = E + (r*stride) + (c*stride)*h2;
            int b0 = eBnds[k*nBnds], b1 = eBnds[k*nBnds + 1];
            if (b0 == b1)
              continue;
            for (int b = b0; b < b1; b++)
              E1[eids[eBins[b]]]++;
          }
        }
      }
    }
  }

  // computed sharpened edge maps, snapping to local color values
  if (sharpen) {
    // compute neighbors array
    const int g = gtWidth;
    uint16 N[4096 * 4];
    for (int c = 0; c < g; c++) {
      for (int r = 0; r < g; r++) {
        int i = c*g + r;
        uint16 *N1 = N + i * 4;
        N1[0] = c>0 ? i - g : i;
        N1[1] = c < g - 1 ? i + g : i;
        N1[2] = r > 0 ? i - 1 : i;
        N1[3] = r < g - 1 ? i + 1 : i;
      }
    }

    for (int c = 0; c < w1; c++) {
      for (int r = 0; r < h1; r++) {
        for (int t = 0; t < nTreesEval; t++) {
          // get current segment and copy into S
          uint32 k = ind[r + c*h1 + t*h1*w1];
          int m = nSegs[k];
          if (m == 1)
            continue;
          uint8 S0[4096], *S = S0;
          memcpy(S, segs + k*g*g, g*g*sizeof(uint8));
          // compute color model for each segment using every other pixel
          int ci, ri, s, z;
          float ns[100], mus[1000];
          const float *I1 = I + (c*stride + (imWidth - g) / 2)*h + r*stride + (imWidth - g) / 2;
          for (s = 0; s < m; s++) {
            ns[s] = 0;
            for (z = 0; z < Z; z++)
              mus[s*Z + z] = 0;
          }
          for (ci = 0; ci < g; ci += 2) {
            for (ri = 0; ri < g; ri += 2) {
              s = S[ci*g + ri]; ns[s]++;
              for (z = 0; z < Z; z++)
                mus[s*Z + z] += I1[z*h*w + ci*h + ri];
            }
          }
          for (s = 0; s < m; s++) {
            for (z = 0; z < Z; z++)
              mus[s*Z + z] /= ns[s];
          }
          // update segment S according to local color values
          int b0 = eBnds[k*nBnds], b1 = eBnds[k*nBnds + sharpen];
          for (int b = b0; b < b1; b++) {
            float vs[10], d, e, eBest = 1e10f;
            int i, sBest = -1, ss[4];
            for (i = 0; i < 4; i++)
              ss[i] = S[N[eBins[b] * 4 + i]];
            for (z = 0; z < Z; z++)
              vs[z] = I1[iids[eBins[b]] + z*h*w];
            for (i = 0; i < 4; i++) {
              s = ss[i];
              if (s == sBest)
                continue;
              e = 0;
              for (z = 0; z < Z; z++) {
                d = mus[s*Z + z] - vs[z]; e += d*d;
              }
              if (e < eBest) {
                eBest = e; sBest = s;
              }
            }
            S[eBins[b]] = sBest;
          }
          // convert mask to edge maps (examining expanded set of pixels)
          float *E1 = E + c*stride*h2 + r*stride; b1 = eBnds[k*nBnds + sharpen + 1];
          for (int b = b0; b < b1; b++) {
            int i = eBins[b]; uint8 s = S[i]; uint16 *N1 = N + i * 4;
            if (s != S[N1[0]] || s != S[N1[1]] || s != S[N1[2]] || s != S[N1[3]])
              E1[eids[i]]++;
          }
        }
      }
    }
  }

  // free memory
  delete[] iids; delete[] eids;
  delete[] cids; delete[] cids1; delete[] cids2;
  delete [] ind;
}







