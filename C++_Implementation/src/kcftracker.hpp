#pragma once

#include "tracker.h"
#include "opencv2/opencv.hpp"

#ifndef _OPENCV_KCFTRACKER_HPP_
#define _OPENCV_KCFTRACKER_HPP_
#endif

class KCFTracker : public Tracker
{
public:
    // Constructor
    KCFTracker(bool hog = true, bool fixed_window = true, bool multiscale = true, bool lab = true);

    // Initialize tracker 
    virtual void init(const cv::Rect &roi, cv::Mat image);
    
    // Update position based on the new frame
    virtual cv::Rect update(cv::Mat image);

    // Update Position based on the new frame
    virtual cv::Rect updateWROI(cv::Mat image);

    // Update position based on the new frame
    virtual cv::Rect updateScale(cv::Mat image);

    // Update the Correlation Filter Result with the Particle Filter
    void updateKCFbyPF(cv::Rect _roi);

    // Apply Homography to Previous Target Position for Motion Removal
    cv::Rect_<float> applyHomography(cv::Mat homography, cv::Rect_<float> oldRoi);
    
    // Target Re-detection Module
    std::pair<int,float> target_redetection(std::vector<cv::Vec4i> BoundingBoxes, cv::Mat frame);

    float interp_factor, interp_factor_scale, interp_factor_w_roi; // linear interpolation factor for adaptation
    float sigma;        // gaussian kernel bandwidth
    float lambda;       // regularization
    int cell_size;      // HOG cell size
    int cell_sizeQ;     // cell size^2, to avoid repeated operations
    float padding, padding_scale, padding_w_roi; // extra area surrounding the target
    float output_sigma_factor, output_sigma_factor_scale, output_sigma_factor_w_roi; // bandwidth of gaussian target
    int template_size, template_size_scale, template_size_w_roi; // template size
    float scale_step;   // scale step for multi-scale estimation
    float scale_weight; // to downweight detection scores of other scales for added stability	

protected:
    // Detect object in the current frame.
    cv::Point2f detect(cv::Mat z, cv::Mat x, float &peak_value);

    // Detect object in the current frame.
    cv::Point2f detectWROI(cv::Mat z, cv::Mat x, float &peak_value);

    // Detect object in the current frame.
    cv::Point2f detectScale(cv::Mat z, cv::Mat x, float &peak_value);

    // Detect object in the current frame.
    void detectScaleRedetection(cv::Mat z, cv::Mat x, float &peak_value);

    // train tracker with a single image
    void train(cv::Mat x, float train_interp_factor);

    // train tracker with a single image
    void trainWROI(cv::Mat x, float train_interp_factor_w_roi);

    // train tracker with a single image
    void trainScale(cv::Mat x, float train_interp_factor_scale);

    // Evaluates a Gaussian kernel with bandwidth SIGMA for all relative shifts between input images X and Y, which must both be MxN. They must    also be periodic (ie., pre-processed with a cosine window).
    cv::Mat gaussianCorrelation(cv::Mat x1, cv::Mat x2);

    // Evaluates a Gaussian kernel with bandwidth SIGMA for all relative shifts between input images X and Y, which must both be MxN. They must    also be periodic (ie., pre-processed with a cosine window).
    cv::Mat gaussianCorrelationWROI(cv::Mat x1, cv::Mat x2);

    // Evaluates a Gaussian kernel with bandwidth SIGMA for all relative shifts between input images X and Y, which must both be MxN. They must    also be periodic (ie., pre-processed with a cosine window).
    cv::Mat gaussianCorrelationScale(cv::Mat x1, cv::Mat x2);

    // Create Gaussian Peak. Function called only in the first frame.
    cv::Mat createGaussianPeak(int sizey, int sizex);

    // Create Gaussian Peak. Function called only in the first frame.
    cv::Mat createGaussianPeakWROI(int sizey, int sizex);

    // Create Gaussian Peak. Function called only in the first frame.
    cv::Mat createGaussianPeakScale(int sizey, int sizex);

    // Obtain sub-window from image, with replication-padding and extract features
    cv::Mat getFeatures(const cv::Mat & image, bool inithann, float scale_adjust = 1.0f);

    // Obtain sub-window from image, with replication-padding and extract features
    cv::Mat getFeaturesWROI(const cv::Mat & image, bool inithann, float scale_adjust = 1.0f);

    // Obtain sub-window from image, with replication-padding and extract features
    cv::Mat getFeaturesScale(const cv::Mat & image, bool inithann, float scale_adjust = 1.0f);

    // Obtain sub-window from image, with replication-padding and extract features
    cv::Mat getFeaturesScaleRedetection(const cv::Mat & image, cv::Vec4i box);

    // Initialize Hanning window. Function called only in the first frame.
    void createHanningMats();

    // Initialize Hanning window. Function called only in the first frame.
    void createHanningMatsWROI();

    // Calculate sub-pixel peak for one dimension
    float subPixelPeak(float left, float center, float right);

    cv::Mat _alphaf,_alphafScale,_alphaf_w_roi;
    cv::Mat _prob, _prob_scale, _prob_w_roi;
    cv::Mat _tmpl,_tmplScale,_tmpl_w_roi;
    cv::Mat _num;
    cv::Mat _den;
    cv::Mat _labCentroids;

private:
    int size_patch[3], size_patch_scale[3], size_patch_w_roi[3];
    cv::Mat hann, hann_wroi;
    cv::Size _tmpl_sz, _tmpl_sz_scale, _tmpl_sz_w_roi;
    float _scale,_scale2,_scale_w_roi;
    int _gaussian_size;
    bool _hogfeatures;
    bool _labfeatures;
};
