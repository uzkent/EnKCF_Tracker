#pragma once

#include <opencv2/opencv.hpp>
#include <string>
//abstract base class 
class Tracker
{
public:
   Tracker()  {}
   virtual  ~Tracker() { }

    virtual void init(const cv::Rect &roi, cv::Mat image) = 0;
    virtual cv::Rect  update( cv::Mat image) = 0;
    virtual cv::Rect  updateWROI( cv::Mat image) = 0;
    virtual cv::Rect  updateScale( cv::Mat image) = 0;
    double PSR_sroi,PSR_wroi,PSR_scale;
    cv::Rect_<float> _roi, _roi_w,_roi_scale;
    cv::Rect extracted_roi, extracted_w_roi, extracted_roi_scale;
    double Train_Indicator;	

// protected:
    //cv::Rect_<float> _roi;
};

