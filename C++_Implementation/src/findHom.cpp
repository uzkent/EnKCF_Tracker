#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace cv::xfeatures2d;

cv::Mat cameraMotionModel(cv::Mat frame_r, cv::Mat frame_t){

  // Convert to GrayScale Imagery
  cv::cvtColor(frame_r,frame_r, CV_RGB2GRAY);
  cv::cvtColor(frame_t,frame_t, CV_RGB2GRAY);

  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 800;

  // Instantiate SURF Object
  Ptr<Feature2D> features = SURF::create(minHessian);

  //-- Step 2: Calculate descriptors (feature vectors)
  std::vector<KeyPoint> keypoints_reference, keypoints_test;
  cv::Mat descriptors_reference, descriptors_test;
  features->detect(frame_r, keypoints_reference);
  features->detect(frame_t, keypoints_test);
  
  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector< DMatch > good_matches;
  features->compute( frame_r, keypoints_reference, descriptors_reference);
  features->compute( frame_t, keypoints_test, descriptors_test);

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector<DMatch> matches;
  matcher.match( descriptors_reference, descriptors_test, matches);

  double max_dist = 0; double min_dist = 100;
  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_reference.rows; i++ )
  { 
    double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  // Find the Good Matches
  for( int i = 0; i < descriptors_reference.rows; i++ )
  { 
     if(matches[i].distance<3*min_dist)
     { 
       good_matches.push_back( matches[i]); 
     }
  }

  //-- Localize the object
  std::vector<cv::Point2f> reference;
  std::vector<cv::Point2f> test;
  
  for(size_t i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    reference.push_back( keypoints_reference[ good_matches[i].queryIdx ].pt );
    test.push_back( keypoints_test[ good_matches[i].trainIdx ].pt );
  }
 
  Mat H;
  if (reference.size() < 1){
     H = Mat::zeros(3,3,CV_32FC1);
     H.at<float>(0,0) = 1; H.at<float>(1,1) = 1; H.at<float>(2,2) = 1;
  }  
  else{
     // Compute Homography Between Reference and Test Frame
     H = findHomography( reference, test, CV_RANSAC );
  }

  return H;
}
