#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace cv;
using namespace cv::xfeatures2d;

cv::Mat cameraMotionModel(cv::Mat frame_r, cv::Mat frame_t){

  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;

  // Instantiate Surf Object
  Ptr<FeatureDetector> detector = FastFeatureDetector::create(minHessian);

  // Define Keypoint Vector
  std::vector<KeyPoint> keypoints_reference, keypoints_test;

  // Detect Keypoints for the reference
  detector->detect( frame_r, keypoints_reference );

  // Detect Keypoints for the model
  detector->detect( frame_t, keypoints_test );

  //-- Step 2: Calculate descriptors (feature vectors)
  Ptr<SURF> extractor = SURF::create();
  cv::Mat descriptors_reference, descriptors_test;
  
  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector< DMatch > good_matches;
  extractor->compute( frame_r, keypoints_reference, descriptors_reference);
  extractor->compute( frame_t, keypoints_test, descriptors_test);

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector<DMatch> matches;
  matcher.match( descriptors_reference, descriptors_test, matches);

  double max_dist = 0; double min_dist = 100;
  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_reference.rows; i++ )
  { double dist = matches[i].distance;
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

  for( int i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    reference.push_back( keypoints_reference[ good_matches[i].queryIdx ].pt );
    test.push_back( keypoints_test[ good_matches[i].trainIdx ].pt );
  }

  // Compute Homography Between Reference and Test Frame
  Mat H = findHomography( reference, test, CV_RANSAC );

  return H;

}

