#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../include/interface.h"
#include "/image_tools/include/image_tools.h"
#include <vector>
#include "../../edge/include/edge_boxes_interface.h"

using namespace cv;
using namespace std;

// unsigned char g_memoryEdgeBox[1000*1000*10];

// AuRect<float> g_setInitialBox;
// unsigned char g_buffer[1000*1000*10];

int main(int argc, char* argv[]) {

  /// opencv read image
  char path_model[256] = "/home/buzkent/EdgeBox/model_train.txt";
  cv::Mat Img = cv::imread("/home/buzkent/Desktop/000029.jpg");
  std::vector<cv::Mat> src;
  cv::split(Img,src); 
 
  ///
  /// Fill in the Image Data Structure Parameters
  ///
  autel::computer_vision::AuMat image;
  image.cols_ = Img.cols;
  image.rows_ = Img.rows;
  image.depth_ = 0;
  image.dims_ = 2;
  image.channels_ = Img.channels();
  image.step_[1] = image.channels_;
  image.step_[0] = image.cols_ * image.step_[1];
  unsigned char* memory_image = (unsigned char*)malloc(Img.cols*Img.rows*Img.channels());////memory of image is bigger than that of buffer, because ImPad function
  image.data_ = memory_image;

  for (int i = 0; i < Img.rows ; i++){
      for(int j = 0; j < Img.cols ; j++){
         image.data_[(i*Img.cols+j)*Img.channels()] = src[0].at<uchar>(i,j);
         image.data_[(i*Img.cols+j)*Img.channels()+1] = src[1].at<uchar>(i,j);
         image.data_[(i*Img.cols+j)*Img.channels()+2] = src[2].at<uchar>(i,j);
     }
  }
  
  //initialize edge box
  std::cout << "222" << std::endl;
  InitializedBox(path_model);
  std::cout << "3333" << std::endl;
  std::vector<cv::Vec4i> BoundingBoxes;
  BoundingBoxes = EdgeBoxInterface(image);
  std::cout << BoundingBoxes.size() << std::endl;
  
  // Display Them
  // rectangle
  for(int i = 0; i < 100; i++){
      cv::rectangle(Img,cv::Point(BoundingBoxes[i][1],BoundingBoxes[i][0]),cv::Point(BoundingBoxes[i][1]+BoundingBoxes[i][3],
      BoundingBoxes[i][0]+BoundingBoxes[i][2]),cv::Scalar(0,255,0),2,2);
  } 

  cv::imshow("EdgeBox Result",Img);
  cv::waitKey(0);  

  return 0;
}
