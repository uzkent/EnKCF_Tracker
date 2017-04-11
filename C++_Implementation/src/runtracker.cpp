#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include "kcftracker.hpp"
#include "Filter_Definition.h"
#include "findHomography.hpp"
#include <vector>
#include <math.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/videoio/videoio.hpp"
// #include "opencv2/saliency.hpp"
#include <unistd.h>
#include <stack>
#include <random>

std::stack<clock_t> tictoc_stack;
///
/// tic and toc is used to measure the time to process an operation
///
void tic() {
    tictoc_stack.push(clock());
}

void toc() {
    std::cout << "Time elapsed: "
              << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
              << std::endl;
    tictoc_stack.pop();
}

void help()
{
    cout
    << "\n----------------------------------------------------------------------------\n"
    << "USAGE:\n"
    << "./KCF -d <MOV image filename> |-g <ground truth filename> | -o <output file> | -s <save Video file> | -e <save estimation file> \n"
    << "\nEXAMPLE:\n"                                                                     
    << "./KCF -d DJI_1.MOV -g Ground_Truth1.txt -o output1.txt -e estimation.txt -s DJI_1_Output.wmv\n"
    << "----------------------------------------------------------------------------\n"
    << endl;
}


// Define the Bounding Box Parameters
float xMin, yMin, width, height;
float xMinScale,yMinScale;
cv::Point pos;
int var1=1;
///
/// An Interactive ROI Drawal Tool
///
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
/* Need to be able to control both up and down event for the ROI */
    switch( event )
    {
    case EVENT_MOUSEMOVE:
        break;
    case EVENT_LBUTTONDOWN:
        xMin = x;
        yMin = y;
        break;
    case EVENT_LBUTTONUP:
        pos.x = x;
        pos.y = y;
        width = x - xMin;
        height =  y - yMin;
        cout << "xMin = " << xMin << " yMin = " << yMin << "  width = " << width << "  height = " << height << endl;
        break;
    }
}

int main(int argc, char* argv[])
{

   // Define Parameters for the Kernelized Correlation Filter Tracker - Options
   int opt, verbose = 0;
   bool HOG = true;   // Enable HoG Features
   bool FIXEDWINDOW = false;  // Fixed Window
   bool MULTISCALE = true;    // Scale Space Search Enabled
   bool SILENT = false;	      // Suppress the Outputs
   bool LAB = true;	      // Enable or Disable Color Features
   char szDataFile[256];
   char szImageFile[256];
   char szSaveVideofile[256];
 
   // Terminate if less than 9 inputs are provided
   if (argc < 5) {
      help();
      return -2;
   }

   while ((opt = getopt(argc,argv,"e:d:")) != EOF) {
      switch(opt)
      {
         case 'e': memset(szImageFile, '\0', sizeof(szImageFile));
            strcpy(szImageFile, optarg);
            cout <<" Input MOV Data File: "<< optarg <<endl;
            break;
         case 'd': memset(szDataFile, '\0', sizeof(szDataFile));
            strcpy(szDataFile, optarg); 
            cout <<" Input MOV Data File: "<< optarg <<endl; 
            break;
      }
   }

   ///
   /// Create KCFTracker Object
   /// \param[in] HOG : Flag to Enable or Disable Hog Features
   /// \param[in] FIXEDWINDOW : Flag to Enable or Disable Fixed Window Implementation
   /// \param[in] MULTISCALE : Flag to Include Scale Space Search
   /// \param[in] HOG : Flag to Include Color Features
   ///
   KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

   ///
   /// Create Particle Filter Tracker Object
   /// \param[in] N_Particles : Number of Particles in the Particle Filter
   /// \param[in] dimension : The State Space Dimension
   /// \param[in] beta :  Weight Function Parameter
   /// \param[in] Q : Transition Noise Variance
   /// \param[in] R : Measurement Noise Variance
   ///
   int N_Particles = 1000;     // Number of Particles
   int dimension = 4;          // State Space Dimensionality
   vector<double> Q{10,10,5,5}; // Transition Noise Variance
   double R = 5;               // Measurement Noise Variance
   double beta = 0.10;         // Likelihood Parameter
   Particle_Filter Tracker(N_Particles,dimension,beta,Q,R);

   // Frame read
   cv::Mat frame;

   // Tracker results
   cv::Rect result;

   // Read Tracker-by-Detection Output and Ground Truth
   std::vector<double> Obs {0,0,0,0};
   std::vector<double> State_Mean {0,0,0,0};

   // Video to Overlay Bounding Boxes on the Frames
   cv::VideoWriter outvid;
   int codec = cv::VideoWriter::fourcc('W', 'M','V', '2');  // select desired codec (must be available at runtime)
   std::vector<std::vector<double> > RMSE(2);

   // Generate Random Numbers used in the Particle Filter
   int seed2 = time(0);
   std::default_random_engine engine2(seed2);
   std::uniform_int_distribution<int> dist2(0,1000);

   // Read the Video
   cv::VideoCapture capt;
   if (strncmp(argv[2],"video",5) == 0){
      string path = szDataFile;
      capt.open(path);
   }
   int nFrames = 0;
   int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
   int thickness = 2;
   cv::namedWindow("Name", WINDOW_NORMAL);
   cv::resizeWindow("Name", 800,800);
   
   while (1) // Read the Next Frame
   {
     // Read the Frame to Perform Tracking
     std::stringstream ss;
     ss << std::setfill('0') << std::setw(5) << nFrames+1;
     string frame_id = to_string(nFrames); 
     if (strncmp(argv[2],"video",5) == 0){
	capt >> frame;  // Read Next Frame
        cv::resize(frame,frame,cv::Size(1280,720)); // Resize it to Make it Same with H2 Implementation     
      }
      else{
	frame = cv::imread(szDataFile+ss.str()+".jpg");
      }
      using std::default_random_engine; // Initiate the random device at each step
      using std::uniform_int_distribution;

      ///
      /// PERFORM TRACKING
      ///
      if (nFrames == 0) 
      {
         cout << "\nPlease Select ROI: \n";
	 while(var1==1)
         {
    	    // namedWindow("Name");
            setMouseCallback("Name", CallBackFunc, NULL);
            cv::imshow("Name", frame);
            // Wait until user press some key
            char c = (char)waitKey(10);
	    if( c == 'c' || c == 'C' ) {
               var1=0;
               break;
            }
         }
         // Observation on the first frame given by the user
         Obs[0] = xMin+width/2; Obs[1] = yMin+height/2; Obs[2] = width; Obs[3] = height;
         ///
	 /// Initiate the Kernelized Correlation Filter Trackers - Translation and Scale Filters
	 /// \param[in] Rect : Rectangle Object for the Bounding Box
	 /// \param[in] frame : The First Frame
	 ///
	 tracker.init( Rect(xMin, yMin, width, height), frame );
         
	 ///
	 /// Initiate the Particle of the Particle Filter
	 /// \param[in] Obs : Observation given by the user in the first frame
	 ///
         Tracker.particle_initiation(Obs);
      }
      else {
    	/// -------------------- UPDATE STEP --------------------------------------------
 
	/// Apply Camera Motion Model	
	cv::Mat homography = cameraMotionModel(frame,frame);

        /// Use Translation and Scale Filter Interchangeably
	tracker.PSR_scale = 10;
	float PSR;
        if (nFrames % 5 > 1)
        {
	    // Apply Homography to the Track Position at Previous Frame
	    tracker._roi = tracker.applyHomography(homography, tracker._roi);
            result = tracker.update(frame);        // Estimate Translation
	    PSR = tracker.PSR_sroi;
        }
        if (nFrames % 5 == 1)
        {
            // Apply Homography to the Track Position at Previous Frame
            tracker._roi_w = tracker.applyHomography(homography, tracker._roi_w);
            result = tracker.updateWROI(frame);   // Estimate Translation using Wider ROI
	    PSR = tracker.PSR_wroi;
        }
        if ( nFrames % 5 == 0)
        {
            // Apply Homography to the Track Position at Previous Frame
            tracker._roi_scale = tracker.applyHomography(homography, tracker._roi_scale);
            result = tracker.updateScale(frame);   // Estimate Scale
	    PSR = tracker.PSR_scale;
        }
       
	// TRACKING RESULTS
        cv::rectangle( frame, cv::Point(result.x,result.y), cv::Point(result.x+result.width,result.y+result.height), cv::Scalar(255,0,0),4,8);

        // Display Text on the Frame - CONFIDENCE from Translation and Scale Filter Interchangeably
        std::string confidence = to_string(int(PSR));
        cv::putText(frame,confidence, cv::Point(result.x-result.width/2,result.y-result.height/2), fontFace, 4, cv::Scalar::all(255), thickness, 4);

      }
      /// Save the Frame into the Output Video
      if (strncmp(argv[2],"video",5) == 0){
         cv::resize(frame,frame,cv::Size(800,800));
      }
      // outvid.write(frame);
      cv::imshow("Name", frame);
      nFrames++;
      if (!SILENT) {
         cv::imshow("Name", frame);
         cv::waitKey(2);
      }
   }
// Estimate Precision Curve
// precision_curve(RMSE);
}
