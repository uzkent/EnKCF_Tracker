#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include "kcftracker.hpp"
#include "Filter_Definition.h"
#include <dirent.h>
#include <ctime>
#include <vector>
#include <math.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/videoio/videoio.hpp"
#include <unistd.h>
#include <stack>
#include <ctime>
#include <random>

using namespace std;
using namespace cv;
using std::default_random_engine;
using std::uniform_int_distribution;

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
Point pos;
int var1=1;
///
/// An Interactive ROI Drawal Tool
///
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
/* Need to be able to control both up and down event for the ROI */
    cout << "In CallBack Function" << endl;
    switch( event )
    {
    case EVENT_MOUSEMOVE:
	cout << "MOUSEMOVE" << endl;
        break;
    case EVENT_LBUTTONDOWN:
        xMin = x;
        yMin = y;
	cout << "BOTTOM DOWN" << endl;
        break;
    case EVENT_LBUTTONUP:
        pos.x = x;
        pos.y = y;
        width = x - xMin;
        height =  y - yMin;
	cout << "BOTTOM UP" << endl;
        cout << "xMin = " << xMin << " yMin = " << yMin << "  width = " << width << "  height = " << height << endl;
//        var1=0;
        break;
    }
/*   
   if  ( event == EVENT_LBUTTONDOWN )
   {
      pos.x=x;
      pos.y=y;
      var1=0;
   }
*/
}

int main(int argc, char* argv[])
{

   // Define Parameters for the Kernelized Correlation Filter Tracker - Options
   int opt, verbose = 0;
   bool HOG = true;   // Enable HoG Features
   bool FIXEDWINDOW = false;  // Fixed Window
   bool MULTISCALE = true;    // Scale Space Search Enabled
   bool SILENT = false;	      // Suppress the Outputs
   bool LAB = false;	      // Enable or Disable Color Features
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
   Mat frame;

   // Tracker results
   Rect result;

   // Read Tracker-by-Detection Output and Ground Truth
   vector<double> Obs {0,0,0,0};
   vector<double> State_Mean {0,0,0,0};

   // Video to Overlay Bounding Boxes on the Frames
   VideoWriter outvid;
   int codec = VideoWriter::fourcc('W', 'M','V', '2');  // select desired codec (must be available at runtime)
   // outvid.open(szSaveVideofile,codec,25,Size(800,800),1);
   vector<vector<double> > RMSE(2);

   // Generate Random Numbers used in the Particle Filter
   int seed2 = time(0);
   default_random_engine engine2(seed2);
   uniform_int_distribution<int> dist2(0,1000);

   // Read the Video
   cout << argv[2] << endl;
   cout << szDataFile << endl;
   VideoCapture capt;
   if (strncmp(argv[2],"video",5) == 0){
      cout << "INNNN" << endl;
      string path = szDataFile;
      capt.open(path);
      if (capt.isOpened())
      {
        cout << "Successful" << endl;
      }
   }
   int nFrames = 0;
   namedWindow("Name", WINDOW_NORMAL);
   resizeWindow("Name", 800,800);
   
   while (1) // Read the Next Frame
   {
     std::stringstream ss;
     ss << std::setfill('0') << std::setw(8) << nFrames+1;
     string frame_id = to_string(nFrames); 
     if (strncmp(argv[2],"video",5) == 0){
        cout << "Image Read" << endl;
	capt >> frame;  // Read Next Frame
        cv::resize(frame,frame,Size(1280,720)); // Resize it to Make it Same with H2 Implementation     
      }
      else{
	cout << strncmp(argv[2],"video",5) << endl;
        cout << szDataFile+ss.str()+".jpg" << endl;
	frame = imread(szDataFile+ss.str()+".jpg",1);
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
    	    cout << "Before Callback" << endl;
	    // namedWindow("Name");
            setMouseCallback("Name", CallBackFunc, NULL);
      	    cout << "After CallBack" << endl;
            imshow("Name", frame);
            // Wait until user press some key
            rectangle( frame, Point( xMin, yMin ), Point( xMin+width, yMin+height), Scalar( 255, 0, 0 ), 4, 8 );
            char c = (char)waitKey(10);
	    cout << c << endl;
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
         rectangle( frame, Point( xMin, yMin ), Point( xMin+width, yMin+height), Scalar( 0, 255, 255 ), 15, 8 );
      }
      else {

    	 /// -------------------- UPDATE STEP --------------------------------------------
         // Use Translation and Scale Filter Interchangeably
        if (nFrames % 5 > 1)
        {
            tic();
            result = tracker.update(frame);        // Estimate Translation
            toc();
        }
        if (nFrames % 5 == 1)
        {
            tic();
            result = tracker.updateWROI(frame);   // Estimate Translation using Wider ROI
            toc();
         }
        if ( nFrames % 5 == 0)
        {
            tic();
            result = tracker.updateScale(frame);   // Estimate Scale
            toc();
         }

        /// -------------------------------------------------------------------------------

	     /*
     	 // Observation from the Tracking-by-Detection Output
             Obs[0] = result.x + result.width/2; Obs[1] = result.y + result.height/2;
             Obs[2] = result.width; Obs[3] = result.height;

             Tracker.particle_transition(); // Transit the Particles to the Current Frame

             Tracker.particle_weights(Obs); // Assign Particle Weights

             Tracker.particle_resampling();  // Resample the Particles

             State_Mean[0] = 0; State_Mean[1] = 0; // Initiate the State Mean
             Tracker.mean_estimation(State_Mean); // Estimate the State Mean

             // Update the Tracking-by-Detection Result by the PF results
             result.x = State_Mean[0] - result.width/2;
             result.y = State_Mean[1] - result.height/2;
             tracker.updateKCFbyPF(result);
	     */
         // Draw Points on to the Rectangle
         rectangle( frame, Point(result.x,result.y), Point(result.x+result.width,result.y+result.height), Scalar(255,0,0),4,8);

         // Draw ROI on the Frame - Both Translation and Scale ROI
         cv::Rect _roi = tracker.extracted_roi;
         rectangle(frame,Point(_roi.x,_roi.y),Point(_roi.x+_roi.width,_roi.y+_roi.height),Scalar(0,0,25),4,8);
         cv::Rect _roi_scale = tracker.extracted_roi_scale;
         // rectangle(frame,Point(_roi_scale.x,_roi_scale.y),Point(_roi_scale.x+_roi_scale.width,_roi_scale.y+_roi_scale.height),Scalar(255,0,25),4,8);

         // Display Text on the Frame - CONFIDENCE from Translation and Scale Filter Interchangeably
         string confidence = to_string(int(tracker.PSR));
         int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
         int thickness = 2;
         putText(frame,confidence, Point(result.x-result.width/2,result.y-result.height/2), fontFace, 4, Scalar::all(255), thickness, 4);

      }
      /// Save the Frame into the Output Video
      if (strncmp(argv[2],"video",5) == 0){
         resize(frame,frame,Size(800,800));
      }
      // outvid.write(frame);
      imshow("Name", frame);
      nFrames++;
      if (!SILENT) {
         imshow("Name", frame);
         waitKey(2);
      }
   }
// Estimate Precision Curve
// precision_curve(RMSE);
}
