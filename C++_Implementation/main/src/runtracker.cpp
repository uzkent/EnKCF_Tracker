#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include "kcftracker.hpp"
#include "Filter_Definition.h"
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
#include <random>

using namespace cv;
using namespace std;

std::stack<clock_t> tictoc_stack;

///
/// tic and toc is used to measure the time to process an operation
///
void tic() {
    tictoc_stack.push(clock());
}

// TOC
float toc() {
    float timeCost = ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
    // std::cout << "Time elapsed: "
    //          << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
    //          << std::endl;
    tictoc_stack.pop();
    return timeCost;
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
   bool HOG = true;  	      // Enable HoG Features
   bool FIXEDWINDOW = false;  // Fixed Window
   bool MULTISCALE = true;    // Scale Space Search Enabled
   bool SILENT = false;	      // Suppress the Outputs
   bool LAB = true;	      // Enable or Disable Color Features
   char szDataFile[256];
   char gtDataFile[256];
   char szImageFile[256];
   char szSaveVideofile[256];
   char prDataFile[256];   

   // Terminate if less than 9 inputs are provided
   if (argc < 9) {
      help();
      return -2;
   }

   while ((opt = getopt(argc,argv,"e:d:g:p:")) != EOF) {
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
         case 'g': memset(gtDataFile, '\0', sizeof(gtDataFile));
            strcpy(gtDataFile, optarg);
            cout <<" Input MOV Data File: "<< optarg <<endl;
            break;
        case 'p': memset(prDataFile, '\0', sizeof(prDataFile));
            strcpy(prDataFile, optarg);
            cout <<" Input MOV Data File: "<< optarg <<endl;
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
   vector<double> Q{5,5,1,1}; // Transition Noise Variance
   double R = 5;               // Measurement Noise Variance
   double beta = 0.05;         // Likelihood Parameter
   Particle_Filter PfTracker(N_Particles,dimension,beta,Q,R);

   // Frame read
   cv::Mat frame, frame_old;

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
#ifdef _AutelDATASET_
   cv::VideoCapture capt;
   std::string path = szDataFile;
   std::string vName = prDataFile;
   std::string fullPath = path.append(vName+".mp4");
   capt.open(fullPath);
#endif
   int nFrames = 0;
   int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
   int thickness = 2;
   cv::namedWindow("Name", WINDOW_NORMAL);
   cv::resizeWindow("Name", 800,800);
   
   // Read the Ground Truth Text File
char ch;
#ifdef _VOT15_DATASET_
   float g1X, g1Y, g2X, g2Y, g3X, g3Y, g4X, g4Y;
#endif

#ifdef _OTB100_DATASET_
   float gX, gY, gWidth, gHeight;
   // Read Start Frame
   std::string homeDir = "/home/buzkent/Downloads/OTB100/";
   std::string startFile = homeDir.append(prDataFile);
   std::ifstream file(startFile+"/"+"startframe.txt");
   int firstFrame;
   file >> firstFrame;
   std::cout << firstFrame << std::endl;
#endif

#ifdef _AutelDATASET_
   std::string temp;
   int temp2;
   float gX, gY, gtX, gtY, gbX, gbY, gWidth, gHeight;
#endif

#ifdef _UAV123_DATASET_
   int gX, gY, gWidth, gHeight;
   int fFrame,lFrame,frameID;
   std::string seqName; 
   std::string seqNameID;
   std::string matchString = prDataFile;
   std::ifstream startFrame("/home/buzkent/Desktop/startFrames_UAV123.txt");
   while(startFrame >> fFrame >> lFrame >> seqNameID >> seqName){
     if (seqNameID.compare(matchString) == 0){
         frameID = fFrame;
         break;
      }	
   }
#endif

   // Read the Ground Truth File for the Object of Interest
   std::ifstream groundTruth(gtDataFile);
   std::cout << gtDataFile << std::endl;

   // Declare A Vector for Euclidean Distance
   std::vector<std::vector<float>> Metrics(2);
   float runTime = 0;

#ifdef _USE_VIDEO_INPUT__
   while (1) // Read the Next Frame
   {
#endif
   int rdCallFrames = 0;
int skipOPE = 0;
#ifdef _UAV123_DATASET_
   //int skipOPE = 0;
   while(groundTruth >> gX >> ch >> gY >> ch >> gWidth >> ch >> gHeight)
   {
   if (gX < -100){
      skipOPE = 1;
   }
   else{
      skipOPE = 0;
   }
#endif

#ifdef _AutelDATASET_
    int ID;
    while(groundTruth >> ID >> gtX >> gtY >> gbX >> gbY >> temp2 >> temp2 >> temp2 >> temp2 >> temp){
	gWidth = abs(gtX - gbX);
	gHeight = abs(gtY - gbY);
	gX = gtX;
	gY = gtY;
#endif

#ifdef _VOT15_DATASET_
   while(groundTruth >> g1X >> ch >> g1Y >> ch >> g2X >> ch >> g2Y >> ch >> g3X >> ch >> g3Y >> ch >> g4X >> ch >> g4Y)
   {
      float xVertices[4] = {g1X,g2X,g3X,g4X};
      float yVertices[4] = {g1Y,g2Y,g3Y,g4Y};
      cv::Mat xCoord(2,2, CV_32F, xVertices);
      cv::Mat yCoord(2,2, CV_32F, yVertices);
      double minValue, maxValue;
      cv::Point minLoc, maxLoc;
      cv::Rect boundRect;
      cv::minMaxLoc(xCoord, &minValue, &maxValue, &minLoc, &maxLoc);
      boundRect.x = minValue; boundRect.width = maxValue - minValue;
      cv::minMaxLoc(yCoord, &minValue, &maxValue, &minLoc, &maxLoc);
      boundRect.y = minValue; boundRect.height = maxValue - minValue;
#endif

#ifdef _OTB100_DATASET_
   while(groundTruth >> gX >> gY >> gWidth >> gHeight){
#endif
      // Read the Frame to Perform Tracking
      std::stringstream ss;
#ifdef _UAV123_DATASET_     
      ss << std::setfill('0') << std::setw(6) << frameID;
      frame = cv::imread(szDataFile+ss.str()+".jpg");
#endif
#ifdef _OTB100_DATASET_     
      ss << std::setfill('0') << std::setw(4) << firstFrame;
#endif
      frame = cv::imread(szDataFile+ss.str()+".jpg");


#ifdef _VOT15_DATASET_     
      ss << std::setfill('0') << std::setw(8) << nFrames+1;
#endif
      using std::default_random_engine; // Initiate the random device at each step
      using std::uniform_int_distribution;

#ifdef _AutelDATASET_
      capt >> frame;  // Read Next Frame
      if (frame.rows < 10){
         break;
      }
#endif

      ///
      /// PERFORM TRACKING
      ///
      if (nFrames == 0) 
      {
        /// Observation on the first frame given by the user
#ifdef _UAV123_DATASET_
        Obs[0] = gX; Obs[1] = gY; Obs[2] = gWidth; Obs[3] = gHeight;
#endif

#ifdef _OTB100_DATASET_
	Obs[0] = gX; Obs[1] = gY; Obs[2] = gWidth; Obs[3] = gHeight;
#endif

#ifdef _AutelDATASET_
        Obs[0] = gX; Obs[1] = gY; Obs[2] = gWidth; Obs[3] = gHeight;
#endif

#ifdef _VOT15_DATASET_
        Obs[0] = boundRect.x; Obs[1] = boundRect.y; Obs[2] = boundRect.width; Obs[3] = boundRect.height;
#endif
        ///
        /// Initiate the Kernelized Correlation Filter Trackers - Translation and Scale Filters
        /// \param[in] Rect : Rectangle Object for the Bounding Box
        /// \param[in] frame : The First Frame
        ///
        tracker.init( Rect(Obs[0], Obs[1], Obs[2], Obs[3]), frame );
	
	///
	/// Initiate the Particles for the Particle Filter
	///
	PfTracker.particle_initiation(Obs);
	Metrics[0].push_back(0); // Precision
	Metrics[1].push_back(100); // Success
        cv::rectangle(frame,cv::Point(Obs[0],Obs[1]),cv::Point(Obs[0]+Obs[2],Obs[1]+Obs[3]),cv::Scalar(0,255,0),4,8);
      }
      else {
        /// -------------------- UPDATE STEP --------------------------------------------
	// Perform PF Transition
	State_Mean[0] = 0; State_Mean[1] = 0;
	PfTracker.particle_transition();
	PfTracker.mean_estimation(State_Mean);

	// PfTracker.Draw_Particles(frame, Scalar (0,0,255), 3);

	// Update CF new centroids
	result.x = State_Mean[0] - result.width/2.0;
	result.y = State_Mean[1] - result.height/2.0;
	tracker.updateKCFbyPF(result);
	
	tic();
	float PSR;
	if ((nFrames % 5 > 0) && (nFrames % 5 < 3)){
           result = tracker.updateWROI(frame);  // Estimate Translation using Wider ROI
           PSR = tracker.PSR_wroi;
	}
        if ((nFrames % 5 == 3) || (nFrames % 5 == 4)){
           result = tracker.update(frame);      // Estimate Translation using Smaller ROI
           PSR = tracker.PSR_sroi;
	}
	if (nFrames % 5 == 0){
           result = tracker.updateScale(frame); // Estimate Scale using Scale Filter
           PSR = tracker.PSR_scale;
	}
        float indRunTime = toc();
        runTime += indRunTime;

	// Overlay the EnKCF result
	cv::circle(frame,Point(result.x+result.width/2.0,result.y+result.height/2.0),1,cv::Scalar(0,255,0),6);

        // Update Particle Filter Weights
	Obs[0] = result.x + result.width/2.0;
	Obs[1] = result.y + result.height/2.0;
	PfTracker.particle_weights(Obs); 

	// Perform Re-Sampling
	PfTracker.particle_resampling();

	// Find Posterior Mean
	State_Mean[0] = 0; State_Mean[1] = 0;
	PfTracker.mean_estimation(State_Mean);

	// Update Final Results
	result.x = State_Mean[0] - result.width/2.0;
	result.y = State_Mean[1] - result.height/2.0;

	PfTracker.Draw_Particles(frame,Scalar (0,0,0), 3);

        cv::circle(frame,Point(result.x+result.width/2.0,result.y+result.height/2.0),1,cv::Scalar(255,25,25),6);

        // TRACKING RESULTS OVERLAID ON THE FRAME
        cv::rectangle( frame, cv::Point(result.x,result.y), cv::Point(result.x+result.width,result.y+result.height), cv::Scalar(255,0,0),4,8);
        cv::rectangle( frame, cv::Point(gX,gY), cv::Point(gX+gWidth,gY+gHeight), cv::Scalar(0,255,0),4,8);
	std::string confidence = to_string(int(PSR));
        cv::putText(frame,confidence, cv::Point(result.x-result.width/2,result.y-result.height/2), fontFace, 4, cv::Scalar::all(255), thickness, 4);

        // Compute the Euclidean Distance for Precision Metric
	if (skipOPE == 0){
#ifdef _UAV123_DATASET_
        float eucDistance = pow(pow(((float) gX + (float) gWidth/2.0) - ((float) result.x + (float) result.width/2.0),2) + pow(((float) gY + (float) gHeight/2.0) - ((float) result.y + (float) result.height/2.0),2),0.5);
#endif

#ifdef _OTB100_DATASET_
        float eucDistance = pow(pow(((float) gX + (float) gWidth/2.0) - ((float) result.x + (float) result.width/2.0),2) + pow(((float) gY + (float) gHeight/2.0) - ((float) result.y + (float) result.height/2.0),2),0.5);
#endif

#ifdef _AutelDATASET_
        float eucDistance = pow(pow(((float) gX + (float) gWidth/2.0) - ((float) result.x + (float) result.width/2.0),2) + pow(((float) gY + (float) gHeight/2.0) - ((float) result.y + (float) result.height/2.0),2),0.5);
#endif

#ifdef _VOT15_DATASET_
        float eucDistance = pow(pow((boundRect.x + boundRect.width/2.0) - (result.x + result.width/2),2) + pow((boundRect.y+boundRect.height/2.0) - (result.y + result.height/2.0),2),0.5);
#endif

        // Compute the Success Overlap by Intersection / Union
#ifdef _UAV123_DATASET_
        cv:Rect gtRect(gX,gY,gWidth,gHeight);
#endif

#ifdef _AutelDATASET_
        cv:Rect gtRect(gX,gY,gWidth,gHeight);
#endif

#ifdef _OTB100_DATASET_
        cv:Rect gtRect(gX,gY,gWidth,gHeight);
#endif

#ifdef _VOT15_DATASET_
        cv:Rect gtRect(g1X,g1Y,g4X,g4Y);
#endif

	// Compute Success Overlap (Intersection over Union)
        cv::Rect tRect(result.x,result.y,result.width,result.height);
        cv::Rect Inters = gtRect & tRect;
        cv::Rect Un = gtRect |  tRect;
        float sucOverlap= 100.00 * (float) Inters.area() / (float) Un.area();
        Metrics[0].push_back(eucDistance); // Precision
        Metrics[1].push_back(sucOverlap);  // Overlap
      }
    }
      
      nFrames++;
#ifdef _UAV123_DATASET_      
      frameID++;
      if (frameID > lFrame){
         break;
      }
#endif

#ifdef _OTB100_DATASET_
     firstFrame++;
#endif
      if (!SILENT) {
	// cv::resize(frame,frame,Size(300,150));
        cv::imshow("Name", frame);
        cv::waitKey(2);
      }
   }
   // Estimate Precision Curve
   runTime /= nFrames;
   PrecisionCurve(Metrics, prDataFile, runTime);
}
