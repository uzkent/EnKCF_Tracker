/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
*/  
#include <iostream>
#include "Filter_Definition.h"
#include <vector>
#include <math.h>
#include <numeric>
#include <random>
#include <opencv2/opencv.hpp>

using std::uniform_int_distribution;
using std::random_device;
using std::default_random_engine;

random_device rd;
default_random_engine generator(rd());
uniform_int_distribution<int> randomDevice(0,1000); // Random Number Generator (0,1000)

using namespace cv;

void Particle_Filter::particle_initiation(vector<double> Obs)
{
    // Initiate Particles for the First Time Step
    vector<double> Q{25,25,10,10};
    Obs[0] += Obs[2]/2.0; Obs[1] += Obs[3]/2.0;
    Obs[2] = 0; Obs[3] = 0;
    for (int i = 0; i < N_Particles; i++){
        Weights[i] = 1.0/ N_Particles;
	for (int j = 0; j < ss_dimension; j++){
            particles[i*ss_dimension+j] = Obs[j] + Q[j] * double(randomDevice(generator)-500)/500;
        }
    }
}

void Particle_Filter::particle_transition()
{
    // Propagate the Particles
    for (int i = 0; i < N_Particles; i++)
    {      
	particles[i*ss_dimension] += particles[i*ss_dimension+2] + Q[0] * double(randomDevice(generator)-500)/500;
	particles[i*ss_dimension+1] += particles[i*ss_dimension+3] + Q[1] * double(randomDevice(generator)-500)/500;
	particles[i*ss_dimension+2] += Q[2] * double(randomDevice(generator)-500)/500; // X Velocity
	particles[i*ss_dimension+3] += Q[3] * double(randomDevice(generator)-500)/500; // X Velocity
    }
}

void Particle_Filter::particle_weights(vector<double> Obs)
{
    // Compute Particles' Weights
    double Dist;
    // Compute the Euclidean Distance
    double Acc = 0;
    for (int i = 0; i < N_Particles; i++) {    
	int Obs_x = Obs[0];
	int Obs_y = Obs[1];
	// Find Spatial Distance
	Dist = pow((particles[i*ss_dimension]- Obs_x),2) +
	pow(particles[i*ss_dimension+1] - Obs_y,2);

	Weights[i] = exp(-beta *Dist); // Compute Weight
	Acc += Weights[i];  	       // Cumulative Sum
    }
   
    // Normalize the weights
    Neff = 0;
    for (int i = 0; i < N_Particles; i++){
        Weights[i] = Weights[i] / Acc;
	Neff += pow(Weights[i],2); // Efficient # of Samples
    }
}

void Particle_Filter::particle_weights_cfMap(cv::Mat response, cv::Rect_<int> ROI)
{
    // Compute Particles' Weights
    double Acc = 0;
    for (int i = 0; i < N_Particles; i++) {
	if ((particles[i*ss_dimension] < ROI.x) || (particles[i*ss_dimension+1] < ROI.y) || (particles[i*ss_dimension] > (ROI.x+ROI.width)) || (particles[i*ss_dimension+1] > (ROI.y+ROI.height))){
	   Weights[i] = 0;
	}
	else{
	   int xCoord = abs(particles[i*ss_dimension] - ROI.x);
           int yCoord = abs(particles[i*ss_dimension+1] - ROI.y);
           Weights[i] = response.at<double>(yCoord,xCoord);
	   cv::Point2i pi;
	   double pv;
           cv::minMaxLoc(response, NULL, &pv, NULL, &pi);
	   cout << pi.x << "---" << pi.y << "---" << response.rows << "---" << response.cols << endl;
        }
        Acc += Weights[i];             // Cumulative Sum
    }

    // Normalize the weights
    Neff = 0;
    for (int i = 0; i < N_Particles; i++){
        Weights[i] = Weights[i] / Acc;
        Neff += pow(Weights[i],2); // Efficient # of Samples
    }
}

void Particle_Filter::particle_resampling()
{
    // Resample Particles with Low Variance Sampling
    double U;
    double r = double(randomDevice(generator))/(1000*N_Particles);
    double c = Weights[0];
    int i = 0;
    if ((1/Neff) < (N_Particles / 0.50)){
       for (int m = 0; m < N_Particles; m++){
	   U = r + m * (1/double(N_Particles));	// Update
	
	   while (U > c){		
	       i++;
	       c += Weights[i];
	   }

	   particles[m*ss_dimension] = particles[i*ss_dimension];         // Sample with Replacement
           particles[m*ss_dimension + 1] = particles[i*ss_dimension + 1]; // Sample with Replacement
           particles[m*ss_dimension + 2] = particles[i*ss_dimension + 2]; // Sample with Replacement
           particles[m*ss_dimension + 3] = particles[i*ss_dimension + 3]; // Sample with Replacement  	
       }
       rsFlag = 1;
    }
}

void Particle_Filter::mean_estimation(vector<double>& State_Mean)
{
    // Estimate State Mean
    for (int i = 0; i < N_Particles; i++)
    {
	if (rsFlag == 0){
           State_Mean[0] += Weights[i] * particles[i*ss_dimension];
           State_Mean[1] += Weights[i] * particles[i*ss_dimension+1];
    	}
	else{
   	   State_Mean[0] += (1.0/N_Particles) * particles[i*ss_dimension];
           State_Mean[1] += (1.0/N_Particles) * particles[i*ss_dimension+1];
	}
    }
    rsFlag = 0;
}

void precision_curve(vector<vector<double> > RMSE)
{
    // Compute Precision Curve for TBM and TBD
    vector<vector<double> > prScore(2);
    for (int i = 1; i < 101; i++)
        {
        vector<int> precision{0,0};
        for(int j = 0; j < RMSE[0].size(); j++)
            {
            if (RMSE[0][j] < i)
                precision[0] += 1; // Successfull tracking
            if (RMSE[1][j] < i)
                precision[1] += 1; // Successfull tracking
            }
        prScore[0].push_back(double(precision[0])/RMSE[0].size()); // Precision Score for the Video - TBM
        prScore[1].push_back(double(precision[1])/RMSE[1].size()); // Precision Score for the Video - TBD
        cout << prScore[0][i-1] << "---------------" << prScore[1][i-1] << "-------------" << i << endl;
        }
}

Mat Particle_Filter::Draw_Particles(Mat frame,Scalar color, int thickness)
{

    for (int i = 0; i < N_Particles; i++){
	    circle(frame, Point(particles[i*ss_dimension],particles[i*ss_dimension+1]),1,color,thickness);
    }
    return frame;
}
