/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Filter_Definition.h
 * Author: buzkent
 *
 * Created on September 9, 2016, 12:09 PM
 */

#ifndef FILTER_DEFINITION_H
#define FILTER_DEFINITION_H
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void precision_curve(vector<vector<double> > RMSE);

class Particle_Filter{

public:
    int N_Particles;  // Number of Particles Allocated
    int ss_dimension,Obs_Dim; // Dimensionality
    double beta,Neff;     
    vector<double> Q; // Transition Noise Covariance
    double R; // Measurement Noise Covariance
    double* Weights; 
    double* particles; //Dynamic Array to Store Particles
    int rsFlag;

    Particle_Filter(int nParticles,int Dim,double bt,vector<double> Q_transition,double R_measurement)
    {
	N_Particles = nParticles;
	ss_dimension = Dim;
	beta = bt; Neff = 0;
	Q = Q_transition;
	R = R_measurement;
	Obs_Dim = 2;
	particles = new double[N_Particles*Dim] (); // Initiate PF State
	Weights = new double[N_Particles] ();  // Initiate PF Weights
        rsFlag = 0;
    };
    
    ~Particle_Filter(){
       delete[] Weights;
       delete[] particles;	
    };

    void particle_initiation(vector<double> Obs);
    
    void particle_transition();

    void particle_weights(vector<double> Obs);

    void particle_resampling();
    
    void mean_estimation(vector<double> &State_Mean);
	
    Mat Draw_Particles(Mat frame,Scalar color,int thickness);
  
};

#endif /* FILTER_DEFINITION_H */
