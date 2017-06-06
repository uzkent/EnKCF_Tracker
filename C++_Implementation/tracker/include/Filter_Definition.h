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
    double* sigmaPoints; // Dynamic Array to Store Sigma Points	
    double* cov_measmeas;
    double* Covariance;
    double* Covariance_Prior;
    double* particles_prior;
    double* cov_predmeas;
    double* sigmaWeights;
    double* measMean;
    double* particles_gain;
    double* predMeas;


    Particle_Filter(int nParticles,int Dim,double bt,vector<double> Q_transition,double R_measurement)
    {
		N_Particles = nParticles;
		ss_dimension = Dim;
		beta = bt; Neff = 0;
		Q = Q_transition;
		R = R_measurement;
		Obs_Dim = 2;
		particles = new double[N_Particles*Dim] (); // Initiate PF State
		particles_prior = new double[N_Particles*Obs_Dim] (); // Initiate PF Prior State
		Weights = new double[N_Particles] ();  // Initiate PF Weights
		sigmaPoints = new double[N_Particles*Obs_Dim*(2*Obs_Dim+1)] (); // Initiate UT Sigma Points
		cov_measmeas = new double[N_Particles*Obs_Dim*Obs_Dim] (); // P(yt,yt)
		cov_predmeas = new double[N_Particles*Obs_Dim*Obs_Dim] (); // P(xt,yt)
		particles_gain = new double[N_Particles*Obs_Dim*Obs_Dim] (); // K(t)
		Covariance = new double[N_Particles*Obs_Dim*Obs_Dim];      // P(t)
		Covariance_Prior = new double[N_Particles*Obs_Dim*Obs_Dim] (); // P(t) - Prior
		sigmaWeights = new double[2*Obs_Dim+1];    	 // UT Weights
		predMeas = new double[N_Particles*Obs_Dim*(2*Obs_Dim+1)] (); // Y(t) for UT
		measMean = new double[N_Particles*Obs_Dim] ();   // y(t) for UT
        
	// Initiate the Covariance Matrix for UT
	for (int i = 0; i < N_Particles; i++){
	     Covariance[i*Dim] = 0.5;
	     Covariance[i*Dim+1] = 0.1;
	     Covariance[i*Dim+2] = 0.5;
	     Covariance[i*Dim+3] = 0.1;
	}
    };
    
    ~Particle_Filter(){
       delete[] Weights;
       delete[] particles;	
    };
 
    void particle_initiation(vector<double> Obs);
    
    void particle_computeSigmaPoints();
    
    void particle_transitSigmaPoints(); 
 
    void particle_sigmaPointsMeanCovariance();

    void particle_unscentedMeasurementUpdate(vector<double> Obs);

    void particle_unscentedWeights(vector<double> Obs);

    void particle_transition(double psr);
    
    void particle_histograms(Mat image, vector<double> Obs);

    void particle_weights(vector<double> Obs,double psr);

    void particle_resampling(double psr);
    
    void mean_estimation(vector<double> &State_Mean);
	
    Mat Draw_Particles(Mat frame,Scalar color,int thickness);
  
};

#endif /* FILTER_DEFINITION_H */
