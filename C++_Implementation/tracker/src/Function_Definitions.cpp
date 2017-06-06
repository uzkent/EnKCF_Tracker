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
// #include <cmath>

using std::uniform_int_distribution;
using std::random_device;
using std::default_random_engine;

random_device rd;
default_random_engine generator( rd() );
uniform_int_distribution<int> randomDevice(0,1000); // Random Number Generator (0,1000)

using namespace cv;

void Particle_Filter::particle_initiation(vector<double> Obs)
{
    // Initiate Particles for the First Time Step
    vector<double> Q{10,10,5,5};
    Obs[2] = 0; Obs[3] = 0;

    for (int i = 0; i < N_Particles; i++)
        for (int j = 0; j < ss_dimension; j++){
            particles[i*ss_dimension+j] = Obs[j] + Q[j] * double(randomDevice(generator)-500)/500;
        }
}

void Particle_Filter::particle_computeSigmaPoints(){

    // Initiate the Sigma Points based on the Posterior Distribution
    double lambda = pow(0.5,2) * (Obs_Dim + 2) - Obs_Dim;
    for (int i = 0; i < N_Particles; i++)
	for (int j = 0; j < (2*Obs_Dim+1); j+=2){
	  if (j == 0){
	      	sigmaPoints[i*10] = particles[i*ss_dimension];
              	sigmaPoints[i*10+1] = particles[i*ss_dimension+1];
              	// sigmaPoints[i*10+2] = particles[i*Obs_Dim+2];
              	// sigmaPoints[i*10+3] = particles[i*Obs_Dim+3];
	      	sigmaWeights[j] = lambda / (Obs_Dim + lambda);
	 }
	 else {  
	        cout << sqrt((Obs_Dim+lambda) * abs(Covariance[i*4+(j/2-1)*Obs_Dim])) << "---" << lambda << endl;  
		sigmaPoints[i*10+j*Obs_Dim] = particles[i*ss_dimension] + sqrt((Obs_Dim+lambda) * abs(Covariance[i*4+(j/2-1)*Obs_Dim]));
	 	sigmaPoints[i*10+j*Obs_Dim+1] = particles[i*ss_dimension+1] + sqrt((Obs_Dim+lambda) * abs(Covariance[i*4+(j/2-1)*Obs_Dim+1]));
         	// sigmaPoints[i*10+j*Obs_Dim+2] = particles[i*Obs_Dim+2] + sqrt((Obs_Dim+lambda) * abs(Covariance[i*4+(j/2-1)*Obs_Dim+2]));
         	// sigmaPoints[i*10+j*Obs_Dim+3] = particles[i*Obs_Dim+3] + sqrt((Obs_Dim+lambda) * abs(Covariance[i*4+(j/2-1)*Obs_Dim+3]));
	 	sigmaWeights[j-1] = 1 / (2*(Obs_Dim + lambda));
         	sigmaWeights[j] = 1/ (2*(Obs_Dim + lambda)); 
		cout << sigmaWeights[0] << "---" << sigmaWeights[1] << endl;		

	 	sigmaPoints[i*10+(j-1)*Obs_Dim]=particles[i*ss_dimension]-sqrt((Obs_Dim+lambda)*abs(Covariance[i*4+(j/2-1)*Obs_Dim]));
         	sigmaPoints[i*10+(j-1)*Obs_Dim+1]=particles[i*ss_dimension+1]-sqrt((Obs_Dim+lambda)*abs(Covariance[i*4+(j/2-1)*Obs_Dim+1]));
         	// sigmaPoints[i*10+(j-1)*Obs_Dim+2] = particles[i*Obs_Dim+2] - sqrt((Obs_Dim+lambda) * abs(Covariance[i*4+(j/2-1)*Obs_Dim+2]));
         	// sigmaPoints[i*10+(j-1)*Obs_Dim+3] = particles[i*Obs_Dim+3] - sqrt((Obs_Dim+lambda) * abs(Covariance[i*4+(j/2-1)*Obs_Dim+3]));
	} 
      }    	
}

void Particle_Filter::particle_transitSigmaPoints()
{
  
   // Propagate Sigma Points and Get Predicted Measurements
   for (int i = 0; i < N_Particles; i++)
       for (int j = 0; j < (2*Obs_Dim+1); j++){
	   
	   sigmaPoints[i*10+j*Obs_Dim] += particles[i*ss_dimension+2] + Q[0] * double(randomDevice(generator)-500)/500;
           sigmaPoints[i*10+j*Obs_Dim+1] += particles[i*ss_dimension+3] + Q[1] * double(randomDevice(generator)-500)/500;
           // sigmaPoints[i*10+j*Obs_Dim+2] += Q[2] * double(randomDevice(generator)-500)/500;
           // sigmaPoints[i*10+j*Obs_Dim+3] += Q[3] * double(randomDevice(generator)-500)/500;
 	   
	   predMeas[i*10+j*Obs_Dim] =   sigmaPoints[i*10+j*Obs_Dim]   + Q[0] * double(randomDevice(generator)-500)/500;
           predMeas[i*10+j*Obs_Dim+1] = sigmaPoints[i*10+j*Obs_Dim+1] + Q[1] * double(randomDevice(generator)-500)/500;
           // predMeas[i*10+j*Obs_Dim+2] = sigmaPoints[i*10+j*Obs_Dim+2] + Q[2] * double(randomDevice(generator)-500)/500;
           // predMeas[i*10+j*Obs_Dim+3] = sigmaPoints[i*10+j*Obs_Dim+3] + Q[3] * double(randomDevice(generator)-500)/500;

	   // cout << sigmaPoints[i*10+j*Obs_Dim] << endl;
	}           
}

void Particle_Filter::particle_sigmaPointsMeanCovariance(){

   // Compute Mean for Each Particle	
   particles_prior = new double[N_Particles*Obs_Dim] ();
   measMean = new double[N_Particles*Obs_Dim] ();
   for (int i = 0; i < N_Particles; i++){
	for (int j = 0; j < (2*Obs_Dim+1); j++){

	    particles_prior[i*Obs_Dim ] += sigmaPoints[i*10+j*Obs_Dim] * sigmaWeights[j];
	    particles_prior[i*Obs_Dim+1] += sigmaPoints[i*10+j*Obs_Dim+1] * sigmaWeights[j];

	    measMean[i*Obs_Dim] += predMeas[i*10+j*Obs_Dim] * sigmaWeights[j];
	    measMean[i*Obs_Dim+1] += predMeas[i*10+j*Obs_Dim+1] * sigmaWeights[j];
			
	}
	particles[i*ss_dimension] = particles_prior[i*Obs_Dim];
	particles[i*ss_dimension+1] = particles_prior[i*Obs_Dim+1];
   }

   // Compute Covariance for Each Particle
   Covariance = new double[N_Particles*Obs_Dim*Obs_Dim] ();
   Covariance_Prior = new double[N_Particles*Obs_Dim*Obs_Dim] ();
   cov_measmeas = new double[N_Particles*Obs_Dim*Obs_Dim] ();
   cov_predmeas = new double[N_Particles*Obs_Dim*Obs_Dim] ();
   for (int i = 0; i < N_Particles; i++)
	for (int j = 0; j < (2*Obs_Dim+1); j++)
	    for (int m = 0; m < Obs_Dim; m++)
		for (int n = 0; n < Obs_Dim; n++){
	
 		     double Diff = sigmaPoints[i*10 + j*Obs_Dim+m] - particles[i*ss_dimension+m];
		     double Diff2 = sigmaPoints[i*10+ j*Obs_Dim+n] - particles[i*ss_dimension+n];
	             Covariance[i*4 + n*Obs_Dim + m] += sigmaWeights[j] * Diff * Diff2;
		     Covariance_Prior[i*4 + n*Obs_Dim + m] += sigmaWeights[j] * Diff * Diff2;

		     Diff2 = predMeas[i*10 + j*Obs_Dim + n] - measMean[i*Obs_Dim + n];
                     cov_predmeas[i*4 + n*Obs_Dim + m] += sigmaWeights[j] * Diff * Diff2;

		     Diff = predMeas[i*10 + j*Obs_Dim + m] - measMean[i*Obs_Dim + m];
		     Diff2 = predMeas[i*10 + j*Obs_Dim + n] - measMean[i*Obs_Dim + n];
		     cov_measmeas[i*4 + n*Obs_Dim + m] += sigmaWeights[j] * Diff * Diff2;
		
	        }
}

void Particle_Filter::particle_unscentedMeasurementUpdate(vector<double> Obs){

   // Compute UT Gains
   // particles_gain = new double[N_Particles*Obs_Dim*Obs_Dim] ();
   for (int i = 0; i < N_Particles; i++){
	double den = (cov_measmeas[i*4] * cov_measmeas[i*4+3]) -(cov_measmeas[i*4+1] * cov_measmeas[i*4+2]);
	particles_gain[i*4]   = cov_predmeas[i*4]   * (cov_measmeas[i*4]/den);
        particles_gain[i*4+1] = cov_predmeas[i*4+1] * -(cov_measmeas[i*4+1]/den);		
        particles_gain[i*4+2] = cov_predmeas[i*4+2] * -(cov_measmeas[i*4+2]/den);
        particles_gain[i*4+3] = cov_predmeas[i*4+3] * (cov_measmeas[i*4+3]/den);
	// cout << particles_gain[i*4] << "----" << cov_predmeas[i*4] << "----" << cov_measmeas[i*4]/den << endl;
   }

   // Update the Measurements of the Particles
   for (int i = 0; i < N_Particles; i++){
	particles[i*ss_dimension] += particles_gain[i*4] * (Obs[0] - particles[i*ss_dimension]) + particles_gain[i*4+2] * (Obs[1] - particles[i*ss_dimension+1]);
        particles[i*ss_dimension+1] += particles_gain[i*4+1] * (Obs[0] - particles[i*ss_dimension]) + particles_gain[i*4+3] * (Obs[1] - particles[i*ss_dimension+1]);
    }
   
   // Update the Covariance of the Particles
   for (int i = 0; i < N_Particles; i++)
	for (int m = 0; m < Obs_Dim; m++)
	    for (int n = 0; n < Obs_Dim; n++){
		Covariance[i*4+m*Obs_Dim+n] = Covariance_Prior[i*4+m*Obs_Dim+n] - (particles_gain[i*4+m*Obs_Dim+n]*cov_measmeas[i*4+m*Obs_Dim+n]*
		particles_gain[i*4+n*Obs_Dim+m]);
	    }
}

void Particle_Filter::particle_unscentedWeights(vector<double> Obs){

   // Do it on Monday
   double Diff[2];
   double Acc = 0;
   double pi = 3.14;
   for (int i = 0; i < N_Particles; i++){
	
	// Compute Proposal Weight - Compute Determinant
	double det = Covariance[i*4] * Covariance[i*4+3] -(Covariance[i*4+1] * Covariance[i*4+2]);
	double proposalWeight = 100000 * pow(2*pi,-1) * pow(abs(det),-0.5);	

	// Compute Likelihood Weight
	Diff[0] = particles[i*ss_dimension] - Obs[0];
        Diff[1] = particles[i*ss_dimension+1] - Obs[1];
        double den = 400;
	double DiffMultCov = pow(Diff[0],2) * (20/den) + Diff[0] * Diff[1] * -(0/den) + Diff[0] * Diff[1] * -(0/den) + pow(Diff[1],2) * (20/den);
	double likelihoodWeight = 100000 * pow(2*pi,-1) * pow(abs(den),-0.5) * exp(-0.5*DiffMultCov);
	// cout << Diff[0] << "---" << Diff[1] << "---" << likelihoodWeight << "---" << DiffMultCov << endl;

	// Compute Transition Likelihood
	Diff[0] = particles[i*ss_dimension] - particles_prior[i*Obs_Dim];		
        Diff[1] = particles[i*ss_dimension+1] - particles_prior[i*Obs_Dim+1]; 
	den = Covariance_Prior[i*4] * Covariance_Prior[i*4+3] - (Covariance_Prior[i*4+1] * Covariance_Prior[i*4+2]);
        DiffMultCov = pow(Diff[0],2) * (Covariance_Prior[i*4+3]/den) + Diff[0] * Diff[1] * -(Covariance_Prior[i*4+1]/den) + Diff[0] * Diff[1] * 
	-(Covariance_Prior[i*4+2]/den) + pow(Diff[1],2) * (Covariance_Prior[i*4]/den);
	double transitionWeight = 100000 * pow(2*pi,-1) * pow(abs(den),-0.5) * exp(-0.5*DiffMultCov);
        // cout << Diff[0] << "---" << Diff[1] << "---" << transitionWeight << "---" << DiffMultCov << endl;

	// Compute Particle Weight
	Weights[i] = likelihoodWeight * transitionWeight / proposalWeight;
	Acc += Weights[i];
    }
    
  // Normalize the Particle Weights
  for(int i = 0; i < N_Particles; i++)
     Weights[i] = Weights[i] / Acc;
}		


void Particle_Filter::particle_transition(double PSR)
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

void Particle_Filter::particle_weights(vector<double> Obs, double PSR)
{
    // Compute Particles' Weights
    double Dist;
    // Compute the Euclidean Distance
    double Acc = 0;
    for (int i = 0; i < N_Particles; i++) {    
		int Obs_x = Obs[0];
		int Obs_y = Obs[1];
		Dist = pow((particles[i*ss_dimension]- Obs_x),2) +
		pow(particles[i*ss_dimension+1] - Obs_y,2);

		Weights[i] = exp(-beta *Dist);
		// Weights[i] = exp(-beta * Dist);
		Acc += Weights[i];  	       // Cumulative Sum
    }
   
    // Normalize the weights
    Neff = 0;
    for (int i = 0; i < N_Particles; i++){
        Weights[i] = Weights[i] / Acc;
	Neff += pow(Weights[i],2);
    }
}

void Particle_Filter::particle_resampling(double PSR)
{

    // Resample Particles with Low Variance Sampling
    double U;
    double r = double(randomDevice(generator))/(1000*N_Particles);
    double c = Weights[0];
    int i = 0;
    if ((1/Neff) < (N_Particles/2)){
    for (int m = 0; m < N_Particles; m++){
	U = r + m * (1/double(N_Particles));  			   // Update
	
	while (U > c){		
	    i++;
	    c += Weights[i];
	}

	particles[m*ss_dimension] = particles[i*ss_dimension];         // Sample with Replacement
        particles[m*ss_dimension + 1] = particles[i*ss_dimension + 1]; // Sample with Replacement
        particles[m*ss_dimension + 2] = particles[i*ss_dimension + 2]; // Sample with Replacement
        particles[m*ss_dimension + 3] = particles[i*ss_dimension + 3]; // Sample with Replacement  	
   
    }
    }
}

void Particle_Filter::mean_estimation(vector<double> &State_Mean)
{
    // Estimate State Mean
    for (int i = 1; i < N_Particles; i++)
    {
        State_Mean[0] = State_Mean[0] + particles[i*ss_dimension]/double(N_Particles-1);
        State_Mean[1] = State_Mean[1] + particles[i*ss_dimension+1]/double(N_Particles-1);
    }
}

void Particle_Filter::particle_histograms(Mat Image, vector<double> Obs)
{
    // Compute Color Histogram for Each Particle
    vector<Mat> channels;
    split(Image,channels);	
    
    /// Establish the number of bins
    int histSize = 256;

    /// Set the ranges ( for B,G,R) )
    float range[] = { 0, 256 } ;
    const float* histRange = { range };

    bool uniform = true; bool accumulate = false;

    Mat b_hist, g_hist, r_hist;

    for (int i = 0; i < N_Particles; i++){
	Rect ROI(particles[i*Obs_Dim]-Obs[2]/2,particles[i*Obs_Dim+1]-Obs[3]/2,Obs[2],Obs[3]);
	Mat b_hist,g_hist,r_hist;
	Mat cropped_image_b(channels[0],ROI);
        Mat cropped_image_g(channels[1],ROI);
        Mat cropped_image_r(channels[2],ROI);
        calcHist( &cropped_image_b, 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
	calcHist( &cropped_image_g, 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
        calcHist( &cropped_image_r, 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );
    }
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

