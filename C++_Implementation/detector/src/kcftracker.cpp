/*

Constructor parameters, all boolean:
    hog: use HOG features (default), otherwise use raw pixels
    fixed_window: fix window size (default), otherwise use ROI size (slower but more accurate)
    multiscale: use multi-scale tracking (default; cannot be used with fixed_window = true)

Default values are set for all properties of the tracker depending on the above choices.
Their values can be customized further before calling init():
    interp_factor: linear interpolation factor for adaptation
    sigma: gaussian kernel bandwidth
    lambda: regularization
    cell_size: HOG cell size
    padding: area surrounding the target, relative to its size
    output_sigma_factor: bandwidth of gaussian target
    template_size: template size in pixels, 0 to use ROI size
    scale_step: scale step for multi-scale estimation, 1 to disable it
    scale_weight: to downweight detection scores of other scales for added stability

For speed, the value (template_size/cell_size) should be a power of 2 or a product of small prime numbers.

Inputs to init():
   image is the initial frame.
   roi is a cv::Rect with the target positions in the initial frame

Inputs to update():
   image is the current frame.

Outputs of update():
   cv::Rect with target positions for the current frame
*/

#ifndef _KCFTRACKER_HEADERS
#include "kcftracker.hpp"
#include "ffttools.hpp"
#include "recttools.hpp"
#include "fhog.hpp"
#include "labdata.hpp"
#include <unistd.h>
#include <iterator>
#include <fstream>
#endif

using namespace cv;
// Constructor
KCFTracker::KCFTracker(bool hog, bool fixed_window, bool multiscale, bool lab)
{

    // Parameters equal in all cases
    lambda = 0.0001;	    			// Regularization Parameter for the Regression Task
    padding = 2.50;     			// The ROI parameter to get n times larger area than the Selected Target - Translation Filter
    padding_w_roi = 3.00; 			// The Wider ROI Parameter for the Second Translation Filter
    padding_scale = 1.00;   			// The ROI Parameter for the scale filter
    output_sigma_factor = 0.125 * 1.00;    	// Variance Parameter for the Desired Translation Filter Response
    output_sigma_factor_w_roi = 0.125 * 1.25; 	// Variance Parameter for the Wide ROI translation filter Response
    output_sigma_factor_scale = 0.125 * 0.75; 	// Variance Parameter for the Desired Scale Filter Response

    if (hog) {    		// HOG to be Used?
        interp_factor = 0.020; 	// Learning Rate for the Translation Filter
        interp_factor_w_roi = 0.020; // Learning Rate for the Wide ROI Translation Filter
        interp_factor_scale = 0.010; // Learning Rate for the Scale Filter
        sigma = 0.60;           // Gaussian Bandwith Parameter for the Gaussian Kernel in KCF
        cell_size = 4;          // Cell Size for the HoG Feature Channels
        _hogfeatures = true;

        if (lab) {  // Activate the LAB Color Features
            sigma_scale = 0.9;
            sigma_w_roi = 0.7;
            output_sigma_factor_w_roi = 0.06;
            output_sigma_factor_scale = 0.04;
            _labfeatures = true;
            _labCentroids = cv::Mat(nClusters, 3, CV_32FC1, &data);
            cell_sizeQ = cell_size*cell_size;
        }
        else{
            _labfeatures = false;
        }
    }
    else {   // RAW - Grayscale Intensity Features
        interp_factor = 0.075;
        sigma = 0.2; 
        cell_size = 1;
        _hogfeatures = false;

        if (lab) {
            printf("Lab features are only used with HOG features.\n");
            _labfeatures = false;
        }
    }


    if (multiscale) { // Multiscale KCF Implementation
        template_size = 96;       // Template Size for the Translation Filter
        template_size_w_roi = 96; // Template Size for the Wide ROI Translation Filter
        template_size_scale = 64; // Template Size for the Scale Filter
        scale_step = 1.05;     	  // Scale Factor for the Multiscale Search
        scale_weight = 2.0 - scale_step;    // Priority is given to the Same Scale with Previous Frame Scale
        if (!fixed_window) {     // Fixed Window KCF Implementation during Tracking
            fixed_window = true;
        }
    }
    else if (fixed_window) {  // fit correction without multiscale
        template_size = 96; //template_size = 100;
        scale_step = 1;
    }
    else {
        template_size = 1;
        scale_step = 1;
    }
}

// Initialize tracker 
void KCFTracker::init(const cv::Rect &roi, cv::Mat image)
{
    _roi = roi;
    assert(roi.width >= 0 && roi.height >= 0);
    _tmpl = getFeatures(image, 1);     				// Compute Feature Channels for the _roi
    _prob = createGaussianPeak(size_patch[0], size_patch[1]); 	//  Desired Correlation Output
    _alphaf = cv::Mat(size_patch[0], size_patch[1], CV_32FC2, float(0)); // Initiate the Translation Filter Model

    _roi_w = roi;
    _tmpl_w_roi = getFeaturesWROI(image, 1);     	 	// Compute Feature Channels for the _roi
    _prob_w_roi = createGaussianPeakWROI(size_patch_w_roi[0], size_patch_w_roi[1]); //  Desired Correlation Output
    _alphaf_w_roi = cv::Mat(size_patch_w_roi[0], size_patch_w_roi[1], CV_32FC2, float(0)); // Initiate the Translation Filter Model

    _roi_scale = roi;
    _tmplScale = getFeaturesScale(image, 1); 			// Compute Feature Channels for the Scale Filter
    _prob_scale = createGaussianPeakScale(size_patch_scale[0], size_patch_scale[1]);    // Desired Response for the Scale Filter
    _alphafScale = cv::Mat(size_patch_scale[0], size_patch_scale[1], CV_32FC2, float(0));   // Initiate the Scale FIlter Model

    _tmplScaleRD = getFeaturesScale(image, 1); 			// Compute Feature Channels for the Scale Filter
    _alphafScaleRD = cv::Mat(size_patch_scale[0], size_patch_scale[1], CV_32FC2, float(0));   // Initiate the Scale FIlter Model

    train(_tmpl, 1.0); // train with initial frame - translation filter 
    trainWROI(_tmpl_w_roi,1.0); // train with initial frame - translation filter Wide ROI
    trainScale(_tmplScale,1.0); // train with initial frame - scale filter
}

// Update position based on the new frame
cv::Rect KCFTracker::update(cv::Mat image)
{
    if (_roi.x + _roi.width <= 0) _roi.x = -_roi.width + 1;
    if (_roi.y + _roi.height <= 0) _roi.y = -_roi.height + 1;
    if (_roi.x >= image.cols - 1) _roi.x = image.cols - 2;
    if (_roi.y >= image.rows - 1) _roi.y = image.rows - 2;

    float cx = _roi.x + _roi.width / 2.0f;
    float cy = _roi.y + _roi.height / 2.0f;

    // Detect the Target Translation
    float peak_value;
    cv::Point2f res = detect(_tmpl, getFeatures(image, 0, 1.0f), peak_value);

    // Adjust by cell size and _scale
    _roi.x = cx - _roi.width / 2.0f + ((float) res.x * cell_size * _scale);
    _roi.y = cy - _roi.height / 2.0f + ((float) res.y * cell_size * _scale);

    // Adjust by cell size and _scale
    _roi_scale.x = _roi.x;
    _roi_scale.y = _roi.y;

    // Adjust Wider Area Translation Filter
    _roi_w.x = _roi.x;
    _roi_w.y = _roi.y;

    if (_roi.x >= image.cols - 1) _roi.x = image.cols - 1;
    if (_roi.y >= image.rows - 1) _roi.y = image.rows - 1;
    if (_roi.x + _roi.width <= 0) _roi.x = -_roi.width + 2;
    if (_roi.y + _roi.height <= 0) _roi.y = -_roi.height + 2;

    assert(_roi.width >= 0 && _roi.height >= 0);

    // Train the Translation Filter Model
    cv::Mat x = getFeatures(image, 0);
    train(x, interp_factor);

    return _roi;
}

// Update position based on the new frame
cv::Rect KCFTracker::updateWROI(cv::Mat image)
{
    if (_roi_w.x + _roi_w.width <= 0) _roi_w.x = -_roi_w.width + 1;
    if (_roi_w.y + _roi_w.height <= 0) _roi_w.y = -_roi_w.height + 1;
    if (_roi_w.x >= image.cols - 1) _roi_w.x = image.cols - 2;
    if (_roi_w.y >= image.rows - 1) _roi_w.y = image.rows - 2;

    float cx = _roi_w.x + _roi_w.width / 2.0f;
    float cy = _roi_w.y + _roi_w.height / 2.0f;

    // Detect the Target Translation
    float peak_value;
    cv::Point2f res = detectWROI(_tmpl_w_roi, getFeaturesWROI(image, 0, 1.0f), peak_value);

    // Adjust by cell size and _scale
    _roi_w.x = cx - _roi_w.width / 2.0f + ((float) res.x * cell_size * _scale_w_roi);
    _roi_w.y = cy - _roi_w.height / 2.0f + ((float) res.y * cell_size * _scale_w_roi);

    // Adjust Scale Filter ROI
    _roi_scale.x = _roi_w.x;
    _roi_scale.y = _roi_w.y;

    // Adjust 1st Translation Filter ROI
    _roi.x = _roi_w.x;
    _roi.y = _roi_w.y;

    if (_roi_w.x + _roi_w.width <= 0)  _roi_w.x = -_roi_w.width + 1;
    if (_roi_w.y + _roi_w.height <= 0) _roi_w.y = -_roi_w.height + 1;
    if (_roi_w.x >= image.cols - 1) _roi_w.x = image.cols - 2;
    if (_roi_w.y >= image.rows - 1) _roi_w.y = image.rows - 2;

    assert(_roi_w.width >= 0 && _roi_w.height >= 0);

    // Train the Translation Filter Model
    cv::Mat x = getFeaturesWROI(image, 0);
    trainWROI(x, interp_factor);

    return _roi_w;
}


// Update position based on the new frame
cv::Rect KCFTracker::updateScale(cv::Mat image)
{
    ///
    /// Detect the Peak at the Same Scale and Corresponding Value
    ///
    float scale_peak_value;
    cv::Point2f res = detectScale(_tmplScale, getFeaturesScale(image, 0, 1.0f / 1.0), scale_peak_value);
    float new_peak_value;

    // Search Different Scales
   if (scale_step != 1) {

        // Test at a smaller _scale
        cv::Point2f new_res = detectScale(_tmplScale, getFeaturesScale(image, 0, 1.0f / scale_step), new_peak_value);

        // Update the Scale if the Confidence is Larger than the Maximum
        if (scale_weight * new_peak_value > scale_peak_value) {
            res = new_res;
            scale_peak_value = new_peak_value;
            _scale /= scale_step;
            _scale_w_roi /= scale_step;
            _scale2 /= scale_step;
            _roi.width /= scale_step;
            _roi.height /= scale_step;
            _roi_w.width /= scale_step;
            _roi_w.height /= scale_step;
            _roi_scale.width /= scale_step;
            _roi_scale.height /= scale_step;
        }

        // Test at a bigger _scale
        new_res = detectScale(_tmplScale, getFeaturesScale(image, 0, scale_step), new_peak_value);

        // Update the Scale if the Confidence is Larger than the Maximum
        if (scale_weight * new_peak_value > scale_peak_value) {
            res = new_res;
            scale_peak_value = new_peak_value;
            _scale *= scale_step;
            _scale_w_roi *= scale_step;
            _scale2 *= scale_step;
            _roi.width *= scale_step;
            _roi.height *= scale_step;
            _roi_w.width *= scale_step;
            _roi_w.height *= scale_step;
            _roi_scale.width *= scale_step;
            _roi_scale.height *= scale_step;
        }
    }
    // Train the Scale Filter
    cv::Mat x = getFeaturesScale(image, 0);
    if (PSR_scale > 4.00){
       trainScale(x, interp_factor_scale);
    }
    /// --------------------------------------------------------------------

    return _roi_scale;
}


// Update KCF output by the PF output
void KCFTracker::updateKCFbyPF(cv::Rect ROI){
	_roi.x = ROI.x;     // Centroid given by Particle Filter
	_roi.y = ROI.y;
	_roi_scale.x = ROI.x;   // Centroid given by Particle Filter
	_roi_scale.y = ROI.y;
}

// Detect object in the current frame.
cv::Point2f KCFTracker::detect(cv::Mat z, cv::Mat x, float &peak_value)
{
    using namespace FFTTools;

    cv::Mat k = gaussianCorrelation(x, z); // Apply Kernel Trick with the Test Template
    cv::Mat res = (real(fftd(complexMultiplication(_alphaf, fftd(k)), true))); // Response Map

    // minMaxLoc only accepts doubles for the peak, and integer points for the coordinates
    cv::Point2i pi;
    double pv;
    cv::minMaxLoc(res, NULL, &pv, NULL, &pi);
    peak_value = (float) pv;

    // Compute the Peak-to-Sidelobe Ratio
    // PSR_sroi = computePSR(res);

    // subpixel peak estimation, coordinates will be non-integer
    cv::Point2f p((float)pi.x, (float)pi.y);

    if (pi.x > 0 && pi.x < res.cols-1) {
        p.x += subPixelPeak(res.at<float>(pi.y, pi.x-1), peak_value, res.at<float>(pi.y, pi.x+1));
    }

    if (pi.y > 0 && pi.y < res.rows-1) {
        p.y += subPixelPeak(res.at<float>(pi.y-1, pi.x), peak_value, res.at<float>(pi.y+1, pi.x));
    }

    p.x -= (res.cols) / 2;
    p.y -= (res.rows) / 2;

    return p;
}

// Detect object in the current frame.
cv::Point2f KCFTracker::detectWROI(cv::Mat z, cv::Mat x, float &peak_value)
{
    using namespace FFTTools;

    cv::Mat k = gaussianCorrelationWROI(x, z); // Apply Kernel Trick with the Test Template
    cv::Mat res = (real(fftd_w_roi(complexMultiplication(_alphaf_w_roi, fftd_w_roi(k)), true))); // Response Map

    // minMaxLoc only accepts doubles for the peak, and integer points for the coordinates
    cv::Point2i pi;
    double pv;
    cv::minMaxLoc(res, NULL, &pv, NULL, &pi);
    peak_value = (float) pv;

    // Compute the Peak-to-Sidelobe Ratio
    // PSR_wroi = computePSR(res);

    // subpixel peak estimation, coordinates will be non-integer
    cv::Point2f p((float)pi.x, (float)pi.y);

    if (pi.x > 0 && pi.x < res.cols-1) {
        p.x += subPixelPeak(res.at<float>(pi.y, pi.x-1), peak_value, res.at<float>(pi.y, pi.x+1));
    }

    if (pi.y > 0 && pi.y < res.rows-1) {
        p.y += subPixelPeak(res.at<float>(pi.y-1, pi.x), peak_value, res.at<float>(pi.y+1, pi.x));
    }

    p.x -= (res.cols) / 2;
    p.y -= (res.rows) / 2;

    return p;
}

// Detect object in the current frame.
cv::Point2f KCFTracker::detectScale(cv::Mat z, cv::Mat x, float &peak_value)
{
    using namespace FFTTools;

    cv::Mat k = gaussianCorrelationScale(x, z); // Apply Kernel Trick with the Test Template
    cv::Mat res = (real(fftd_scale(complexMultiplication(_alphafScale, fftd_scale(k)), true))); // Response Map

    // minMaxLoc only accepts doubles for the peak, and integer points for the coordinates
    cv::Point2i pi;
    double pv;
    cv::minMaxLoc(res, NULL, &pv, NULL, &pi);
    peak_value = (float) pv;

    // Compute the Peak-to-Sidelobe Ratio
    PSR_scale = computePSR(res);

    // subpixel peak estimation, coordinates will be non-integer
    cv::Point2f p((float)pi.x, (float)pi.y);

    if (pi.x > 0 && pi.x < res.cols-1) {
        p.x += subPixelPeak(res.at<float>(pi.y, pi.x-1), peak_value, res.at<float>(pi.y, pi.x+1));
    }

    if (pi.y > 0 && pi.y < res.rows-1) {
        p.y += subPixelPeak(res.at<float>(pi.y-1, pi.x), peak_value, res.at<float>(pi.y+1, pi.x));
    }

    p.x -= (res.cols) / 2;
    p.y -= (res.rows) / 2;

    return p;

}


// train tracker with a single image
void KCFTracker::train(cv::Mat x, float train_interp_factor)
{
    using namespace FFTTools;

    cv::Mat k = gaussianCorrelation(x, x);  // Apply the Kernel Trick
    cv::Mat alphaf = complexDivision(_prob, (fftd(k) + lambda));    // Model for the CUrrent Frame
    
    _tmpl = (1 - train_interp_factor) * _tmpl + (train_interp_factor) * x;  // Update the Overall Template Model
    _alphaf = (1 - train_interp_factor) * _alphaf + (train_interp_factor) * alphaf; // Update the Overall Correlation Filter Model

}

// train tracker with a single image
void KCFTracker::trainWROI(cv::Mat x, float train_interp_factor_w_roi)
{
    using namespace FFTTools;

    cv::Mat k = gaussianCorrelationWROI(x, x);  				      // Apply the Kernel Trick
    cv::Mat alphaf_w_roi = complexDivision(_prob_w_roi, (fftd_w_roi(k) + lambda));    // Model for the CUrrent Frame
    
    _tmpl_w_roi = (1 - train_interp_factor_w_roi) * _tmpl_w_roi + (train_interp_factor_w_roi) * x;  // Update the Overall Template Model
    _alphaf_w_roi = (1 - train_interp_factor_w_roi) * _alphaf_w_roi + (train_interp_factor_w_roi) * alphaf_w_roi; // Update the Overall Correlation Filter Model

}

void KCFTracker::trainScale(cv::Mat x, float train_interp_factor)
{

    using namespace FFTTools;

    cv::Mat k = gaussianCorrelationScale(x, x); // Apply the Kernel Trick
    cv::Mat alphafScale = complexDivision(_prob_scale, (fftd_scale(k) + lambda));   // Model for the Current Frame
    
    _tmplScale = (1 - train_interp_factor) * _tmplScale + (train_interp_factor) * x;    // Update the Overall Template Model
    _alphafScale = (1 - train_interp_factor) * _alphafScale + (train_interp_factor) * alphafScale;  // Update the Overall Correlation Filter Model
    if (train_interp_factor == 1){
       _alphafScaleRD = _alphafScale.clone();
       _tmplScaleRD = _tmplScale.clone();
    }
}

// Evaluates a Gaussian kernel with bandwidth SIGMA for all relative shifts between input images X and Y, which must both be MxN. They must    also be periodic (ie., pre-processed with a cosine window).
cv::Mat KCFTracker::gaussianCorrelation(cv::Mat x1, cv::Mat x2)
{
    using namespace FFTTools;
    cv::Mat c = cv::Mat( cv::Size(size_patch[1], size_patch[0]), CV_32F, cv::Scalar(0) );
    // HOG features
    if (_hogfeatures) {
        cv::Mat caux;
        cv::Mat x1aux;
        cv::Mat x2aux;
        for (int i = 0; i < size_patch[2]; i++) {
            x1aux = x1.row(i);   // Procedure do deal with cv::Mat multichannel bug
            x1aux = x1aux.reshape(1, size_patch[0]);
            x2aux = x2.row(i).reshape(1, size_patch[0]);
            cv::mulSpectrums(fftd(x1aux), fftd(x2aux), caux, 0, true); 
            caux = fftd(caux, true);
            rearrange(caux);
            caux.convertTo(caux,CV_32F);
            c = c + real(caux);
        }
    }
    // Gray features
    else {
        cv::mulSpectrums(fftd(x1), fftd(x2), c, 0, true);
        c = fftd(c, true);
        rearrange(c);
        c = real(c);
    }
    cv::Mat d; 
    cv::max(( (cv::sum(x1.mul(x1))[0] + cv::sum(x2.mul(x2))[0])- 2. * c) / (size_patch[0]*size_patch[1]*size_patch[2]) , 0, d);

    cv::Mat k;
    cv::exp((-d / (sigma * sigma)), k);
    return k;
}

// Evaluates a Gaussian kernel with bandwidth SIGMA for all relative shifts between input images X and Y, which must both be MxN. They must    also be periodic (ie., pre-processed with a cosine window).
cv::Mat KCFTracker::gaussianCorrelationWROI(cv::Mat x1, cv::Mat x2)
{
    using namespace FFTTools;
    cv::Mat c = cv::Mat( cv::Size(size_patch_w_roi[1], size_patch_w_roi[0]), CV_32F, cv::Scalar(0) );
    // HOG features
    if (_hogfeatures) {
        cv::Mat caux;
        cv::Mat x1aux;
        cv::Mat x2aux;
        for (int i = 0; i < size_patch_w_roi[2]; i++) {
            x1aux = x1.row(i);   // Procedure do deal with cv::Mat multichannel bug
            x1aux = x1aux.reshape(1, size_patch_w_roi[0]);
            x2aux = x2.row(i).reshape(1, size_patch_w_roi[0]);
            cv::mulSpectrums(fftd_w_roi(x1aux), fftd_w_roi(x2aux), caux, 0, true); 
            caux = fftd_w_roi(caux, true);
            rearrange(caux);
            caux.convertTo(caux,CV_32F);
            c = c + real(caux);
        }
    }
    // Gray features
    else {
        cv::mulSpectrums(fftd_w_roi(x1), fftd_w_roi(x2), c, 0, true);
        c = fftd_w_roi(c, true);
        rearrange(c);
        c = real(c);
    }
    cv::Mat d; 
    cv::max(( (cv::sum(x1.mul(x1))[0] + cv::sum(x2.mul(x2))[0])- 2. * c) / (size_patch_w_roi[0]*size_patch_w_roi[1]*size_patch_w_roi[2]) , 0, d);
    cv::Mat k;
    cv::exp((-d / (sigma_w_roi * sigma_w_roi)), k);
    return k;
}


// Evaluates a Gaussian kernel with bandwidth SIGMA for all relative shifts between input images X and Y, which must both be MxN. They must    also be periodic (ie., pre-processed with a cosine window).
cv::Mat KCFTracker::gaussianCorrelationScale(cv::Mat x1, cv::Mat x2)
{
    using namespace FFTTools;
    cv::Mat c = cv::Mat( cv::Size(size_patch_scale[1], size_patch_scale[0]), CV_32F, cv::Scalar(0) );
    // HOG features
    if (_hogfeatures) {
        cv::Mat caux;
        cv::Mat x1aux;
        cv::Mat x2aux;
        for (int i = 0; i < size_patch_scale[2]; i++) {
            x1aux = x1.row(i);   // Procedure do deal with cv::Mat multichannel bug
            x1aux = x1aux.reshape(1, size_patch_scale[0]);
            x2aux = x2.row(i).reshape(1, size_patch_scale[0]);
            cv::mulSpectrums(fftd_scale(x1aux), fftd_scale(x2aux), caux, 0, true); 
            caux = fftd_scale(caux, true);
            rearrange(caux);
            caux.convertTo(caux,CV_32F);
            c = c + real(caux);
        }
    }
    // Gray features
    else {
        cv::mulSpectrums(fftd_scale(x1), fftd_scale(x2), c, 0, true);
        c = fftd_scale(c, true);
        rearrange(c);
        c = real(c);
    }
    cv::Mat d; 
    cv::max(( (cv::sum(x1.mul(x1))[0] + cv::sum(x2.mul(x2))[0])- 2. * c) / (size_patch_scale[0]*size_patch_scale[1]*size_patch_scale[2]) , 0, d);

    cv::Mat k;
    cv::exp((-d / (sigma_scale * sigma_scale)), k);
    return k;
}

// Create Gaussian Peak. Function called only in the first frame.
cv::Mat KCFTracker::createGaussianPeak(int sizey, int sizex)
{
    cv::Mat_<float> res(sizey, sizex);

    int syh = (sizey) / 2;
    int sxh = (sizex) / 2;

    float output_sigma = std::sqrt((float) sizex * sizey) / padding * output_sigma_factor;
    float mult = -0.5 / (output_sigma * output_sigma);

    for (int i = 0; i < sizey; i++)
        for (int j = 0; j < sizex; j++)
        {
            int ih = i - syh;
            int jh = j - sxh;
            res(i, j) = std::exp(mult * (float) (ih * ih + jh * jh));
        }
    return FFTTools::fftd(res);
}

// Create Gaussian Peak. Function called only in the first frame.
cv::Mat KCFTracker::createGaussianPeakWROI(int sizey, int sizex)
{
    cv::Mat_<float> res(sizey, sizex);

    int syh = (sizey) / 2;
    int sxh = (sizex) / 2;

    float output_sigma = std::sqrt((float) sizex * sizey) / padding_w_roi * output_sigma_factor_w_roi;
    float mult = -0.5 / (output_sigma * output_sigma);

    for (int i = 0; i < sizey; i++)
        for (int j = 0; j < sizex; j++)
        {
            int ih = i - syh;
            int jh = j - sxh;
            res(i, j) = std::exp(mult * (float) (ih * ih + jh * jh));
        }

    return FFTTools::fftd_w_roi(res);
}

// Create Gaussian Peak. Function called only in the first frame.
cv::Mat KCFTracker::createGaussianPeakScale(int sizey, int sizex)
{
    cv::Mat_<float> res(sizey, sizex);

    int syh = (sizey) / 2;
    int sxh = (sizex) / 2;

    float output_sigma = std::sqrt((float) sizex * sizey) / padding_scale * output_sigma_factor_scale;
    float mult = -0.5 / (output_sigma * output_sigma);

    for (int i = 0; i < sizey; i++)
        for (int j = 0; j < sizex; j++)
        {
            int ih = i - syh;
            int jh = j - sxh;
            res(i, j) = std::exp(mult * (float) (ih * ih + jh * jh));
        }

    return FFTTools::fftd_scale(res);
}

// Obtain sub-window from image, with replication-padding and extract features
cv::Mat KCFTracker::getFeatures(const cv::Mat & image, bool inithann, float scale_adjust)
{   

    float cx = _roi.x + _roi.width / 2;
    float cy = _roi.y + _roi.height / 2;

    if (inithann) {
        int padded_w = _roi.width * padding;
        int padded_h = _roi.height * padding;
        
        if (template_size > 1) {  // Fit largest dimension to the given template size
            if (padded_w >= padded_h)  //fit to width
                _scale = padded_w / (float) template_size;
            else
                _scale = padded_h / (float) template_size;

            _tmpl_sz.width = padded_w / _scale;
            _tmpl_sz.height = padded_h / _scale;
        }
        else {  //No template size given, use ROI size
            _tmpl_sz.width = padded_w;
            _tmpl_sz.height = padded_h;
            _scale = 1;
        }

        if (_hogfeatures) {
            // Round to cell size and also make it even
            _tmpl_sz.width = ( ( (int)(_tmpl_sz.width / (2 * cell_size)) ) * 2 * cell_size ) + cell_size*2;
            _tmpl_sz.height = ( ( (int)(_tmpl_sz.height / (2 * cell_size)) ) * 2 * cell_size ) + cell_size*2;
        }
        else {  //Make number of pixels even (helps with some logic involving half-dimensions)
            _tmpl_sz.width = (_tmpl_sz.width / 2) * 2;
            _tmpl_sz.height = (_tmpl_sz.height / 2) * 2;
        }
    }

    extracted_roi.width = scale_adjust * _scale * _tmpl_sz.width;
    extracted_roi.height = scale_adjust * _scale * _tmpl_sz.height;

    // center roi with new size
    extracted_roi.x = cx - extracted_roi.width / 2;
    extracted_roi.y = cy - extracted_roi.height / 2;

    // Resize the ROI
    cv::Mat FeaturesMap;  
    cv::Mat z = RectTools::subwindow(image, extracted_roi, cv::BORDER_REPLICATE);
    
    
if (z.cols != _tmpl_sz.width || z.rows != _tmpl_sz.height) {
        cv::resize(z, z, _tmpl_sz);
    }   

    // HOG features
    if (_hogfeatures) {
        IplImage z_ipl = z;
        CvLSVMFeatureMapCaskade *map;
        getFeatureMaps(&z_ipl, cell_size, &map);
        normalizeAndTruncate(map,0.2f);
        PCAFeatureMaps(map);
        size_patch[0] = map->sizeY;
        size_patch[1] = map->sizeX;
        size_patch[2] = map->numFeatures;

        FeaturesMap = cv::Mat(cv::Size(map->numFeatures,map->sizeX*map->sizeY), CV_32F, map->map);  // Procedure do deal with cv::Mat multichannel bug
        FeaturesMap = FeaturesMap.t();
        freeFeatureMapObject(&map);
    }
    else {
        FeaturesMap = RectTools::getGrayImage(z);
        FeaturesMap -= (float) 0.5; // In Paper;
        size_patch[0] = z.rows;
        size_patch[1] = z.cols;
        size_patch[2] = 1;  
    }
    
    if (inithann) {
        createHanningMats();
    }

    FeaturesMap = hann.mul(FeaturesMap);

    return FeaturesMap;
}
    
// Obtain sub-window from image, with replication-padding and extract features
cv::Mat KCFTracker::getFeaturesWROI(const cv::Mat & image, bool inithann, float scale_adjust)
{
    // cv::Rect extracted_roi;

    float cx = _roi_w.x + _roi_w.width / 2;
    float cy = _roi_w.y + _roi_w.height / 2;

    if (inithann) {
        int padded_wroi_w = _roi_w.width * padding_w_roi;
        int padded_wroi_h = _roi_w.height * padding_w_roi;
        
        if (template_size_w_roi > 1) {           // Fit largest dimension to the given template size
            if (padded_wroi_w >= padded_wroi_h)  //fit to width
                _scale_w_roi = padded_wroi_w / (float) template_size_w_roi;
            else
                _scale_w_roi = padded_wroi_h / (float) template_size_w_roi;

            _tmpl_sz_w_roi.width = padded_wroi_w / _scale_w_roi;
            _tmpl_sz_w_roi.height = padded_wroi_h / _scale_w_roi;
        }
        else {  //No template size given, use ROI size
            _tmpl_sz_w_roi.width = padded_wroi_w;
            _tmpl_sz_w_roi.height = padded_wroi_h;
            _scale_w_roi = 1;
        }
        if (_hogfeatures) {
            // Round to cell size and also make it even
            _tmpl_sz_w_roi.width = ( ( (int)(_tmpl_sz_w_roi.width / (2 * cell_size)) ) * 2 * cell_size ) + cell_size*2;
            _tmpl_sz_w_roi.height = ( ( (int)(_tmpl_sz_w_roi.height / (2 * cell_size)) ) * 2 * cell_size ) + cell_size*2;
        }
        else {  //Make number of pixels even (helps with some logic involving half-dimensions)
            _tmpl_sz_w_roi.width = (_tmpl_sz_w_roi.width / 2) * 2;
            _tmpl_sz_w_roi.height = (_tmpl_sz_w_roi.height / 2) * 2;
        }
    }

    // Region of Interest for the Wide ROI Translation Filter
    extracted_w_roi.width = scale_adjust * _scale_w_roi * _tmpl_sz_w_roi.width;
    extracted_w_roi.height = scale_adjust * _scale_w_roi * _tmpl_sz_w_roi.height;

    // center roi with new size
    extracted_w_roi.x = cx - extracted_w_roi.width / 2;
    extracted_w_roi.y = cy - extracted_w_roi.height / 2;

    // Crop the ROI Area
    cv::Mat FeaturesMap;  
    cv::Mat z = RectTools::subwindow(image, extracted_w_roi, cv::BORDER_REPLICATE);

    // Resize the ROI to the Template
    if (z.cols != _tmpl_sz_w_roi.width || z.rows != _tmpl_sz_w_roi.height) {
        cv::resize(z, z, _tmpl_sz_w_roi);
    }   

    // HOG features
    if (_hogfeatures) {
        IplImage z_ipl = z;
        CvLSVMFeatureMapCaskade *map;
        getFeatureMaps(&z_ipl, cell_size, &map);
        normalizeAndTruncate(map,0.2f);
        PCAFeatureMaps(map);
        size_patch_w_roi[0] = map->sizeY;
        size_patch_w_roi[1] = map->sizeX;
        size_patch_w_roi[2] = map->numFeatures;

        FeaturesMap = cv::Mat(cv::Size(map->numFeatures,map->sizeX*map->sizeY), CV_32F, map->map);  // Procedure do deal with cv::Mat multichannel bug
        FeaturesMap = FeaturesMap.t();
        freeFeatureMapObject(&map);

        // Lab features
        if (_labfeatures) {
            cv::Mat imgLab;
            cvtColor(z, imgLab, CV_BGR2Lab);
            unsigned char *input = (unsigned char*)(imgLab.data);

            // Sparse output vector
            cv::Mat outputLab = cv::Mat(_labCentroids.rows, size_patch_w_roi[0]*size_patch_w_roi[1], CV_32F, float(0));

            int cntCell = 0;
            // Iterate through each cell
            for (int cY = cell_size; cY < z.rows-cell_size; cY+=cell_size){
                for (int cX = cell_size; cX < z.cols-cell_size; cX+=cell_size){
                    // Iterate through each pixel of cell (cX,cY)
                    for(int y = cY; y < cY+cell_size; ++y){
                        for(int x = cX; x < cX+cell_size; ++x){
                            // Lab components for each pixel
                            float l = (float)input[(z.cols * y + x) * 3];
                            float a = (float)input[(z.cols * y + x) * 3 + 1];
                            float b = (float)input[(z.cols * y + x) * 3 + 2];

                            // Iterate trough each centroid
                            float minDist = FLT_MAX;
                            int minIdx = 0;
                            float *inputCentroid = (float*)(_labCentroids.data);
                            for(int k = 0; k < _labCentroids.rows; ++k){
                                float dist = ( (l - inputCentroid[3*k]) * (l - inputCentroid[3*k]) )
                                           + ( (a - inputCentroid[3*k+1]) * (a - inputCentroid[3*k+1]) )
                                           + ( (b - inputCentroid[3*k+2]) * (b - inputCentroid[3*k+2]) );
                                if(dist < minDist){
                                    minDist = dist;
                                    minIdx = k;
                                }
                            }
                            // Store result at output
                            outputLab.at<float>(minIdx, cntCell) += 1.0 / cell_sizeQ;
                            //((float*) outputLab.data)[minIdx * (size_patch_scale[0]*size_patch_scale[1]) + cntCell] += 1.0 / cell_sizeQ; 
                        }
                    }
                    cntCell++;
                }
            }
            // Update size_patch_scale[2] and add features to FeaturesMap
            size_patch_w_roi[2] += _labCentroids.rows;
            FeaturesMap.push_back(outputLab);
        }
    }
    else {
        FeaturesMap = RectTools::getGrayImage(z);
        FeaturesMap -= (float) 0.5; // In Paper;
        size_patch_w_roi[0] = z.rows;
        size_patch_w_roi[1] = z.cols;
        size_patch_w_roi[2] = 1;  
    }
    
    if (inithann) {
        createHanningMatsWROI();
    }

    FeaturesMap = hann_wroi.mul(FeaturesMap);

    return FeaturesMap;
}

// Obtain sub-window from image, with replication-padding and extract features
cv::Mat KCFTracker::getFeaturesScale(const cv::Mat &image, bool inithann, float scale_adjust)
{
    // cv::Rect extracted_roi;

    float cx = _roi_scale.x + _roi_scale.width / 2;
    float cy = _roi_scale.y + _roi_scale.height / 2;

    if (inithann) {
        int padded_w = _roi_scale.width * padding_scale;
        int padded_h = _roi_scale.height * padding_scale;
        
        if (template_size_scale > 1) {  // Fit largest dimension to the given template size
            if (padded_w >= padded_h)  //fit to width
                _scale2 = padded_w / (float) template_size_scale;
            else
                _scale2 = padded_h / (float) template_size_scale;
            _tmpl_sz_scale.width = padded_w / _scale2;
            _tmpl_sz_scale.height = padded_h / _scale2;
        }
        else {  //No template size given, use ROI size
            _tmpl_sz_scale.width = padded_w;
            _tmpl_sz_scale.height = padded_h;
            _scale = 1;
        }

        if (_hogfeatures) {
            // Round to cell size and also make it even
            _tmpl_sz_scale.width = ( ( (int)(_tmpl_sz_scale.width / (2 * cell_size)) ) * 2 * cell_size ) + cell_size*2;
            _tmpl_sz_scale.height = ( ( (int)(_tmpl_sz_scale.height / (2 * cell_size)) ) * 2 * cell_size ) + cell_size*2;
        }
        else {  //Make number of pixels even (helps with some logic involving half-dimensions)
            _tmpl_sz_scale.width = (_tmpl_sz_scale.width / 2) * 2;
            _tmpl_sz_scale.height = (_tmpl_sz_scale.height / 2) * 2;
        }
    }

    extracted_roi_scale.width = scale_adjust * _scale2 * _tmpl_sz_scale.width;
    extracted_roi_scale.height = scale_adjust * _scale2 * _tmpl_sz_scale.height;

    // Center ROI with new size
    extracted_roi_scale.x = cx - extracted_roi_scale.width / 2;
    extracted_roi_scale.y = cy - extracted_roi_scale.height / 2;

    // Crop the Scale Filter ROI
    cv::Mat FeaturesMap;  
    cv::Mat z = RectTools::subwindow(image, extracted_roi_scale, cv::BORDER_REPLICATE);
    
    // Resize the ROI Template
    if (z.cols != _tmpl_sz_scale.width || z.rows != _tmpl_sz_scale.height) {
        cv::resize(z, z, _tmpl_sz_scale);
    } 

    // HOG features
    if (_hogfeatures) {
        IplImage z_ipl = z;
        CvLSVMFeatureMapCaskade *map;
        getFeatureMaps(&z_ipl, cell_size, &map);
        normalizeAndTruncate(map,0.2f);
        PCAFeatureMaps(map);
        size_patch_scale[0] = map->sizeY;
        size_patch_scale[1] = map->sizeX;
        size_patch_scale[2] = map->numFeatures;

        FeaturesMap = cv::Mat(cv::Size(map->numFeatures,map->sizeX*map->sizeY), CV_32F, map->map);  // Procedure do deal with cv::Mat multichannel bug
        FeaturesMap = FeaturesMap.t();
        freeFeatureMapObject(&map);

        // Lab features
        if (_labfeatures) {
            cv::Mat imgLab;
            cvtColor(z, imgLab, CV_BGR2Lab);
            unsigned char *input = (unsigned char*)(imgLab.data);

            // Sparse output vector
            cv::Mat outputLab = cv::Mat(_labCentroids.rows, size_patch_scale[0]*size_patch_scale[1], CV_32F, float(0));

            int cntCell = 0;
            // Iterate through each cell
            for (int cY = cell_size; cY < z.rows-cell_size; cY+=cell_size){
                for (int cX = cell_size; cX < z.cols-cell_size; cX+=cell_size){
                    // Iterate through each pixel of cell (cX,cY)
                    for(int y = cY; y < cY+cell_size; ++y){
                        for(int x = cX; x < cX+cell_size; ++x){
                            // Lab components for each pixel
                            float l = (float)input[(z.cols * y + x) * 3];
                            float a = (float)input[(z.cols * y + x) * 3 + 1];
                            float b = (float)input[(z.cols * y + x) * 3 + 2];

                            // Iterate trough each centroid
                            float minDist = FLT_MAX;
                            int minIdx = 0;
                            float *inputCentroid = (float*)(_labCentroids.data);
                            for(int k = 0; k < _labCentroids.rows; ++k){
                                float dist = ( (l - inputCentroid[3*k]) * (l - inputCentroid[3*k]) )
                                           + ( (a - inputCentroid[3*k+1]) * (a - inputCentroid[3*k+1]) ) 
                                           + ( (b - inputCentroid[3*k+2]) * (b - inputCentroid[3*k+2]) );
                                if(dist < minDist){
                                    minDist = dist;
                                    minIdx = k;
                                }
                            }
                            // Store result at output
                            outputLab.at<float>(minIdx, cntCell) += 1.0 / cell_sizeQ; 
                            //((float*) outputLab.data)[minIdx * (size_patch_scale[0]*size_patch_scale[1]) + cntCell] += 1.0 / cell_sizeQ; 
                        }
                    }
                    cntCell++;
                }
            }
            // Update size_patch_scale[2] and add features to FeaturesMap
            size_patch_scale[2] += _labCentroids.rows;
            FeaturesMap.push_back(outputLab);
        }
    }
    else {
        FeaturesMap = RectTools::getGrayImage(z);
        FeaturesMap -= (float) 0.5; // In Paper;
        size_patch_scale[0] = z.rows;
        size_patch_scale[1] = z.cols;
        size_patch_scale[2] = 1;  
    }
    return FeaturesMap;
}


// Initialize Hanning window. Function called only in the first frame.
void KCFTracker::createHanningMats()
{   
    cv::Mat hann1t = cv::Mat(cv::Size(size_patch[1],1), CV_32F, cv::Scalar(0));
    cv::Mat hann2t = cv::Mat(cv::Size(1,size_patch[0]), CV_32F, cv::Scalar(0)); 

    for (int i = 0; i < hann1t.cols; i++)
        hann1t.at<float > (0, i) = 0.5 * (1 - std::cos(2 * 3.14159265358979323846 * i / (hann1t.cols - 1)));
    for (int i = 0; i < hann2t.rows; i++)
        hann2t.at<float > (i, 0) = 0.5 * (1 - std::cos(2 * 3.14159265358979323846 * i / (hann2t.rows - 1)));

    cv::Mat hann2d = hann2t * hann1t;
    // HOG features
    if (_hogfeatures) {
        cv::Mat hann1d = hann2d.reshape(1,1); // Procedure do deal with cv::Mat multichannel bug
        
        hann = cv::Mat(cv::Size(size_patch[0]*size_patch[1], size_patch[2]), CV_32F, cv::Scalar(0));
        for (int i = 0; i < size_patch[2]; i++) {
            for (int j = 0; j<size_patch[0]*size_patch[1]; j++) {
                hann.at<float>(i,j) = hann1d.at<float>(0,j);
            }
        }
    }
    // Gray features
    else {
        hann = hann2d;
    }
}

// Initialize Hanning window. Function called only in the first frame.
void KCFTracker::createHanningMatsWROI()
{
    cv::Mat hann1t = cv::Mat(cv::Size(size_patch_w_roi[1],1), CV_32F, cv::Scalar(0));
    cv::Mat hann2t = cv::Mat(cv::Size(1,size_patch_w_roi[0]), CV_32F, cv::Scalar(0));

    for (int i = 0; i < hann1t.cols; i++)
        hann1t.at<float > (0, i) = 0.5 * (1 - std::cos(2 * 3.14159265358979323846 * i / (hann1t.cols - 1)));
    for (int i = 0; i < hann2t.rows; i++)
        hann2t.at<float > (i, 0) = 0.5 * (1 - std::cos(2 * 3.14159265358979323846 * i / (hann2t.rows - 1)));

    cv::Mat hann2d = hann2t * hann1t;
    // HOG features
    if (_hogfeatures) {
        cv::Mat hann1d = hann2d.reshape(1,1); // Procedure do deal with cv::Mat multichannel bug

        hann_wroi = cv::Mat(cv::Size(size_patch_w_roi[0]*size_patch_w_roi[1], size_patch_w_roi[2]), CV_32F, cv::Scalar(0));
        for (int i = 0; i < size_patch_w_roi[2]; i++) {
            for (int j = 0; j<size_patch_w_roi[0]*size_patch_w_roi[1]; j++) {
                hann_wroi.at<float>(i,j) = hann1d.at<float>(0,j);
            }
        }
    }
    // Gray features
    else {
        hann_wroi = hann2d;
    }
}
  
float KCFTracker::subPixelPeak(float left, float center, float right)
{   
    float divisor = 2 * center - right - left;
    if (divisor == 0)
        return 0;
    
    return 0.5 * (right - left) / divisor;
}

cv::Rect_<float> KCFTracker::applyHomography(cv::Mat homography, cv::Mat image, cv::Rect_<float> roi){

    float cx = roi.x + roi.width/2.0;  // Determine the Central Point
    float cy = roi.y + roi.height/2.0; // Determine the Central Point

    // Transform to the New Position
    cx = cx * homography.at<double>(0,0) + cy * homography.at<double>(0,1) + 1 * homography.at<double>(0,2);
    cy = cx * homography.at<double>(1,0) + cy * homography.at<double>(1,1) + 1 * homography.at<double>(1,2);
    float cz = cx * homography.at<double>(2,0) + cy * homography.at<double>(2,1) + 1 * homography.at<double>(2,2);

    // Find the new ROI
    roi.x = cx - roi.width/2.0;
    roi.y = cy - roi.height/2.0;
 
    // Boundary condition
    if (roi.x + roi.width <= 0)  roi.x = -roi.width + 1;
    if (roi.y + roi.height <= 0) roi.y = -roi.height + 1;
    if (roi.x >= image.cols - 1) roi.x = image.cols - 2;
    if (roi.y >= image.rows - 1) roi.y = image.rows - 2;

    assert(roi.width >= 0 && roi.height >= 0);
 
    return roi;
}

void PrecisionCurve(std::vector<std::vector<float>> EucDistance, std::string prDataFile, float runTime)
{
    // Save into the Corresponding Text File
    std::ofstream prFile("/home/buzkent/Downloads/Results/Precision/"+prDataFile+".txt");
    std::ofstream successFile("/home/buzkent/Downloads/Results/Success/"+prDataFile+".txt");
    std::ofstream runTimeFile("/home/buzkent/Downloads/Results/RunTime/RunTimes.txt", std::ios::app);    
    for (int i = 0; i <= 50; i++){      	/// \param[in] i Spatial Threshold
        float precision = 0;
        for(int j = 0; j < EucDistance[0].size(); j++){ /// Check each time step
            if ( i != 0){
               if (EucDistance[0][j] <= i){ 	// Compare With Threshold for Precision
                  precision += 1;       	// Successfull Tracking
               }
            }
            else{
               if (EucDistance[0][j] == i){     // Compare With Threshold for Precision
                  precision += 1;               // Successfull Tracking
               }
            }
        }
        prFile << (precision/(float)EucDistance[0].size());
        prFile << std::endl;
    }

    for (int i = 0; i <= 100; i+=1){             /// \param[in] i Spatial Threshold
        float success = 0;
        for(int j = 0; j < EucDistance[1].size(); j++){ /// Check each time step
            if (EucDistance[1][j] > i){     // Compare with Threshold for Success Overlap
               success += 1;               // Successfull Tracking  
            }
        }
        successFile << (success/(float)EucDistance[1].size());
        successFile << std::endl;
    }

    runTimeFile << runTime;
    runTimeFile << std::endl;
}

float KCFTracker::computePSR(const Mat &correlation_mat)
{//Compute Peak-to-Sidelobe Ratio

    double max_val = 0;
    Point max_loc;
    cv::Mat PSR_mask = cv::Mat::ones(correlation_mat.rows,correlation_mat.cols, CV_8U);
    cv::Scalar mean,stddev;

    minMaxLoc(correlation_mat,NULL,&max_val,NULL,&max_loc);     //Get location of max arg

    //Define PSR mask
    int win_size = floor(11/2);
    Rect mini_roi = Rect(std::max(max_loc.x - win_size,0), std::max(max_loc.y - win_size,0), 11, 11);

    //Handle image boundaries
    if ( (mini_roi.x+mini_roi.width) > PSR_mask.cols )
    {
        mini_roi.width = PSR_mask.cols - mini_roi.x;
    }
    if ( (mini_roi.y+mini_roi.height) > PSR_mask.rows )
    {
        mini_roi.height = PSR_mask.rows - mini_roi.y;
    }

    Mat temp = PSR_mask(mini_roi);
    temp *= 0;
    meanStdDev(correlation_mat,mean,stddev,PSR_mask);   //Compute matrix mean and std

    return (max_val - mean.val[0]) / stddev.val[0];     //Compute PSR
}
