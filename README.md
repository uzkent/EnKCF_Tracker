<p> This code includes the Ensemble of Kernelized Correlation Filter Tracker for the BMVC17 Submission.
The EnKCF runs multiple KCFs [1] to tackle different aspects of tracking such as : scaling, and fast motion.
We also employ a Particle Filter to smoothen the interaction among different KCFs. Our tracker achieves
higher success and precision rates than the baseline KCF tracker at 416hz on UAV123 dataset. We will share
more details on our tracker soon. Below, you can find the hyperparameters and their optimal values for
the proposed EnKCF tracker. </p>

<ul>
<li> sigma_scale = 0.7   // Gaussian Kernel in KCF
<li> sigma_large_roi_translation = 0.7 // Gaussian Kernel in KCF
<li> sigma_small_roi_translation = 0.6 // Gaussian Kernel in KCF
<li> lambda = 0.0001 // Regularization Weight - Same for all the KCFs
<li> scale_filter_frequency = 5 // Scale Filter Applied every 5 frames
<li> learning_rate_scale = 0.15
<li> learning_rate_large_roi_translation = 0.20
<li> learning_rate_small_roi_translation = 0.20
<li> scale_filter_training_psr_threshold = 4.0 // Threshold to Train Scale Filter
<li> padding_scale_filter = 1.0 // Area to Consider for Scale Filter
<li> padding_large_roi_translation = 3.0 // Area to Consider for Large Area Trans. Filter
<li> padding_small_roi_translation = 2.5 // Area to COnsider for Small Area Trans. Filter
<li> responsevariance_scale = 0.04	// Variance for Desired Gaussian Response
<li> responsevariance_large_roi_translation = 0.06
<li> responsevariance_small_roi_translation = 0.125
<li> number_particles = 1000		// Number of Particles in the Particle Filter
<li> number_efficient_particles = 1000/3.0 // Number of Efficient Particles to Enable Resampling
</ul>

[1] - Henriques, João F., Rui Caseiro, Pedro Martins, and Jorge Batista. "High-speed tracking with kernelized correlation filters." IEEE Transactions on Pattern Analysis and Machine Intelligence 37, no. 3 (2015): 583-596.

