## Algorithm Description

<p> This code includes the Ensemble of Kernelized Correlation Filter Tracker for the BMVC17 Submission.
The EnKCF runs multiple KCFs [1] to tackle different aspects of tracking such as : scaling, and fast motion.
We also employ a Particle Filter to smoothen the interaction among different KCFs. Our tracker achieves
higher success and precision rates than the baseline KCF tracker at 416hz on UAV123 dataset. We will share
more details on our tracker soon. Below, you can find the hyperparameters and their optimal values for
the proposed EnKCF tracker. This tracker is inspired by the long-term correlation (LCT) tracker proposed by [2], 
however, our goal is to use multiple KCFs in an efficient way to keep the complexity at each frame similar to the 
baseline KCF (O(nlogn)) [1]. </p>

### EnKCF Hyperparameters
<ul>
<li> sigma_scale = 0.9   // Gaussian Kernel Bandwith in KCF
<li> sigma_large_roi_translation = 0.7 // Gaussian Kernel Bandwith in KCF
<li> sigma_small_roi_translation = 0.6 // Gaussian Kernel Bandwith in KCF
<li> lambda = 0.0001 // Regularization Weight - Same for all the KCFs
<li> scale_filter_frequency = 5 // Scale Filter Applied every 5 frames
<li> learning_rate_scale = 0.10
<li> learning_rate_large_roi_translation = 0.20
<li> learning_rate_small_roi_translation = 0.20
<li> scale_filter_training_psr_threshold = 4.0 // Threshold to Train Scale Filter
<li> padding_scale_filter = 1.0 // Area to Consider for Scale Filter
<li> padding_large_roi_translation = 3.0 // Area to Consider for Large Area Trans. Filter
<li> padding_small_roi_translation = 2.5 // Area to COnsider for Small Area Trans. Filter
<li> responsevariance_scale = 0.04	// Variance for Desired Gaussian Response
<li> responsevariance_large_roi_translation = 0.06
<li> responsevariance_small_roi_translation = 0.125
</ul>

### Particle Filter Hyperparameters
<ul>
<li> number_particles = 300		// Number of Particles in the Particle Filter
<li> number_efficient_particles = 1000/3.0 // Number of Efficient Particles to Enable Resampling
<li> process_noise_uniform_x = [-10,10]	// Transition Noise X Coordinate
<li> process_noise_uniform_y = [-10,10] // Transition Noise Y Coordinate
<li> process_noise_uniform_vx = [-2,2]  // Transition Noise X Velocity
<li> process_noise_uniform_vy = [-2,2]  // Transition Noise Y Velocity
<li> transition_noise_uniform_x = [-25,25] // Distribution Interval X Coordinate
<li> transition_noise_uniform_y = [-25,25] // Y Coordinate
<li> transition_noise_uniform_vx = [-10,10] // X Velocity
<li> transition_noise_uniform_vy = [-10,10] // Y Velocity
<li> beta_weight_function = 0.05 // exp(-dist * beta) - Weight Function Hyperparameter - For Importance Sampling based on spatial Eclidean Distance to Maximum of response map
</ul>

[1] - Henriques, João F., Rui Caseiro, Pedro Martins, and Jorge Batista. "High-speed tracking with kernelized correlation filters." IEEE Transactions on Pattern Analysis and Machine Intelligence 37, no. 3 (2015): 583-596.

[2] - Ma, Chao, Xiaokang Yang, Chongyang Zhang, and Ming-Hsuan Yang. "Long-term correlation tracking." In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition, pp. 5388-5396. 2015.
