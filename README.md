<p> This code includes the Ensemble of Kernelized Correlation Filter Tracker for the BMVC17 Submission.
The EnKCF runs multiple KCFs to tackle different aspects of tracking such as : scaling, and fast motion.
We also employ a Particle Filter to smoothen the interaction among different KCFs. Our tracker achieves
higher success and precision rates than the baseline KCF tracker at 416hz on UAV123 dataset. We will share
more details on our tracker soon. Below, you can find the hyperparameters and their optimal values for
the proposed EnKCF tracker. </p>

<ul>
<li> sigma_scale = 0.7
<li> sigma_large_roi_translation = 0.7
<li> sigma_small_roi_translation = 0.7
<li> lambda = 0.0001 => Same for all the KCFs
<li> scale_filter_frequency = every 5 frames
<li> learning_rate_scale = 0.15
<li> learning_rate_large_roi_translation = 0.20
<li> learning_rate_small_roi_translation = 0.20
<li> scale_filter_training_psr_threshold = 4.0
</ul>
