EKF-SLAM Simulator
------------------

This simulator demonstrates a simple implementation of
standard EKF-SLAM. It permits simple configuration via 
'configfile.m' to perform SLAM with various control parameters,
noises, etc. Also various switches are available to choose
known data-association versus gating, etc.


The key file in this simulator is called 'ekfslam_sim.m'. Type
'help ekfslam_sim' for more information of how to use it.

In addition to on-line animations, the simulator returns a
data-structure of the logged state information for off-line
processing. An example use of this data is shown in m-file
'plot_feature_loci.m', which plots the trajectories of the 
landmark estimates.

The task is to reprogram this EKF-SLAM algorithm in C++/Python
language. I recomend you use ROS platform for doing this. For the 
matrix operation, I recomend you use Eigen library.

From the homework of probabilistic robot course
