EKF-SLAM Simulator
------------------

To watch the video, please click on the picture below!
[![Watch the video](https://github.com/plumewind/ekf_slam/blob/master/ekfslam_sim.PNG)](https://youtu.be/Yhs36RszeTs)

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


From the homework of probabilistic robot course


How to use ?
------------------
Open matlab and enter the following two commands on the command line:

load('example_webmap.mat')

ekfslam_sim(lm,wp)

