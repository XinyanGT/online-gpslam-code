# Welcome to online-gpslam
[Gaussian process|incremental|GTSAM 3.2|SLAM]
>Here is the code repository for the paper `Incremental Sparse GP Regression for Continuous-time Trajectory Estimation and Mapping`  *By Xinyan Yan, Vadim Indelman, and Byron Boots*

## Introduction
The code is dependent on [GTSAM 3.2](https://collab.cc.gatech.edu/borg/gtsam/), which implements *smoothing and mapping (SAM)* in robotics and vision, using *factor graphs* and *Bayes networks* as the underlying computing paradigm rather than sparse matrices.

The code consists of two parts:
* **cpp** :c++ code that implements the factors used in the paper, including:
	- 2D Gaussian process prior factor ([GaussianProcessPriorPose2.h](https://github.com/XinyanGT/online-gpslam-code/blob/master/cpp/GaussianProcessPriorPose2.h))
	- BearingRange factor for LieVector ([BearingRangeFactorLV.h](https://github.com/XinyanGT/online-gpslam-code/blob/master/cpp/BearingRangeFactorLV.h))
	- Range factor for LieVector ([RangeFactorLV.h](https://github.com/XinyanGT/online-gpslam-code/blob/master/cpp/RangeFactorLV.h))
	- Interpolated Range factor ([InterpolatedRangeFactor.h](https://github.com/XinyanGT/online-gpslam-code/blob/master/cpp/InterpolatedRangeFactor.h))
	- Projected velocity factor ([VFactor.h](https://github.com/XinyanGT/online-gpslam-code/blob/master/cpp/VFactor.h))
	- Interpolated velocity factor ([InterpolatedVFactor.h](https://github.com/XinyanGT/online-gpslam-code/blob/master/cpp/InterpolatedVFactor.h))
* **matlab**: matlab scripts that produce the experimental results in the paper. We conducted experiments on one synthetic and two real-world datasets:
	- synthetic dataset with 1,500 poses
	- autonomous mower Plaza dataset available at [here](http://www.frc.ri.cmu.edu/projects/emergencyresponse/RangeData/)
	- victoria park vehicle dataset available at [here](http://www-personal.acfr.usyd.edu.au/nebot/victoria_park.htm)
	
In particular, matlab scripts in names like `XXX_periodic` correspond to the experiments using the *periodic batch update (PB)* approach, and scrips in names like `XXX_isam2` correspond to the experiments using the *Bayes tree with Gaussian process (BTGP)* approach.

## Installation
The code relies on GTSAM 3.2. After installing GTSAM 3.2 and cloning the repository, just execute the following commands in a shell:
``` powershell
cd [repo_folder] 	# go to the repo folder
mkdir build      
cd build
cmake ..	  		# configure
make 				# build
sudo make install 	# install
```
Please reach me at <voidpointer@gatech.edu> if there's any problem. Thanks!
# Welcome to online-gpslam
[Gaussian process|incremental|GTSAM 3.2|SLAM]
>Here is the code repository for the paper `Incremental Sparse GP Regression for Continuous-time Trajectory Estimation and Mapping`  *By Xinyan Yan, Vadim Indelman, and Byron Boots*

## Introduction
The code is dependent on [GTSAM 3.2](https://collab.cc.gatech.edu/borg/gtsam/), which implements *smoothing and mapping (SAM)* in robotics and vision, using *factor graphs* and *Bayes networks* as the underlying computing paradigm rather than sparse matrices.

The code consists of two parts:
- **cpp** :c++ code that implements the factors used in the paper, including:
	- 2D Gaussian process prior factor (
	- BearingRange factor for LieVector
	- Range factor for LieVector
	- Interpolated Range factor
	- Projected velocity factor
	- Interpolated velocity factor
- **matlab**: matlab scripts that produce the experimental results in the paper. We conducted experiments on one synthetic and two real-world datasets:
	- synthetic dataset with 1,500 poses
	- autonomous mower Plaza dataset available at [here](http://www.frc.ri.cmu.edu/projects/emergencyresponse/RangeData/)
	- victoria park vehicle dataset available at [here](http://www-personal.acfr.usyd.edu.au/nebot/victoria_park.htm)
	
In particular, matlab scripts in names like `XXX_periodic` correspond to the experiments using the *periodic batch update (PB)* approach, and scrips in names like `XXX_isam2` correspond to the experiments using the *Bayes tree with Gaussian process (BTGP)* approach.

## Installation
The code relies on GTSAM 3.2. After installing GTSAM 3.2 and cloning the repository, just execute the following commands in a shell:
``` powershell
cd [repo_folder] 	# go to the repo folder
mkdir build      
cd build
cmake ..	  		# configure
make 				# build
sudo make install 	# install
```
Please reach me at <voidpointer@gatech.edu> if there's any problem. Thanks!