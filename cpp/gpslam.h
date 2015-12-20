// GP-SLAM wrapper

// delearation
class gtsam::LieVector;
class gtsam::Rot2;
class gtsam::Point2;
class gtsam::Pose2;

class gtsam::GaussianFactorGraph;
class gtsam::Values;
virtual class gtsam::noiseModel::Base;
virtual class gtsam::NonlinearFactor;
virtual class gtsam::NonlinearFactorGraph;
virtual class gtsam::NoiseModelFactor;


/* ************************************************************************** */
// Linear classes
namespace gtsam {

#include <GaussianProcessPriorPose2.h>
template<POSE, VELOCITY>
virtual class GaussianProcessPriorPose2 : gtsam::NoiseModelFactor {
  GaussianProcessPriorPose2(size_t key1, size_t key2, size_t key3, size_t key4, double delta, const gtsam::noiseModel::Base* noiseModel);
};

typedef gtsam::GaussianProcessPriorPose2<gtsam::LieVector,gtsam::LieVector> GaussianProcessPriorPose2LieVector;


#include <InterpolatedVFactor.h>
template<POSE, VELOCITY>
virtual class InterpolatedVFactor: gtsam::NoiseModelFactor {
  InterpolatedVFactor(size_t key1, size_t key2, size_t key3, size_t key4, double delta_t, double tao, const VELOCITY& vel, const gtsam::noiseModel::Base* noiseModel);
};

typedef gtsam::InterpolatedVFactor<gtsam::LieVector, gtsam::LieVector> InterpolatedVFactorLieVector;

 
#include <VFactor.h>
template<POSE>
virtual class VFactor : gtsam::NoiseModelFactor {
  VFactor(size_t key1, size_t key2, const POSE& bt, const gtsam::noiseModel::Base* noiseModel);
};

typedef gtsam::VFactor<gtsam::LieVector> VFactorLieVector;


#include <RangeFactorLV.h>
template<POSE, POINT>
virtual class RangeFactorLV : gtsam::NoiseModelFactor {
  RangeFactorLV(size_t key1, size_t key2, double measured, const gtsam::noiseModel::Base* noiseModel);
};

typedef gtsam::RangeFactorLV<gtsam::LieVector, gtsam::Point2> RangeFactorLVLieVectorPoint2;


#include <InterpolatedRangeFactor.h>
template<POSE, VELOCITY, POINT>
virtual class InterpolatedRangeFactor: gtsam::NoiseModelFactor {
  InterpolatedRangeFactor(size_t key1, size_t key2, size_t key3, size_t key4, size_t key5, double delta_t, double tao, double measured, const gtsam::noiseModel::Base* noiseModel);
};

typedef gtsam::InterpolatedRangeFactor<gtsam::LieVector, gtsam::LieVector, gtsam::Point2> InterpolatedRangeFactorLieVectorPoint2;


#include <BearingRangeFactorLV.h>
template<POSE, POINT, ROTATION>
virtual class BearingRangeFactorLV : gtsam::NoiseModelFactor {
  BearingRangeFactorLV(size_t poseKey, size_t pointKey, const ROTATION& measuredBearing, double measuredRange, const gtsam::noiseModel::Base* noiseModel);

  pair<ROTATION, double> measured() const;

  // enabling serialization functionality
  void serialize() const;
};

typedef gtsam::BearingRangeFactorLV<gtsam::LieVector, gtsam::Point2, gtsam::Rot2> BearingRangeFactorLV2D;
 
}

