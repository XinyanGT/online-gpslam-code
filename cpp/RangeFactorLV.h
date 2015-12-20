/* ----------------------------------------------------------------------------
A new factor of GTSAM 3.2
https://collab.cc.gatech.edu/borg/gtsam/
* -------------------------------------------------------------------------- */

/**
 *  @file  RangeFactorLV.H
 *  @author Xinyan Yan
 **/

#pragma once

#include <boost/lexical_cast.hpp>
#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

  /**
   * Binary factor for a range measurement
   * @addtogroup SLAM
   */
  template<class POSE, class POINT>
  class RangeFactorLV: public NoiseModelFactor2<POSE, POINT> {
  private:

    double measured_; /** measurement */
    typedef RangeFactorLV<POSE, POINT> This;
    typedef NoiseModelFactor2<POSE, POINT> Base;
    typedef POSE Pose;
    typedef POINT Point;

  public:

    RangeFactorLV() {} /* Default constructor */

    RangeFactorLV(Key poseKey, Key pointKey, double measured,
        const SharedNoiseModel& model) :
          Base(model, poseKey, pointKey), measured_(measured) {
    }

    virtual ~RangeFactorLV() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** h(x)-z */
    Vector evaluateError(const POSE& pose, const POINT& point,
        boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {

	  Vector vec = pose.vector();
	  Point d = point - Point(vec(0), vec(1));
	  Matrix H;
	  double r = d.norm(H);
//	  std::cout << "In LieVector range\n d: " << d << "	range: " << d.norm() << std::endl;
//      std::cout << "H: \n" << H << std::endl;
	  if(H1) {
		  *H1 = (Matrix(1,3) << -H, 0.0);
		  *H2 = H;
//		  std::cout << "H1: \n" << *H1 << std::endl;
//		  std::cout << "H2: \n" << *H2 << std::endl;
      } 
      return (Vector(1) << r - measured_);
    }

    /** return the measured */
    double measured() const {
      return measured_;
    }

    /** equals specialized to this factor */
    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
      const This *e = dynamic_cast<const This*> (&expected);
      return e != NULL
          && Base::equals(*e, tol)
          && fabs(this->measured_ - e->measured_) < tol;

    }

    /** print contents */
    void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "RangeFactor, range = " << measured_ << std::endl;
      Base::print("", keyFormatter);
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("NoiseModelFactor2",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measured_);
    }
  }; // RangeFactorLV

} // namespace gtsam
