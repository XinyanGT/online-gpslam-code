/* ----------------------------------------------------------------------------
A new factor of GTSAM 3.2
https://collab.cc.gatech.edu/borg/gtsam/
* -------------------------------------------------------------------------- */

/**
*  @file  VFactor.H
*  @author Xinyan Yan
**/


#pragma once
#include <ostream>

#include <boost/lexical_cast.hpp>
#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/geometry/Pose2.h>

namespace gtsam {

	/**
	* 2-way factor for projected velocity
	*	cos(theta) * xdot + sin(theta) * ydot = distdot
	*	thetadot = thetadot
	* @addtogroup SLAM
	*/

	template<class VELOCITY>
	class VFactor: public NoiseModelFactor2<VELOCITY, VELOCITY> {

	private:
		Vector measured_; /** measurement   distdot, 0, thetadot */
		typedef VFactor<VELOCITY> This;
		typedef NoiseModelFactor2<VELOCITY, VELOCITY> Base;
	public:

		VFactor() {}	/* Default constructor */
		
		VFactor(Key poseKey, Key velKey, const VELOCITY& velMeasured, const SharedNoiseModel& model) :
			Base(model, poseKey, velKey) {
			measured_ = velMeasured.vector();
		}

		virtual ~VFactor() {}

		/// @return a deep copy of this factor
		virtual gtsam::NonlinearFactor::shared_ptr clone() const {
			return boost::static_pointer_cast<gtsam::NonlinearFactor>(
				gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

		/** h(x)-z */
		// Give current estimate of pose and full velocity,
		// Calculate error and Jacobians
		Vector evaluateError(const VELOCITY& poseP, const VELOCITY& velP, 
			boost::optional< Matrix & > H1 = boost::none, boost::optional< Matrix & > H2 = boost::none) const {

				Vector vel = velP.vector();
				Vector pose = poseP.vector();
				double s = sin(pose(2)), c = cos(pose(2));
/*
				std::cout << "********************************************" << std::endl;
				std::cout << "In VFactor evaluateError" << std::endl;
				std::cout << "pose: \n" << pose << std::endl;
				std::cout << "vel: \n" << vel << std::endl;
				std::cout << "error: \n" <<  (Vector(3) << c * vel(0) + s * vel(1) - measured_(0), -s * vel(0) + c * vel(1) - measured_(1), vel(2) - measured_(2)) << std::endl;
*/					
				// compute Jacobian
				if (H1) {

					*H1 = (Matrix(3,3) << 0, 0, vel(0) * (-s) + vel(1) * c,
										  0, 0, vel(0) * (-c) + vel(1) * (-s),
										  0, 0, 0);
					*H2 = (Matrix(3,3) << c, s, 0,
										  -s, c, 0,
										  0, 0, 1);
/*
					std::cout << "H1: \n" << *H1 << std::endl;
					std::cout << "H2: \n" << *H2 << std::endl;
*/
				}
				Vector ret = (Vector(3) << c * vel(0) + s * vel(1) - measured_(0), -s * vel(0) + c * vel(1) - measured_(1), vel(2) - measured_(2));
				return ret;
		}


		/** return the measured */
		    /** return the measured */
		const Vector& measured() const {
			return measured_;
		}

		/** number of variables attached to this factor */
		std::size_t size() const {
			return 2;
		}

		/** equals specialized to this factor */
		virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
			const This *e =  dynamic_cast<const This*> (&expected);
			return e != NULL && Base::equals(*e, tol) &&  LieVector(this->measured_).equals(LieVector(e->measured_), tol);
		}

		/** print contents */
		void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
			std::cout << s << "2-way projected velocity factor" << std::endl;
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
	}; // VFactor

} // namespace gtsam
