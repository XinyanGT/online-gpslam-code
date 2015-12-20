/* ----------------------------------------------------------------------------
A new factor of GTSAM 3.2
https://collab.cc.gatech.edu/borg/gtsam/
* -------------------------------------------------------------------------- */

/**
*  @file  GaussianProcessPriorPose2.H
*  @author Xinyan Yan
**/


#pragma once
#include <ostream>

#include <boost/lexical_cast.hpp>
#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>

namespace gtsam {

	/**
	* 4-way factor for Gaussian Process
	* @addtogroup SLAM
	*/
	template<class POSE, class VELOCITY>
	class GaussianProcessPriorPose2: public NoiseModelFactor4<POSE, VELOCITY, POSE, VELOCITY> {
	private:
		double delta_t_;
		Vector measured_;
		typedef GaussianProcessPriorPose2<POSE, VELOCITY> This;
		typedef NoiseModelFactor4<POSE, VELOCITY, POSE, VELOCITY> Base;

	public:

		GaussianProcessPriorPose2() {}	/* Default constructor */
		// delta_t is the time between the two states
		GaussianProcessPriorPose2(Key poseKey1, Key velKey1, Key poseKey2, Key velKey2, double delta_t, const SharedNoiseModel& model) :
			Base(model, poseKey1, velKey1, poseKey2, velKey2) {
			delta_t_ = delta_t;
			measured_ = (Vector(6) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		}


		virtual ~GaussianProcessPriorPose2() {}


		/// @return a deep copy of this factor
		virtual gtsam::NonlinearFactor::shared_ptr clone() const {
			return boost::static_pointer_cast<gtsam::NonlinearFactor>(
				gtsam::NonlinearFactor::shared_ptr(new This(*this))); }


		/** h(x)-z */
		Vector evaluateError(const POSE& pose1, const VELOCITY& vel1, const POSE& pose2, const VELOCITY& vel2, 
			boost::optional< Matrix & > H1 = boost::none, boost::optional< Matrix & > H2 = boost::none,
			boost::optional< Matrix & > H3 = boost::none, boost::optional< Matrix & > H4 = boost::none ) const {

				Vector x = (Vector(6) << pose1(0), pose1(1), pose1(2), vel1(0), vel1(1), vel1(2));
				Vector y = (Vector(6) << pose2(0), pose2(1), pose2(2), vel2(0), vel2(1), vel2(2));

				// Transition function
				Matrix phi = (Matrix(6,6) << eye(3), delta_t_ * eye(3),
											 zeros(3,3), eye(3));

//#if 0
				// Compute Jacobian
				if (H1) {
					*H1 = (Matrix(6,3) << eye(3),
										  zeros(3,3));
					*H2 = (Matrix(6,3) << delta_t_ * eye(3),
										  eye(3));
					*H3 = (Matrix(6,3) << -1.0 * eye(3),
										  zeros(3,3));
					*H4 = (Matrix(6,3) << zeros(3,3),
										  -1.0 * eye(3));
					
				}
//#endif
#if 0
				if (H1) {
					*H1 = (Matrix(3,3) << eye(3));
					*H2 = (Matrix(3,3) << eye(3));
					*H3 = (Matrix(3,3) << -1.0 * eye(3));
					*H4 = (Matrix(3,3) << zeros(3, 3));
				}
#endif
#if 0
				// compute error
				std::cout << "********************************************" << std::endl;
				std::cout << "In Gaussian Process evaluateError: " << std::endl;
				std::cout << "x:\n " << x << std::endl;
				std::cout << "y:\n " << y << std::endl;
				std::cout << "phi:\n " << phi << std::endl;
				std::cout << "error:\n " <<  phi*x-y << std::endl;
//				std::cout << "error:\n " <<  x.head(3) - y.head(3) << std::endl;

				if (H1) {
					std::cout << "H1:\n " << *H1 << std::endl;
					std::cout << "H2:\n " << *H2 << std::endl;
					std::cout << "H3:\n " << *H3 << std::endl;
					std::cout << "H4:\n " << *H4 << std::endl;
				}
#endif
				return phi*x-y;
//				return x.head(3) - y.head(3);
		}





		/** return the measured */
		    /** return the measured */
		const Vector& measured() const {
			return measured_;
		}

		/** number of variables attached to this factor */
		std::size_t size() const {
			return 4;
		}

		/** equals specialized to this factor */
		virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
			const This *e =  dynamic_cast<const This*> (&expected);
			return e != NULL && Base::equals(*e, tol) && fabs(this->delta_t_ - e->delta_t_) < tol;	
		}

		/** print contents */
		void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
			std::cout << s << "4-way Gaussian Process Factor" << std::endl;
			Base::print("", keyFormatter);
		}

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NoiseModelFactor4",
				boost::serialization::base_object<Base>(*this));
			ar & BOOST_SERIALIZATION_NVP(measured_);
		}
	}; // GaussianProcessPriorPose2

} // namespace gtsam
