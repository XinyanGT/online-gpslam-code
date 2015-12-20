/* ----------------------------------------------------------------------------
A new factor of GTSAM 3.2
https://collab.cc.gatech.edu/borg/gtsam/
* -------------------------------------------------------------------------- */

/**
*  @file  InterpolatedRangeFactor.H
*  @author Frank Dellaert & Xinyan Yan
**/


#pragma once
#include <ostream>

#include <boost/lexical_cast.hpp>
#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/LieVector.h>

namespace gtsam {

	/**
	* 5-way factor for range measurement after interpolation
	* @addtogroup SLAM
	*/

	template<class POSE, class VELOCITY, class POINT>

	// Given p and pdot
	class InterpolatedRangeFactor: public NoiseModelFactor5<POSE, VELOCITY, POSE, VELOCITY, POINT> {

	private:
		double measured_;		// range
		double delta_t_;		// t_{i+1} - t_i
		double tao_;			// tao - t_i. we use tao as time interval from t_i instead of from t_0 as in Barfoot papers
		Matrix Lambda_;
		Matrix Psi_;
		typedef InterpolatedRangeFactor<POSE, VELOCITY, POINT> This;
		typedef NoiseModelFactor5<POSE, VELOCITY, POSE, VELOCITY, POINT> Base;
		typedef POINT Point;

		Matrix calcQ(const Matrix& Qc, double tao) {
			Matrix Q = (Matrix(6,6) <<	1.0 * 1/3 * pow(tao, 3.0) * Qc, 1.0 * 1/2 * pow(tao, 2.0) * Qc,
										1.0 * 1/2 * pow(tao, 2.0) * Qc, 1.0 * tao * Qc);
			return Q;
		}


		Matrix calcQ_inv(const Matrix& Qc, double tao) {
			Matrix Qc_inv = Qc.inverse();
			Matrix Q_inv = (Matrix(6,6) <<	1.0 * 12 * pow(tao, -3.0) * Qc_inv, 1.0 * (-6) * pow(tao, -2.0) * Qc_inv,
											1.0 * (-6) * pow(tao, -2.0) * Qc_inv,  1.0 * 4 * pow(tao, -1.0) * Qc_inv);
			return Q_inv;
		}


		Matrix calcPhi(double tao) {
			Matrix eye = Eigen::MatrixXd::Identity(3,3);
			Matrix zero = Eigen::MatrixXd::Zero(3,3);
			Matrix Phi = (Matrix(6,6) << eye, tao * eye,
										 zero, eye);
			return Phi;
		}


		Matrix calcLambda(const Matrix& Qc, double delta_t, double tao) {
			Matrix Lambda = calcPhi(tao) - calcQ(Qc, tao) * (calcPhi(delta_t-tao).transpose()) * calcQ_inv(Qc, delta_t) * calcPhi(delta_t);
			return Lambda;

		}


		Matrix calcPsi(const Matrix& Qc, double delta_t, double tao) {
			Matrix Psi = calcQ(Qc, tao) * (calcPhi(delta_t-tao).transpose()) * calcQ_inv(Qc, delta_t);
			return Psi;
		}


	public:

		InterpolatedRangeFactor() {}	/* Default constructor */

		// p key, pdot key, p key, pdotkey, delta_t, tao, odometry measurement
		InterpolatedRangeFactor(Key poseKey1, Key velKey1, Key poseKey2, Key velKey2, Key pointKey,
			double delta_t, double tao, double range, const SharedNoiseModel& model) :
			Base(model, poseKey1, velKey1, poseKey2, velKey2, pointKey) {
				measured_ = range;
				delta_t_ = delta_t;
				tao_ = tao;

				// Calcuate Lambda and Psi
				// Assume that the value of Qc does not matter
				Matrix Qc = Eigen::MatrixXd::Identity(3,3);
				Lambda_ = calcLambda(Qc, delta_t_, tao_);
				Psi_ = calcPsi(Qc, delta_t_, tao_);

		}

		virtual ~InterpolatedRangeFactor() {}

		/// @return a deep copy of this factor
		virtual gtsam::NonlinearFactor::shared_ptr clone() const {
			return boost::static_pointer_cast<gtsam::NonlinearFactor>(
				gtsam::NonlinearFactor::shared_ptr(new This(*this))); }


		/** h(x)-z */
		Vector evaluateError(const POSE& pose1, const VELOCITY& vel1, 
								const POSE& pose2, const VELOCITY& vel2, const POINT& point,
			boost::optional< Matrix & > H1 = boost::none, boost::optional< Matrix & > H2 = boost::none,
			boost::optional< Matrix & > H3 = boost::none, boost::optional< Matrix & > H4 = boost::none,
			boost::optional< Matrix & > H5 = boost::none) const {

				Matrix A; 
				Matrix B;
				Matrix C;

				// Interpolation
				Vector x = (Vector(6) << pose1(0), pose1(1), pose1(2), vel1(0), vel1(1), vel1(2));
				Vector y = (Vector(6) << pose2(0), pose2(1), pose2(2), vel2(0), vel2(1), vel2(2));
				Vector z = Lambda_ * x + Psi_ * y;

				Vector pose = z.head(3);
				Vector vec = pose;
				Point d = point - Point(vec(0), vec(1));
				Matrix H;
				double r = d.norm(H);

				// compute Jacobian
				if (H1) {

					// Gradient to p
					A = (Matrix(1,3) <<	-H, 0.0);

					// Gradient to pdot
					B = (Matrix(1,3) << 0.0,  0.0, 0.0);

					// Gradient to combined
					C = (Matrix(1,6) << A, B);

					// Chain rule
					*H1 = C * Lambda_.block(0, 0, 6, 3);
					*H2 = C * Lambda_.block(0, 3, 6, 3);
					*H3 = C * Psi_.block(0, 0, 6, 3);
					*H4 = C * Psi_.block(0, 3, 6, 3);
					*H5 = H;

				}

				// Error
				return (Vector(1) << r - measured_);
		}



		/** return the measured */
		double measured() const {
			return measured_;
		}

		/** number of variables attached to this factor */
		std::size_t size() const {
			return 5;
		}

		/** equals specialized to this factor */
		virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
			const This *e =  dynamic_cast<const This*> (&expected);
			return e != NULL && Base::equals(*e, tol) &&  ( fabs(this->measured_ - e->measured_) < tol) && this->delta_t_ == e->delta_t_ && this->tao_ == e->tao_;
		}

		/** print contents */
		void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
				std::cout << s << "My Interpolated RangeFactor, range = " << measured_ << std::endl;
				Base::print("", keyFormatter);
		}

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NoiseModelFactor5",
				boost::serialization::base_object<Base>(*this));
			ar & BOOST_SERIALIZATION_NVP(measured_);
		}
	}; // InterpolatedRangeFactor

} // namespace gtsam

