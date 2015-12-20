/* ----------------------------------------------------------------------------
A new factor of GTSAM 3.2
https://collab.cc.gatech.edu/borg/gtsam/
* -------------------------------------------------------------------------- */

/**
*  @file  InterpolatedVFactor.H
*  @author Xinyan Yan
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
	* 4-way factor for projected velocity after interpolation
	* Assume odometry measurements are robot-oriented velocity and heading velocity
	*	cos(theta) * xdot + sin(theta) * ydot = distdot
	*	thetadot = thetadot
	* @addtogroup SLAM
	*/

	template<class POSE, class VELOCITY>

	// Given p and pdot
	class InterpolatedVFactor: public NoiseModelFactor4<POSE, VELOCITY, POSE, VELOCITY> {

	private:
		Vector measured_;		// measurement  distdot, 0, thetadot
		double delta_t_;		// t_{i+1} - t_i
		double tao_;			// tao - t_i. we use tao as time interval from t_i instead of from t_0 as in Barfoot papers
		Matrix Lambda_;
		Matrix Psi_;
		typedef InterpolatedVFactor<POSE, VELOCITY> This;
		typedef NoiseModelFactor4<POSE, VELOCITY, POSE, VELOCITY> Base;


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

		InterpolatedVFactor() {}	/* Default constructor */

		// p key, pdot key, p key, pdotkey, delta_t, tao, odometry measurement
		InterpolatedVFactor(Key poseKey1, Key velKey1, Key poseKey2, Key velKey2, double delta_t, double tao, const VELOCITY& vel, const SharedNoiseModel& model) :
			Base(model, poseKey1, velKey1, poseKey2, velKey2) {
				measured_ = vel.vector();
				delta_t_ = delta_t;
				tao_ = tao;

				// Calcuate Lambda and Psi
				// Note: Assume that the value of Qc does not matter
				Matrix Qc = Eigen::MatrixXd::Identity(3,3);
				Lambda_ = calcLambda(Qc, delta_t_, tao_);
				Psi_ = calcPsi(Qc, delta_t_, tao_);

		}

		virtual ~InterpolatedVFactor() {}

		/// @return a deep copy of this factor
		virtual gtsam::NonlinearFactor::shared_ptr clone() const {
			return boost::static_pointer_cast<gtsam::NonlinearFactor>(
				gtsam::NonlinearFactor::shared_ptr(new This(*this))); }


		/** h(x)-z */
		Vector evaluateError(const POSE& pose1, const VELOCITY& vel1, const POSE& pose2, const VELOCITY& vel2,
			boost::optional< Matrix & > H1 = boost::none, boost::optional< Matrix & > H2 = boost::none,
			boost::optional< Matrix & > H3 = boost::none, boost::optional< Matrix & > H4 = boost::none) const {

				Matrix A; 
				Matrix B;
				Matrix C;

				// Interpolation
				Vector x = (Vector(6) << pose1(0), pose1(1), pose1(2), vel1(0), vel1(1), vel1(2));
				Vector y = (Vector(6) << pose2(0), pose2(1), pose2(2), vel2(0), vel2(1), vel2(2));
				Vector z = Lambda_ * x + Psi_ * y;

				Vector vel = z.tail(3);
				Vector pose = z.head(3);
				double s = sin(pose(2)), c = cos(pose(2));

					// compute Jacobian
				if (H1) {

					// Gradient to p
					A = (Matrix(3,3) <<	  0.0, 0.0, vel(0) * (-s) + vel(1) * c,
										  0.0, 0.0, vel(0) * (-c) + vel(1) * (-s),
										  0.0, 0.0, 0.0);

					// Gradient to pdot
					B = (Matrix(3,3) <<   c, s, 0.0,
										  -s, c, 0.0,
										  0.0, 0.0, 1.0);

					// Gradient to combined
					C = (Matrix(3,6) << A, B);

					// Chain rule
					*H1 = C * Lambda_.block(0, 0, 6, 3);
					*H2 = C * Lambda_.block(0, 3, 6, 3);
					*H3 = C * Psi_.block(0, 0, 6, 3);
					*H4 = C * Psi_.block(0, 3, 6, 3);

				}


				// Error
				Vector ret = (Vector(3) << c * vel(0) + s * vel(1) - measured_(0), -s * vel(0) + c * vel(1) - measured_(1), vel(2) - measured_(2));
				return ret;
		}



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
			return e != NULL && Base::equals(*e, tol) &&  ( (this->measured_- e->measured_).norm() < tol) && this->delta_t_ == e->delta_t_ && this->tao_ == e->tao_;
		}

		/** print contents */
		void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
			std::cout << s << "4-way projected velocity factor after interpolation" << std::endl;
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
	}; // InterpolatedVFactor

} // namespace gtsam
