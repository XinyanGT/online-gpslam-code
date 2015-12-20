/* ----------------------------------------------------------------------------
A new factor of GTSAM 3.2
https://collab.cc.gatech.edu/borg/gtsam/
* -------------------------------------------------------------------------- */

/**
*  @file  BearingRangeFactorLV.H
*  @author Xinyan Yan
**/

#pragma once

#include <boost/lexical_cast.hpp>
#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>

namespace gtsam {

	/**
	* Binary factor for a range measurement
	* @addtogroup SLAM
	*/
	template<class POSE, class POINT, class ROTATION>
	class BearingRangeFactorLV: public NoiseModelFactor2<POSE, POINT> {
	public:
		typedef BearingRangeFactorLV<POSE, POINT, ROTATION> This;
		typedef NoiseModelFactor2<POSE, POINT> Base;
		typedef boost::shared_ptr<This> shared_ptr;
	private:

		typedef POSE Pose;
		typedef ROTATION Rot;
		typedef POINT Point;

		// the measurement
		Rot measuredBearing_;
		double measuredRange_;

	public:

		BearingRangeFactorLV() {} /* Default constructor */

		BearingRangeFactorLV(Key poseKey, Key pointKey, const Rot& measuredBearing, const double measuredRange,
			const SharedNoiseModel& model) :
		Base(model, poseKey, pointKey), measuredBearing_(measuredBearing), measuredRange_(measuredRange) {
		}

		virtual ~BearingRangeFactorLV() {}

		/// @return a deep copy of this factor
		virtual gtsam::NonlinearFactor::shared_ptr clone() const {
			return boost::static_pointer_cast<gtsam::NonlinearFactor>(
				gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

		/** h(x)-z */
		Vector evaluateError(const POSE& posep2, const POINT& point,
			boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {

				Vector pose = posep2.vector();
				Point d = point - POINT(pose(0), pose(1));
				Matrix H;
				double y2 = d.norm(H);

				double s = sin(pose(2)), c = cos(pose(2));
				Point q = Point(c * d.x() + s * d.y(), -s * d.x() + c * d.y());
				double x = q.x(), y = q.y(), d2 = x * x + y * y, n = sqrt(d2);
				Rot y1;
				if(fabs(n) > 1e-5) {
					y1 = Rot::fromCosSin(x / n, y / n);
				} else {
					y1 = Rot();
				}

				Vector e1 = Rot::Logmap(measuredBearing_.between(y1));
				Vector e2 = (Vector(1) << y2 - measuredRange_);

				Matrix H11, H21, H12, H22;

				boost::optional<Matrix&> H11_ = H1 ? boost::optional<Matrix&>(H11) : boost::optional<Matrix&>();
				boost::optional<Matrix&> H21_ = H1 ? boost::optional<Matrix&>(H21) : boost::optional<Matrix&>();
				boost::optional<Matrix&> H12_ = H2 ? boost::optional<Matrix&>(H12) : boost::optional<Matrix&>();
				boost::optional<Matrix&> H22_ = H2 ? boost::optional<Matrix&>(H22) : boost::optional<Matrix&>();


				//std::cout << "*******In BearingRangeFactorLV evaluateError**********" << std::endl;
				//std::cout << "e1:\n" << e1 << std::endl;
				//std::cout << "e2:\n" << e2 << std::endl;
				//std::cout << "s: " << s << " c: " << c << " x: " << x << " y: " << y << " y1: " << y1.theta() << " y2: " << y2 << std::endl;

				if(H1) {

					*H11_ = (Matrix(2,3) << -c, -s, y,
											s, -c, -x);
					*H12_ = (Matrix(2,2) << c, s,
											-s, c);

					//std::cout << "*H11_\n" << *H11_ << std::endl;
					//std::cout << "*H12_\n" << *H12_ << std::endl;

					Matrix temp;
					if(fabs(n) > 1e-5) {
						temp = (Matrix(1,2) << -y/d2, x/d2);
					} else {
						temp = (Matrix(1,2) << 0.0, 0.0);
					}
					//std::cout << "temp\n" << temp << std::endl;

					*H11_ = temp * (*H11_);
					*H12_ = temp * (*H12_);

					*H21_ = (Matrix(1,3) << -H, 0);
					*H22_ = H;


				} 
				if (H1) *H1 = gtsam::stack(2, &H11, &H21);
				if (H2) *H2 = gtsam::stack(2, &H12, &H22);
				//if (H1) {
				//	std::cout << "*H1\n" << *H1 << std::endl;
				//	std::cout << "*H2\n" << *H2 << std::endl;
				//}


				return concatVectors(2, &e1, &e2);
		}




		/** return the measured */
		const std::pair<Rot, double> measured() const {
			return std::make_pair(measuredBearing_, measuredRange_);
		}


		/** equals specialized to this factor */
		virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
			const This *e =  dynamic_cast<const This*> (&expected);
			return e != NULL && Base::equals(*e, tol) &&
				fabs(this->measuredRange_ - e->measuredRange_) < tol &&
				this->measuredBearing_.equals(e->measuredBearing_, tol);
		}

		/** print contents */
		void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
			std::cout << s << "BearingRangeFactorLV("
				<< keyFormatter(this->key1()) << ","
				<< keyFormatter(this->key2()) << ")\n";
			measuredBearing_.print("measured bearing: ");
			std::cout << "measured range: " << measuredRange_ << std::endl;
			this->noiseModel_->print("noise model:\n");
		}

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("NoiseModelFactor2",
				boost::serialization::base_object<Base>(*this));
			ar & BOOST_SERIALIZATION_NVP(measuredBearing_);
			ar & BOOST_SERIALIZATION_NVP(measuredRange_);
		}
	}; // BearingRangeFactorLV

} // namespace gtsam

