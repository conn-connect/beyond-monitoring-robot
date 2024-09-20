#ifndef __TF_MAT__
#define __TF_MAT__

#include <sstream>
#include <vector>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

#ifndef MS_PI
#define MS_PI 3.14159265358979323846 // pi
#endif
#ifndef MS_PI_HALF
#define MS_PI_HALF 1.57079632679489661923 // pi/2
#endif
#ifndef MS_PI_QUARTER
#define MS_PI_QUARTER 0.78539816339744830962 // pi/4
#endif
#ifndef MS_PI_DOUBLED
#define MS_PI_DOUBLED 6.28318530717958647692 // 2 * MS_PI
#endif
#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * 0.017453293)
#endif
#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * 57.29578)
#endif
#ifndef SIGN
#define SIGN(x) ((x) >= 0 ? (double)(1.f) : (double)(-1.f))
#endif
#ifndef POW2
#define POW2(x) ((x) * (x))
#endif

const double kEpsilonTfMat2 = 1e-7;

template <typename T>
Eigen::Matrix<T, 3, 3> rotz(T deg)
{
	T rad = DEG2RAD(deg);
	Eigen::Matrix<T, 3, 3> result;
	result.col(0) << cos(rad), sin(rad), 0;
	result.col(1) << -sin(rad), cos(rad), 0;
	result.col(2) << 0, 0, 1;
	return result;
}

template <typename T>
Eigen::Matrix<T, 3, 3> roty(T deg)
{
	T rad = DEG2RAD(deg);
	Eigen::Matrix<T, 3, 3> result;
	result.col(0) << cos(rad), 0, -sin(rad);
	result.col(1) << 0, 1, 0;
	result.col(2) << sin(rad), 0, cos(rad);
	return result;
}

template <typename T>
Eigen::Matrix<T, 3, 3> rotx(T deg)
{
	T rad = DEG2RAD(deg);
	Eigen::Matrix<T, 3, 3> result;
	result.col(0) << 1, 0, 0;
	result.col(1) << 0, cos(rad), sin(rad);
	result.col(2) << 0, -sin(rad), cos(rad);
	return result;
}

template <typename T>
Eigen::Matrix<T, 3, 3> skew(Eigen::Matrix<T, 3, 1> xyz)
{
	Eigen::Matrix<T, 3, 3> result;
	result.col(0) << 0.f, xyz.z(), -xyz.y();
	result.col(1) << -xyz.z(), 0.f, xyz.x();
	result.col(2) << xyz.y(), -xyz.x(), 0.f;
	return result;
}

// Expressions to express orientation
//* 0 : rpy - fixed frame(ex : FANUC)
//* 1 : rz-ry-rx (ex : KUKA)
//* 2 : rz-ry-rz (ex : KWSK)
//* 3 : rx-ry-rz (ex : STAUBLI)
//* 4 : Angle - axis (ex : UR) Rodrigues
//* 5 : Quaternion(ex : ABB)
namespace RobotBrand
{
	enum
	{
		FANC,
		KUKA,
		KWSK,
		STBL,
		UR,
		ABB
	};
}
namespace TypeEuler
{
	enum
	{
		RPY,
		ZYX,
		ZYZ,
		XYZ,
		RODRI,
		QUAT
	};
}

template <typename T>
class tf_mat
{

private:
	void IsOrthogonal()
	{
		for (int i = 0; i < 3; ++i)
		{
			// int j = i - 1;
			// if (i == 0)	j = 2;
			// if (fabs(rot_mat.col(i).dot(rot_mat.col(j))) > 1e-3)
			//	rot_mat.setIdentity();
		}
	}

public:
	// EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Eigen::Matrix<T, 3, 3> rot_mat;	  // rotation matrix
	Eigen::Matrix<T, 3, 1> trans_vec; // translation vector

	tf_mat()
	{
		rot_mat.setIdentity();
		trans_vec.setZero();
	};
	~tf_mat() {};

	tf_mat(Eigen::Matrix<T, 3, 3> rot_mat, Eigen::Matrix<T, 3, 1> trans_vec)
	{
		this->rot_mat = rot_mat;
		this->trans_vec = trans_vec;
	};

	tf_mat(cv::Mat cvrot_mat, cv::Mat cvtrans_mat)
	{
		rot_mat(0, 0) = cvrot_mat.at<double>(0, 0);
		rot_mat(0, 1) = cvrot_mat.at<double>(0, 1);
		rot_mat(0, 2) = cvrot_mat.at<double>(0, 2);
		rot_mat(1, 0) = cvrot_mat.at<double>(1, 0);
		rot_mat(1, 1) = cvrot_mat.at<double>(1, 1);
		rot_mat(1, 2) = cvrot_mat.at<double>(1, 2);
		rot_mat(2, 0) = cvrot_mat.at<double>(2, 0);
		rot_mat(2, 1) = cvrot_mat.at<double>(2, 1);
		rot_mat(2, 2) = cvrot_mat.at<double>(2, 2);

		trans_vec(0) = cvtrans_mat.at<double>(0);
		trans_vec(1) = cvtrans_mat.at<double>(1);
		trans_vec(2) = cvtrans_mat.at<double>(2);
	};

	tf_mat(Eigen::Matrix<T, 3, 1> x_axis, Eigen::Matrix<T, 3, 1> y_axis, Eigen::Matrix<T, 3, 1> z_axis)
	{
		rot_mat.col(0) = x_axis;
		rot_mat.col(1) = y_axis;
		rot_mat.col(2) = z_axis;
		trans_vec.setZero();
		IsOrthogonal();
	};

	tf_mat(Eigen::Matrix<T, 3, 1> trans_vec, Eigen::Matrix<T, 3, 1> x_axis, Eigen::Matrix<T, 3, 1> y_axis, Eigen::Matrix<T, 3, 1> z_axis)
	{
		rot_mat.col(0) = x_axis;
		rot_mat.col(1) = y_axis;
		rot_mat.col(2) = z_axis;
		IsOrthogonal();
		this->trans_vec = trans_vec;
	};

	tf_mat(const Eigen::Matrix<T, 4, 4> &e_mat4x4)
	{
		this->rot_mat = e_mat4x4.block(0, 0, 3, 3);
		this->trans_vec = e_mat4x4.block(0, 3, 3, 1);
	}

	Eigen::Matrix<T, 3, 1> axis_x() const
	{
		return rot_mat.col(0);
	}

	Eigen::Matrix<T, 3, 1> axis_y() const
	{
		return rot_mat.col(1);
	}

	Eigen::Matrix<T, 3, 1> axis_z() const
	{
		return rot_mat.col(2);
	}

	/*
	 *@param[in] trans_vec translation vector x, y, z
	 *@param[in] euler euler angle in radian
	 *@param[in] type types are refer to 'TypeEuler' or 'TypeRobot'
	 */
	tf_mat(Eigen::Matrix<T, 3, 1> trans_vec, Eigen::Matrix<T, 3, 1> euler, int type)
	{
		this->trans_vec = trans_vec;

		if (type != TypeEuler::RODRI)
		{
			T c0 = cos(euler.x());
			T s0 = sin(euler.x());
			T c1 = cos(euler.y());
			T s1 = sin(euler.y());
			T c2 = cos(euler.z());
			T s2 = sin(euler.z());

			switch (type)
			{

			case TypeEuler::RPY:
				rot_mat(0, 0) = c2 * c1;
				rot_mat(0, 1) = c2 * s1 * s0 - s2 * c0;
				rot_mat(0, 2) = c2 * s1 * c0 + s2 * s0;

				rot_mat(1, 0) = s2 * c1;
				rot_mat(1, 1) = s2 * s1 * s0 + c2 * c0;
				rot_mat(1, 2) = s2 * s1 * c0 - c2 * s0;

				rot_mat(2, 0) = -s1;
				rot_mat(2, 1) = c1 * s0;
				rot_mat(2, 2) = c1 * c0;
				break;
			case TypeEuler::ZYX:
				rot_mat(0, 0) = c0 * c1;
				rot_mat(0, 1) = c0 * s1 * s2 - s0 * c2;
				rot_mat(0, 2) = c0 * s1 * c2 + s0 * s2;

				rot_mat(1, 0) = s0 * c1;
				rot_mat(1, 1) = s0 * s1 * s2 + c0 * c2;
				rot_mat(1, 2) = s0 * s1 * c2 - c0 * s2;

				rot_mat(2, 0) = -s1;
				rot_mat(2, 1) = c1 * s2;
				rot_mat(2, 2) = c1 * c2;
				break;
			case TypeEuler::ZYZ:
				rot_mat(0, 0) = c0 * c1 * c2 - s0 * s2;
				rot_mat(0, 1) = -c0 * c1 * s2 - s0 * c2;
				rot_mat(0, 2) = c0 * s1;

				rot_mat(1, 0) = s0 * c1 * c2 + c0 * s2;
				rot_mat(1, 1) = -s0 * c1 * s2 + c0 * c2;
				rot_mat(1, 2) = s0 * s1;

				rot_mat(2, 0) = -s1 * c2;
				rot_mat(2, 1) = s1 * s2;
				rot_mat(2, 2) = c1;
				break;
			case TypeEuler::XYZ:
				rot_mat(0, 0) = c1 * c2;
				rot_mat(0, 1) = -c1 * s2;
				rot_mat(0, 2) = s1;

				rot_mat(1, 0) = c0 * s2 + c2 * s0 * s1;
				rot_mat(1, 1) = c0 * c2 - s0 * s1 * s2;
				rot_mat(1, 2) = -c1 * s0;

				rot_mat(2, 0) = s0 * s2 - c0 * c2 * s1;
				rot_mat(2, 1) = c2 * s0 + c0 * s1 * s2;
				rot_mat(2, 2) = c0 * c1;
				break;
			}
		}
		else
		{ // Angle - axis(ex : UNIVERSAL ROBOT)
			T rad = euler.norm();
			if (rad < kEpsilonTfMat2)
				euler = Eigen::Matrix<T, 3, 1>(0.f, 0.f, 0.f);
			else
				euler.normalize();
			Eigen::Matrix<T, 3, 3> skew_v = skew(euler);

			rot_mat = Eigen::Matrix<T, 3, 3>::Identity() + sin(rad) * skew_v + (1 - cos(rad)) * (skew_v * skew_v);
		}

		for (unsigned int i = 0; i < 3; ++i)
		{
			for (unsigned int j = 0; j < 3; ++j)
			{
				if (fabs(rot_mat(i, j)) < kEpsilonTfMat2)
					rot_mat(i, j) = 0.f;
			}
		}
	};

	/*
	 * construct transformation matrix
	 * param[in] trans_vec translation vector x, y, z
	 * param[in] quat quaternion x, y, z, w
	 */
	tf_mat(Eigen::Matrix<T, 3, 1> trans_vec, const Eigen::Matrix<T, 4, 1> &quat)
	{
		// Quaternions x, y, z, w
		// Caution - quat of ABB robot: w, x, y, z
		this->trans_vec = trans_vec;
		T qx = quat.x();
		T qy = quat.y();
		T qz = quat.z();
		T qw = quat.w();

		float tx = 2.0f * qx;
		float ty = 2.0f * qy;
		float tz = 2.0f * qz;

		float twx = tx * qw;
		float twy = ty * qw;
		float twz = tz * qw;
		float txx = tx * qx;
		float txy = ty * qx;
		float txz = tz * qx;
		float tyy = ty * qy;
		float tyz = tz * qy;
		float tzz = tz * qz;

		rot_mat(0, 0) = 1.0f - (tyy + tzz);
		rot_mat(0, 1) = txy - twz;
		rot_mat(0, 2) = txz + twy;

		rot_mat(1, 0) = txy + twz;
		rot_mat(1, 1) = 1.0f - (txx + tzz);
		rot_mat(1, 2) = tyz - twx;

		rot_mat(2, 0) = txz - twy;
		rot_mat(2, 1) = tyz + twx;
		rot_mat(2, 2) = 1.0f - (txx + tyy);
	};

	// Get angle(Degree), refer to tf_mat::type
	Eigen::Matrix<T, 3, 1> GetEuler(int type = 0) const
	{
		T r0, r1, r2;

		if (type == TypeEuler::RPY)
		{ // 0 : rpy - fixed frame(ex : FANUC)
			// rpy is same as zyx (only order is reverse)
			if (rot_mat(2, 0) < 1.)
			{
				if (rot_mat(2, 0) > -1.)
				{
					r2 = atan2(rot_mat(1, 0), rot_mat(0, 0)); // z
					r1 = asin(-rot_mat(2, 0));				  // y
					r0 = atan2(rot_mat(2, 1), rot_mat(2, 2)); // x
				}
				else
				{
					r2 = -atan2(-rot_mat(1, 2), rot_mat(1, 1));
					r1 = MS_PI_HALF;
					r0 = 0.;
				}
			}
			else
			{
				r2 = atan2(-rot_mat(1, 2), rot_mat(1, 1));
				r1 = -MS_PI_HALF;
				r0 = 0.;
			}
		}
		else if (type == TypeEuler::ZYX)
		{ // 1 : rz-ry-rx (ex : KUKA)
			// double sy = sqrt(rot_mat(0, 0)*rot_mat(0, 0) + rot_mat(1, 0)*rot_mat(1, 0));

			// bool singlur = (sy < kEpsilonTfMat2);
			if (rot_mat(2, 0) < 1.f)
			{
				if (rot_mat(2, 0) > -1.f)
				{
					r0 = atan2(rot_mat(1, 0), rot_mat(0, 0)); // z
					r1 = asin(-rot_mat(2, 0));				  // y
					r2 = atan2(rot_mat(2, 1), rot_mat(2, 2)); // x
				}
				else
				{
					r0 = -atan2(-rot_mat(1, 2), rot_mat(1, 1));
					r1 = MS_PI_HALF;
					r2 = 0.f;
				}
			}
			else
			{
				r0 = atan2(-rot_mat(1, 2), rot_mat(1, 1));
				r1 = -MS_PI_HALF;
				r2 = 0.f;
			}
		}
		else if (type == TypeEuler::ZYZ)
		{ // 2 : rz-ry-rz (ex : KWSK)
			if (rot_mat(2, 2) < 1.)
			{
				if (rot_mat(2, 2) > -1.)
				{
					r0 = atan2(rot_mat(1, 2), rot_mat(0, 2));
					r1 = acos(rot_mat(2, 2));
					r2 = atan2(rot_mat(2, 1), -rot_mat(2, 0));
				}
				else // r33 == -1 Gimbal Lock
				{
					r0 = -atan2(rot_mat(1, 0), rot_mat(1, 1));
					r1 = MS_PI;
					r2 = 0.f;
				}
			}
			else
			{
				r0 = atan2(rot_mat(1, 0), rot_mat(1, 1));
				r1 = 0.f;
				r2 = 0.f;
			}
		}
		else if (type == TypeEuler::XYZ)
		{ // 3 : rx-ry-rz (ex : STAUBLI)
			if (rot_mat(0, 2) < 1.)
			{
				if (rot_mat(0, 2) > -1.)
				{
					r0 = atan2(-rot_mat(1, 2), rot_mat(2, 2));
					r1 = asin(rot_mat(0, 2));
					r2 = atan2(-rot_mat(0, 1), rot_mat(0, 0));
				}
				else
				{ // rot_mat(0,2) == -1
					r0 = -atan2(rot_mat(1, 0), rot_mat(1, 1));
					r1 = -MS_PI_HALF;
					r2 = 0.;
				}
			}
			else
			{ // rot_mat(0,2) == 1
				r0 = -atan2(rot_mat(1, 0), rot_mat(1, 1));
				r1 = MS_PI_HALF;
				r2 = 0.;
			}
		}
		else if (type == TypeEuler::RODRI)
		{ // 4 : Angle - axis(ex : UNIVERSAL ROBOT)
			T angle = 0.f;

			T trace = rot_mat.trace();
			if (fabs(trace - 3) <= kEpsilonTfMat2)
			{
				r0 = 0.f;
				r1 = 0.f;
				r2 = 1.f;
			}
			else if (fabs(trace + 1) <= kEpsilonTfMat2)
			{
				Eigen::Matrix<T, 3, 1> diag = rot_mat.diagonal();
				if ((diag(0) > diag(1)) && (diag(0) > diag(2)))
				{
					r0 = rot_mat(0, 0) + 1.f;
					r1 = rot_mat(0, 1);
					r2 = rot_mat(0, 2);
				}
				else if (diag(1) > diag(2))
				{
					r0 = rot_mat(1, 0);
					r1 = rot_mat(1, 1) + 1.f;
					r2 = rot_mat(1, 2);
				}
				else
				{
					r0 = rot_mat(2, 0);
					r1 = rot_mat(2, 1);
					r2 = rot_mat(2, 2) + 1.f;
				}
				Eigen::Matrix<T, 3, 1> tmp(r0, r1, r2);
				tmp.normalize();
				angle = MS_PI;
				r0 = angle * tmp.x();
				r1 = angle * tmp.y();
				r2 = angle * tmp.z();
			}
			else
			{
				angle = acos((rot_mat.trace() - 1.f) * 0.5f);
				T n_inv = 1.f / (2.f * sin(angle));
				r0 = angle * (rot_mat(2, 1) - rot_mat(1, 2)) * n_inv;
				r1 = angle * (rot_mat(0, 2) - rot_mat(2, 0)) * n_inv;
				r2 = angle * (rot_mat(1, 0) - rot_mat(0, 1)) * n_inv;
			}
		}

		Eigen::Matrix<T, 3, 1> angles(r0, r1, r2);
		return angles;
	};

	Eigen::Matrix<T, 4, 1> GetQuat() const
	{
		T r0, r1, r2, r3;
		T trace = rot_mat.trace();

		if (trace > kEpsilonTfMat2)
		{
			trace = sqrtf(trace + 1.0f);
			r3 = 0.5f * trace;
			float t = 0.5f / trace;

			r0 = (rot_mat(2, 1) - rot_mat(1, 2)) * t;
			r1 = (rot_mat(0, 2) - rot_mat(2, 0)) * t;
			r2 = (rot_mat(1, 0) - rot_mat(0, 1)) * t;
		}
		else
		{
			if (rot_mat(1, 1) > rot_mat(0, 0))
			{
				if (rot_mat(2, 2) > rot_mat(1, 1))
				{
					float t = sqrtf(rot_mat(2, 2) - rot_mat(0, 0) - rot_mat(1, 1) + 1.f);
					r2 = 0.5f * t;
					t = 0.5f / t;
					r3 = (rot_mat(1, 0) - rot_mat(0, 1)) * t;
					r0 = (rot_mat(0, 2) + rot_mat(2, 0)) * t;
					r1 = (rot_mat(1, 2) + rot_mat(2, 1)) * t;
				}
				else
				{
					float t = sqrtf(rot_mat(1, 1) - rot_mat(2, 2) - rot_mat(0, 0) + 1.f);
					r1 = 0.5f * t;
					t = 0.5f / t;
					r3 = (rot_mat(0, 2) - rot_mat(2, 0)) * t;
					r2 = (rot_mat(2, 1) + rot_mat(1, 2)) * t;
					r0 = (rot_mat(0, 1) + rot_mat(1, 0)) * t;
				}
			}
			else
			{
				if (rot_mat(2, 2) > rot_mat(0, 0))
				{
					float t = sqrtf(rot_mat(2, 2) - rot_mat(0, 0) - rot_mat(1, 1) + 1.f);
					r2 = 0.5f * t;
					t = 0.5f / t;
					r3 = (rot_mat(1, 0) - rot_mat(0, 1)) * t;
					r0 = (rot_mat(0, 2) + rot_mat(2, 0)) * t;
					r1 = (rot_mat(1, 2) + rot_mat(2, 1)) * t;
				}
				else
				{
					float t = sqrtf(rot_mat(0, 0) - rot_mat(1, 1) - rot_mat(2, 2) + 1.f);
					r0 = 0.5f * t;
					t = 0.5f / t;
					r3 = (rot_mat(2, 1) - rot_mat(1, 2)) * t;
					r1 = (rot_mat(1, 0) + rot_mat(0, 1)) * t;
					r2 = (rot_mat(2, 0) + rot_mat(0, 2)) * t;
				}
			}
		}
		Eigen::Matrix<T, 4, 1> quat(r0, r1, r2, r3);
		return quat.normalized();
	};

	tf_mat Inversed() const
	{
		Eigen::Matrix<T, 3, 1> _trans_vec = -this->rot_mat.transpose() * this->trans_vec;
		return tf_mat(this->rot_mat.transpose(), _trans_vec);
	};

	tf_mat operator*(const tf_mat &mat) const
	{
		// mat3d _rot_mat = this->rot_mat * mat.rot_mat;
		Eigen::Matrix<T, 3, 1> _trans_vec = this->rot_mat * mat.trans_vec + this->trans_vec;
		return tf_mat(this->rot_mat * mat.rot_mat, _trans_vec);
	};

	// tf_mat operator/ (tf_mat& mat);
	Eigen::Matrix<T, 3, 1> operator*(const Eigen::Matrix<T, 3, 1> &trans_vec_) const
	{
		Eigen::Matrix<T, 3, 1> result;
		result << this->rot_mat.row(0) * trans_vec_ + this->trans_vec.x(), this->rot_mat.row(1) * trans_vec_ + this->trans_vec.y(), this->rot_mat.row(2) * trans_vec_ + this->trans_vec.z();
		return result;
	};

	cv::Point3f operator*(const cv::Point3f &pt) const
	{
		cv::Point3f result;
		result.x = rot_mat(0, 0) * pt.x + rot_mat(0, 1) * pt.y + rot_mat(0, 2) * pt.z + trans_vec(0);
		result.y = rot_mat(1, 0) * pt.x + rot_mat(1, 1) * pt.y + rot_mat(1, 2) * pt.z + trans_vec(1);
		result.z = rot_mat(2, 0) * pt.x + rot_mat(2, 1) * pt.y + rot_mat(2, 2) * pt.z + trans_vec(2);
		return result;
	};

	// tf_mat operator*(const Eigen::Matrix<T, 4, 4>& e_mat4x4) const
	//{
	//	Eigen::Matrix<T, 4, 4> e_tf;
	//	e_tf.cols[0] = Eigen::Matrix<T, 4, 1>(rot_mat.cols[0].x(), rot_mat.cols[0].y(), rot_mat.cols[0].z(), 0);
	//	e_tf.cols[1] = Eigen::Matrix<T, 4, 1>(rot_mat.cols[1].x(), rot_mat.cols[1].y(), rot_mat.cols[1].z(), 0);
	//	e_tf.cols[2] = Eigen::Matrix<T, 4, 1>(rot_mat.cols[2].x(), rot_mat.cols[2].y(), rot_mat.cols[2].z(), 0);
	//	e_tf.cols[3] = Eigen::Matrix<T, 4, 1>(trans_vec.x(), trans_vec.y(), trans_vec.z(), 1);

	//	Eigen::Matrix<T, 4, 4> e_tfd = e_tf * e_mat4x4;
	//	tf_mat m;
	//	m.rot_mat = e_tfd.block(0, 0, 3, 3);
	//	m.trans_vec = e_tfd.block(0, 3, 3, 1);
	//	return m;
	//}

	static tf_mat<T> Identity()
	{
		return tf_mat<T>();
	}

	void SetIdentity()
	{
		rot_mat.setIdentity();
		trans_vec.setZero();
	}

	friend std::ostream &operator<<(std::ostream &os, const tf_mat &tf_mat)
	{
		// os << std::ios::scientific << endl;
		os.precision(8);

		for (unsigned int i = 0; i < 3; ++i)
		{
			os << (float)tf_mat.rot_mat(i, 0) << " " << (float)tf_mat.rot_mat(i, 1) << " " << (float)tf_mat.rot_mat(i, 2) << " " << (float)tf_mat.trans_vec(i) << std::endl;
		}
		os << "0 0 0 1\n";
		return os;
	};

	// cvt to std::vector
	std::vector<double> Vectorize()
	{
		std::vector<double> result(12);
		for (int i = 0; i < 3; ++i)
		{
			result[3 * i] = rot_mat.col(i).x();
			result[3 * i + 1] = rot_mat.col(i).y();
			result[3 * i + 2] = rot_mat.col(i).z();

			result[9 + i] = trans_vec(i);
		}
		return result;
	};

	Eigen::Matrix<T, 4, 4> toEigenMatrix()
	{
		Eigen::Matrix<T, 4, 4> e_mat;
		// e_mat.setIdentity();
		// e_mat.block<3, 3>(0, 0) = rot_mat;
		// e_mat.block<3, 1>(0, 3) = trans_vec;

		e_mat << rot_mat(0, 0), rot_mat(0, 1), rot_mat(0, 2), trans_vec(0),
			rot_mat(1, 0), rot_mat(1, 1), rot_mat(1, 2), trans_vec(1),
			rot_mat(2, 0), rot_mat(2, 1), rot_mat(2, 2), trans_vec(2),
			0, 0, 0, 1;
		return e_mat;
	}

	// std std::vector to transformation matrix
	// tf_mat(std::vector<double> v_data, bool row_major = false){
	//	if (row_major){
	//		for (int i = 0; i < 3; ++i){
	//			rot_mat.col(i).x() = (T)v_data[i];
	//			rot_mat.col(i).y() = (T)v_data[4 + i];
	//			rot_mat.col(i).z() = (T)v_data[8 + i];

	//			trans_vec(i) = (T)v_data[4 * i + 3];
	//		}
	//	}
	//	else{
	//		for (int i = 0; i < 3; ++i){
	//			rot_mat.col(i).x() = (T)v_data[3 * i];
	//			rot_mat.col(i).y() = (T)v_data[3 * i + 1];
	//			rot_mat.col(i).z() = (T)v_data[3 * i + 2];

	//			trans_vec(i) = (T)v_data[9 + i];
	//		}
	//	}
	//};

	tf_mat(std::vector<double> v_data, bool row_major = false)
	{
		if (row_major)
		{
			for (int i = 0; i < 3; ++i)
			{
				rot_mat.col(i).x() = (T)v_data[i];
				rot_mat.col(i).y() = (T)v_data[4 + i];
				rot_mat.col(i).z() = (T)v_data[8 + i];

				trans_vec(i) = (T)v_data[4 * i + 3];
			}
		}
		else
		{
			for (int i = 0; i < 3; ++i)
			{
				rot_mat.col(i).x() = (T)v_data[3 * i];
				rot_mat.col(i).y() = (T)v_data[3 * i + 1];
				rot_mat.col(i).z() = (T)v_data[3 * i + 2];

				trans_vec(i) = (T)v_data[9 + i];
			}
		}
	};

	void scale(float scale)
	{
		trans_vec *= scale;
	}

	tf_mat<T> scaled(float scale) const
	{
		tf_mat<T> ret = this;
		ret.trans_vec *= scale;
		return ret;
	}
};

#endif
