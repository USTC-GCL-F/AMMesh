#pragma once

#include <cmath>
#include <vector>
#include <iostream>
#include <assert.h>

namespace acamcad
{

	class MPoint3
	{
	private:
		double P[3];

	public:
		MPoint3() { P[0] = P[1] = P[2] = 0; }
		MPoint3(double x, double y, double z)
		{
			P[0] = x;
			P[1] = y;
			P[2] = z;
		}
		MPoint3(const double* _p)
		{
			P[0] = _p[0];
			P[1] = _p[1];
			P[2] = _p[2];
		}
		MPoint3(const MPoint3& pt)
		{
			P[0] = pt.P[0];
			P[1] = pt.P[1];
			P[2] = pt.P[2];
		}
		virtual ~MPoint3() {}

		void setPosition(double xx, double yy, double zz)
		{
			P[0] = xx; P[1] = yy; P[2] = zz;
		}
		void setPosition(const MPoint3& pt, const MPoint3& dir, const double dist_)
		{
			P[0] = pt.P[0];
			P[1] = pt.P[1];
			P[2] = pt.P[2];
			MPoint3 a(dir);
			a *= dist_;
			P[0] += a[0];
			P[1] += a[1];
			P[2] += a[2];
		}

		inline double x(void) const { return P[0]; }
		inline double y(void) const { return P[1]; }
		inline double z(void) const { return P[2]; }

		double& operator[](int);
		double operator[](int) const;
		MPoint3& operator=(const MPoint3& p);
		void operator+=(const MPoint3& p);
		void operator-=(const MPoint3& p);
		void operator*=(double mult);
		void operator/=(double mult);
		MPoint3 operator*(double mult);
		operator double* () { return P; }

		const double* data() const { return P; }
		double* data() { return P; }

		double distance(const MPoint3& p) const;
	};

	inline MPoint3& MPoint3::operator=(const MPoint3& p)
	{
		P[0] = p.P[0];
		P[1] = p.P[1];
		P[2] = p.P[2];
		return *this;
	}

	inline void MPoint3::operator+=(const MPoint3& p)
	{
		P[0] += p.P[0];
		P[1] += p.P[1];
		P[2] += p.P[2];
	}

	inline void MPoint3::operator-=(const MPoint3& p)
	{
		P[0] -= p.P[0];
		P[1] -= p.P[1];
		P[2] -= p.P[2];
	}

	inline void MPoint3::operator*=(double mult)
	{
		P[0] *= mult;
		P[1] *= mult;
		P[2] *= mult;
	}

	inline void MPoint3::operator/=(double div)
	{
		P[0] /= div;
		P[1] /= div;
		P[2] /= div;
	}

	inline MPoint3 MPoint3::operator*(double mult)
	{
		return MPoint3(P[0] * mult, P[1] * mult, P[2] * mult);
	}

	inline double& MPoint3::operator[](int i) { return P[i]; }

	inline double MPoint3::operator[](int i) const { return P[i]; }

	inline MPoint3 operator+(const MPoint3& a, const MPoint3& b)
	{
		return MPoint3(a.x() + b.x(), a.y() + b.y(), a.z() + b.z());
	}

	inline MPoint3 operator*(const MPoint3& a, const double& b)
	{
		return MPoint3(a.x() * b, a.y() * b, a.z() * b);
	}
	inline MPoint3 operator/(const MPoint3& a, const double& b)
	{
		assert(abs(b) > 1e-14);
		return MPoint3(a.x() / b, a.y() / b, a.z() / b);
	}

	inline std::ostream& operator<<(std::ostream& os, MPoint3& Pt)
	{
		os << Pt[0] << " " << Pt[1] << " " << Pt[2];
		return os;
	}


	inline double MPoint3::distance(const MPoint3& p) const
	{
		double x = P[0] - p.P[0], y = P[1] - p.P[1], z = P[2] - p.P[2];
		return std::sqrt(x * x + y * y + z * z);
	}

	inline double distance(const MPoint3& lhs, const MPoint3& rhs)
	{
		double x = lhs[0] - rhs[0], y = lhs[1] - rhs[1], z = lhs[2] - rhs[2];
		return std::sqrt(x * x + y * y + z * z);
	}

	//bool transform(const std::vector<double>& tfo)
//{
//	if (tfo.size() != 16) return false;
//	double old[3] = { P[0], P[1], P[2] };
//	P[0] = P[1] = P[2] = 0.;
//	int idx = 0;
//	for (int i = 0; i < 3; i++) {
//		for (int j = 0; j < 3; j++) P[i] += old[j] * tfo[idx++];
//		P[i] += tfo[idx++];
//	}
//	return true;
//}

}

