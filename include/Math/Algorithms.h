#pragma once
#include "MVector3.h"

namespace acamcad
{

inline double distPointLineSquared(const MVector3& p, const MVector3& v0, const MVector3& v1, MVector3& nearestPoint)
{
	MVector3 d1(p - v0), d2(v1 - v0);

	double t = dot(d1, d2) / d2.normSq();

	if (t > 1.0)
	{
		nearestPoint = v1;
	}
	else if (t > 0.0)
	{
		nearestPoint = v0 + d2 * t;
	}
	else
	{
		nearestPoint = v0;
	}

	d1 = p - nearestPoint;
	return d1.normSq();
}

inline double distPointTriangleSquared(const MVector3& p,
    const MVector3& v0, const MVector3& v1, const MVector3&v2, MVector3& nearestPoint)
{
    MVector3 v0v1 = v1 - v0;
    MVector3 v0v2 = v2 - v0;
    MVector3 n = v0v1.cross(v0v2); // not normalized !
    double d = n.normSq();

    // Check if the triangle is degenerated
    if (d < FLT_MIN && d > -FLT_MIN) {
            const double l0 = v0v1.normSq();
            const double l1 = v0v2.normSq();
            const double l2 = (v2 - v1).normSq();
            if (l0 > l1 && l0 > l2) {
                return distPointLineSquared(p, v0, v1, nearestPoint);
            }
            else if (l1 > l0 && l1 > l2) {
                return distPointLineSquared(p, v0, v2, nearestPoint);
            }
            else {
                return distPointLineSquared(p, v1, v2, nearestPoint);
            }
    }
    double invD = double(1.0) / d;

    // these are not needed for every point, should still perform
    // better with many points against one triangle
    MVector3 v1v2 = v2 - v1;
    double inv_v0v2_2 = double(1.0) / v0v2.normSq();
    double inv_v0v1_2 = double(1.0) / v0v1.normSq();
    double inv_v1v2_2 = double(1.0) / v1v2.normSq();

    MVector3 v0p = p - v0;
    MVector3 t = v0p.cross(n);              //投影到平面并依法向旋转90度
    double s01, s02, s12;
    double a = (t.dot(v0v2)) * -invD;       //投影点与v0v2的关系，a<0， 在v0v2外面
    double b = (t.dot(v0v1)) * invD;        //投影点与v0v1的关系，b<0， 在v0v1外面


    if (a < 0)
    {
        // Calculate the distance to an edge or a corner vertex
        s02 = (v0v2 | v0p) * inv_v0v2_2;
        if (s02 < 0.0)
        {
            s01 = (v0v1 | v0p) * inv_v0v1_2;
            if (s01 <= 0.0) {
                v0p = v0;
            }
            else if (s01 >= 1.0) {
                v0p = v1;
            }
            else {
                v0p = v0 + v0v1 * s01;
            }
        }
        else if (s02 > 1.0) 
        {
            s12 = (v1v2 | (p - v1)) * inv_v1v2_2;
            if (s12 >= 1.0) {
                v0p = v2;
            }
            else if (s12 <= 0.0) {
                v0p = v1;
            }
            else {
                v0p = v1 + v1v2 * s12;
            }
        }
        else {
            v0p = v0 + v0v2 * s02;
        }
    }
    else if (b < 0.0) {
        // Calculate the distance to an edge or a corner vertex
        s01 = (v0v1 | v0p) * inv_v0v1_2;
        if (s01 < 0.0)
        {
            s02 = (v0v2 | v0p) * inv_v0v2_2;
            if (s02 <= 0.0) {
                v0p = v0;
            }
            else if (s02 >= 1.0) {
                v0p = v2;
            }
            else {
                v0p = v0 + v0v2 * s02;
            }
        }
        else if (s01 > 1.0) 
        {
            s12 = (v1v2 | (p - v1)) * inv_v1v2_2;
            if (s12 >= 1.0) {
                v0p = v2;
            }
            else if (s12 <= 0.0) {
                v0p = v1;
            }
            else {
                v0p = v1 + v1v2 * s12;
            }
        }
        else {
            v0p = v0 + v0v1 * s01;
        }
    }
    else if (a + b > 1.0) {
        // Calculate the distance to an edge or a corner vertex
        s12 = (v1v2 | (p - v1)) * inv_v1v2_2;
        if (s12 >= 1.0) 
        {
            s02 = (v0v2 | v0p) * inv_v0v2_2;
            if (s02 <= 0.0) {
                v0p = v0;
            }
            else if (s02 >= 1.0) {
                v0p = v2;
            }
            else {
                v0p = v0 + v0v2 * s02;
            }
        }
        else if (s12 <= 0.0) 
        {
            s01 = (v0v1 | v0p) * inv_v0v1_2;
            if (s01 <= 0.0) {
                v0p = v0;
            }
            else if (s01 >= 1.0) {
                v0p = v1;
            }
            else {
                v0p = v0 + v0v1 * s01;
            }
        }
        else {
            v0p = v1 + v1v2 * s12;
        }
    }
    else {
        // Calculate the distance to an interior point of the triangle
        nearestPoint = p - n * ((n | v0p) * invD);
        return (nearestPoint - p).normSq();
    }

    nearestPoint = v0p;
    return (nearestPoint - p).normSq();
}

//线段与线段的最近点
inline double distLineLineSquared(const MVector3& _v00, const MVector3& _v01,
	const MVector3& _v10, const MVector3& _v11,
	MVector3& _min_v0, MVector3& _min_v1)
{
	MVector3 kDiff = _v00 - _v10;
	MVector3 d0 = _v01 - _v00;
	MVector3 d1 = _v11 - _v10;

	double fA00 = d0.normSq();
	double fA01 = -(d0 | d1);
	double fA11 = d1.normSq();
	double fB0 = (kDiff | d0);
	double fC = kDiff.normSq();
	double fDet = fabs(fA00 * fA11 - fA01 * fA01);
	double fB1, fS, fT, fSqrDist, fTmp;

	// fB1 = -(kDiff | d1); 
	// [fA00 fA01][s] = [-fB0]		s = (fA01 * fB1 - fA11 * fB0)/fDet
	// [fA01 FA11][t] = [-fB1]		s = (fA01 * fB0 - fA00 * fB1)/fDet

	if (fDet >= FLT_MIN)
	{
		// line segments are not parallel
		fB1 = -(kDiff | d1);
		fS = fA01 * fB1 - fA11 * fB0;
		fT = fA01 * fB0 - fA00 * fB1;


		if (fS >= 0.0)
		{
			if (fS <= fDet)
			{
				if (fT >= 0.0)
				{
					if (fT <= fDet)  // region 0 (interior)
					{
						// minimum at two interior points of 3D lines
						double fInvDet = 1.0 / fDet;
						fS *= fInvDet;
						fT *= fInvDet;
						fSqrDist = fS * (fA00 * fS + fA01 * fT + 2.0 * fB0) +
							fT * (fA01 * fS + fA11 * fT + 2.0 * fB1) + fC;
					}
					else  // region 3 (side)
					{
						fT = 1.0;
						fTmp = fA01 + fB0;
						if (fTmp >= 0.0)
						{
							fS = 0.0;
							fSqrDist = fA11 + 2.0 * fB1 + fC;
						}
						else if (-fTmp >= fA00)
						{
							fS = 1.0;
							fSqrDist = fA00 + fA11 + fC + 2.0 * (fB1 + fTmp);
						}
						else
						{
							fS = -fTmp / fA00;
							fSqrDist = fTmp * fS + fA11 + 2.0 * fB1 + fC;
						}
					}
				}
				else  // region 7 (side)
				{
					fT = 0.0;
					if (fB0 >= 0.0)
					{
						fS = 0.0;
						fSqrDist = fC;
					}
					else if (-fB0 >= fA00)
					{
						fS = 1.0;
						fSqrDist = fA00 + 2.0 * fB0 + fC;
					}
					else
					{
						fS = -fB0 / fA00;
						fSqrDist = fB0 * fS + fC;
					}
				}
			}
			else
			{
				if (fT >= 0.0)
				{
					if (fT <= fDet)  // region 1 (side)
					{
						fS = 1.0;
						fTmp = fA01 + fB1;
						if (fTmp >= 0.0)
						{
							fT = 0.0;
							fSqrDist = fA00 + 2.0 * fB0 + fC;
						}
						else if (-fTmp >= fA11)
						{
							fT = 1.0;
							fSqrDist = fA00 + fA11 + fC + 2.0 * (fB0 + fTmp);
						}
						else
						{
							fT = -fTmp / fA11;
							fSqrDist = fTmp * fT + fA00 + 2.0 * fB0 + fC;
						}
					}
					else  // region 2 (corner)
					{
						fTmp = fA01 + fB0;
						if (-fTmp <= fA00)
						{
							fT = 1.0;
							if (fTmp >= 0.0)
							{
								fS = 0.0;
								fSqrDist = fA11 + 2.0 * fB1 + fC;
							}
							else
							{
								fS = -fTmp / fA00;
								fSqrDist = fTmp * fS + fA11 + 2.0 * fB1 + fC;
							}
						}
						else
						{
							fS = 1.0;
							fTmp = fA01 + fB1;
							if (fTmp >= 0.0)
							{
								fT = 0.0;
								fSqrDist = fA00 + 2.0 * fB0 + fC;
							}
							else if (-fTmp >= fA11)
							{
								fT = 1.0;
								fSqrDist = fA00 + fA11 + fC + 2.0 * (fB0 + fTmp);
							}
							else
							{
								fT = -fTmp / fA11;
								fSqrDist = fTmp * fT + fA00 + 2.0 * fB0 + fC;
							}
						}
					}
				}
				else  // region 8 (corner)
				{
					if (-fB0 < fA00)
					{
						fT = 0.0;
						if (fB0 >= 0.0)
						{
							fS = 0.0;
							fSqrDist = fC;
						}
						else
						{
							fS = -fB0 / fA00;
							fSqrDist = fB0 * fS + fC;
						}
					}
					else
					{
						fS = 1.0;
						fTmp = fA01 + fB1;
						if (fTmp >= 0.0)
						{
							fT = 0.0;
							fSqrDist = fA00 + 2.0 * fB0 + fC;
						}
						else if (-fTmp >= fA11)
						{
							fT = 1.0;
							fSqrDist = fA00 + fA11 + fC + 2.0 * (fB0 + fTmp);
						}
						else
						{
							fT = -fTmp / fA11;
							fSqrDist = fTmp * fT + fA00 + 2.0 * fB0 + fC;
						}
					}
				}
			}
		}
		else
		{
			if (fT >= 0.0)
			{
				if (fT <= fDet)  // region 5 (side)
				{
					fS = 0.0;
					if (fB1 >= 0.0)
					{
						fT = 0.0;
						fSqrDist = fC;
					}
					else if (-fB1 >= fA11)
					{
						fT = 1.0;
						fSqrDist = fA11 + 2.0 * fB1 + fC;
					}
					else
					{
						fT = -fB1 / fA11;
						fSqrDist = fB1 * fT + fC;
					}
				}
				else  // region 4 (corner)
				{
					fTmp = fA01 + fB0;
					if (fTmp < 0.0)
					{
						fT = 1.0;
						if (-fTmp >= fA00)
						{
							fS = 1.0;
							fSqrDist = fA00 + fA11 + fC + 2.0 * (fB1 + fTmp);
						}
						else
						{
							fS = -fTmp / fA00;
							fSqrDist = fTmp * fS + fA11 + 2.0 * fB1 + fC;
						}
					}
					else
					{
						fS = 0.0;
						if (fB1 >= 0.0)
						{
							fT = 0.0;
							fSqrDist = fC;
						}
						else if (-fB1 >= fA11)
						{
							fT = 1.0;
							fSqrDist = fA11 + 2.0 * fB1 + fC;
						}
						else
						{
							fT = -fB1 / fA11;
							fSqrDist = fB1 * fT + fC;
						}
					}
				}
			}
			else   // region 6 (corner)
			{
				if (fB0 < 0.0)
				{
					fT = 0.0;
					if (-fB0 >= fA00)
					{
						fS = 1.0;
						fSqrDist = fA00 + 2.0 * fB0 + fC;
					}
					else
					{
						fS = -fB0 / fA00;
						fSqrDist = fB0 * fS + fC;
					}
				}
				else
				{
					fS = 0.0;
					if (fB1 >= 0.0)
					{
						fT = 0.0;
						fSqrDist = fC;
					}
					else if (-fB1 >= fA11)
					{
						fT = 1.0;
						fSqrDist = fA11 + 2.0 * fB1 + fC;
					}
					else
					{
						fT = -fB1 / fA11;
						fSqrDist = fB1 * fT + fC;
					}
				}
			}
		}
	}
	else
	{
		// line segments are parallel
		if (fA01 > 0.0)
		{
			// direction vectors form an obtuse angle
			if (fB0 >= 0.0)
			{
				fS = 0.0;
				fT = 0.0;
				fSqrDist = fC;
			}
			else if (-fB0 <= fA00)
			{
				fS = -fB0 / fA00;
				fT = 0.0;
				fSqrDist = fB0 * fS + fC;
			}
			else
			{
				fB1 = -(kDiff | d1);
				fS = 1.0;
				fTmp = fA00 + fB0;
				if (-fTmp >= fA01)
				{
					fT = 1.0;
					fSqrDist = fA00 + fA11 + fC + 2.0 * (fA01 + fB0 + fB1);
				}
				else
				{
					fT = -fTmp / fA01;
					fSqrDist = fA00 + 2.0 * fB0 + fC + fT * (fA11 * fT + 2.0 * (fA01 + fB1));
				}
			}
		}
		else
		{
			// direction vectors form an acute angle
			if (-fB0 >= fA00)
			{
				fS = 1.0;
				fT = 0.0;
				fSqrDist = fA00 + 2.0 * fB0 + fC;
			}
			else if (fB0 <= 0.0)
			{
				fS = -fB0 / fA00;
				fT = 0.0;
				fSqrDist = fB0 * fS + fC;
			}
			else
			{
				fB1 = -(kDiff | d1);
				fS = 0.0;
				if (fB0 >= -fA01)
				{
					fT = 1.0;
					fSqrDist = fA11 + 2.0 * fB1 + fC;
				}
				else
				{
					fT = -fB0 / fA01;
					fSqrDist = fC + fT * (2.0 * fB1 + fA11 * fT);
				}
			}
		}
	}


	_min_v0 = _v00 + fS * d0;
	_min_v1 = _v10 + fT * d1;

	return fabs(fSqrDist);
}

//线段与直线的最近点，直线用两个点表示，后面俩个参数是直线
inline double distLine_SLineSquared(const MVector3& _v00, const MVector3& _v01,
	const MVector3& _v1, const MVector3& _v11,
	MVector3& _min_v0, MVector3& _min_v1)
{
	MVector3 kDiff = _v00 - _v1;
	MVector3 d0 = _v01 - _v00;
	MVector3 d1 = _v11 - _v1;

	double fA00 = d0.normSq();
	double fA01 = -(d0 | d1);
	double fA11 = d1.normSq();
	double fB0 = (kDiff | d0);
	double fC = kDiff.normSq();
	double fDet = fabs(fA00 * fA11 - fA01 * fA01);
	double fB1, fS, fT, fSqrDist, fTmp;

	// fB1 = -(kDiff | d1); 
	// [fA00 fA01][s] = [-fB0]		s = (fA01 * fB1 - fA11 * fB0)/fDet
	// [fA01 FA11][t] = [-fB1]		s = (fA01 * fB0 - fA00 * fB1)/fDet

	if (fDet >= FLT_MIN)
	{
		// line segments are not parallel
		fB1 = -(kDiff | d1);
		fS = fA01 * fB1 - fA11 * fB0;
		fT = fA01 * fB0 - fA00 * fB1;

		if (fS >= 0.0)
		{
			if (fS <= fDet)
			{
				double fInvDet = 1.0 / fDet;
				fS *= fInvDet;
				fT *= fInvDet;
			}
			else
			{
				double fInvDet = 1.0 / fDet;
				fT *= fInvDet;
				fS = 1.0;
			}
		}
		else
		{
			double fInvDet = 1.0 / fDet;
			fT *= fInvDet;
			fS = 0.0;
		}
	}
	else
	{
		fB1 = (kDiff | d1);

		fS = 0;
		fT = fB1 / fA11;

	}

	_min_v0 = _v00 + fS * d0;
	_min_v1 = _v1 + fT * d1;
	fSqrDist = (_min_v1 - _min_v0).normSq();
	return fabs(fSqrDist);
}

//直线与平面的交点
inline bool intersection_Line_Plane(const MVector3& v, const MVector3& t, const MVector3& o, const MVector3& n, MVector3& pi)
{
	double tn = t.dot(n);
	if (abs(tn) < 0.1)
	{
		return false;
	}

	MVector3 vp = v - o;
	double opn = vp.dot(n);
	double s = -1 * opn / tn;
	pi = v + s * t;
	return true;
}

//判断一个点在一个面的法向面还是非法向面
inline bool pointPlane(const MVector3& p, const MVector3& v, MVector3& n)
{
	MVector3 vp = p - v;
	if (vp.dot(n) > 0)
		return true;
	else
		return false;
}

inline bool pointPlane(const MVector3& p, const MVector3& v, const MVector3& d1, const MVector3& d2)
{
	MVector3 n = d1.cross(d2); n.normalize();
	return pointPlane(p,v,n);
}

//点是否在一个无限远的锥内，v是锥的起点，t是锥的方向， 注意t的顺序，法向向内。
// 0 1
// 3 2
inline bool pointPyramid(const MVector3& p, const MVector3& v, const std::vector<MVector3>& t)
{
	if (t.size() < 3) return false;

	int t_size = t.size();
	for (int i = 0; i < t.size(); i++)
	{
		int i1 = (i + 1) % t_size;
		bool is_in = pointPlane(p, v, t[i], t[i1]);
		if (!is_in)
			return false;
	}
	return true;
}

}

