#ifndef TINY_VECDTOR_H
#define TINY_VECDTOR_H

#include <istream>
#include <ostream>
#include <string>
#include <cmath>
#include <cassert>
#include <cfloat>

template <typename T, int N>
class TinyVector
{
public:

	// construction
	TinyVector()
	{
		reset_zero();
	}
	//////////////////////////////////////////////////////////////////////////
	TinyVector(const TinyVector<T, N> &TV)
	{
		copy(TV.data_);
	}
	//////////////////////////////////////////////////////////////////////////
	TinyVector(const T *V)
	{
		copy(V);
	}
	//////////////////////////////////////////////////////////////////////////
	TinyVector(char *s)
	{
		std::istringstream ins(s);

		for (int i = 0; i < N; i++)
		{
			ins >> data_[i];
		}
	}
	//////////////////////////////////////////////////////////////////////////
	TinyVector(const T x, const T y)
	{
		data_[0] = x;
		data_[1] = y;
	}
	//////////////////////////////////////////////////////////////////////////
	TinyVector(const T x, const T y, const T z)
	{
		data_[0] = x;
		data_[1] = y;
		data_[2] = z;
	}
	//////////////////////////////////////////////////////////////////////////
	TinyVector(const T x, const T y, const T z, const T w)
	{
		data_[0] = x;
		data_[1] = y;
		data_[2] = z;
		data_[3] = w;
	}
	//////////////////////////////////////////////////////////////////////////
	TinyVector(const T x, const T y, const T z, const T w, const T u, const T v)
	{
		data_[0] = x;
		data_[1] = y;
		data_[2] = z;
		data_[3] = w;
		data_[4] = u;
		data_[5] = v;
	}
	//////////////////////////////////////////////////////////////////////////
	~TinyVector()
	{
	}
	//////////////////////////////////////////////////////////////////////////
	void reset_zero()
	{
		memset(data_, 0, sizeof (T)*N);
	}
	//////////////////////////////////////////////////////////////////////////
	//access
	inline operator const T *() const
	{
		return data_;
	}
	//////////////////////////////////////////////////////////////////////////
	inline operator T *()
	{
		return data_;
	}
	//////////////////////////////////////////////////////////////////////////
	inline const T &operator() (int i,int j) const
	{
		//ONLY FOR MATRIX3F OR MATRIX3D
		//return data_[i - 1];
		assert(N == 9);
		return data_[i * 3 + j];
	}
	//////////////////////////////////////////////////////////////////////////
	inline T &operator() (int i,int j)
	{
		//ONLY FOR MATRIX3F OR MATRIX3D
		//return data_[i - 1];
		assert(N == 9);
		return data_[i*3+j];
	}
	//////////////////////////////////////////////////////////////////////////
	inline T &operator() (int i)
	{
		//return data_[i - 1];
		return data_[i];
	}
	//////////////////////////////////////////////////////////////////////////
	inline const T &operator() (int i) const
	{
		//return data_[i - 1];
		return data_[i];
	}
	//////////////////////////////////////////////////////////////////////////
	inline T &operator[] (int i)
	{
		return data_[i];
	}
	//////////////////////////////////////////////////////////////////////////
	inline const T &operator[] (int i) const
	{
		return data_[i];
	}
	//////////////////////////////////////////////////////////////////////////
	//assignment
	inline TinyVector<T, N> &operator= (const TinyVector<T, N> &A)
	{
		if (data_ == A.data_)
		{
			return *this;
		}

		copy(A.data_);
		return *this;
	}
	//////////////////////////////////////////////////////////////////////////
	inline TinyVector<T, N> &operator= (const T *A)
	{
		if (data_ == A)
		{
			return *this;
		}

		copy(A);
		return *this;
	}
	////////////////////////////////////////////////////////////////////////////
	inline TinyVector<T, N> &operator= (const T &scalar)
	{
		set(scalar);
		return *this;
	}
	//////////////////////////////////////////////////////////////////////////
	// comparison
	bool operator== (const TinyVector<T, N> &rkV) const
	{
		return CompareArrays(rkV) == 0;
	}
	//////////////////////////////////////////////////////////////////////////
	bool operator!= (const TinyVector<T, N> &rkV) const
	{
		return CompareArrays(rkV) != 0;
	}
	//////////////////////////////////////////////////////////////////////////
	bool operator< (const TinyVector<T, N> &rkV) const
	{
		return CompareArrays(rkV) < 0;
	}
	//////////////////////////////////////////////////////////////////////////
	bool operator<= (const TinyVector<T, N> &rkV) const
	{
		return CompareArrays(rkV) <= 0;
	}
	//////////////////////////////////////////////////////////////////////////
	bool operator> (const TinyVector<T, N> &rkV) const
	{
		return CompareArrays(rkV) > 0;
	}
	//////////////////////////////////////////////////////////////////////////
	bool operator>= (const TinyVector<T, N> &rkV) const
	{
		return CompareArrays(rkV) >= 0;
	}
	//////////////////////////////////////////////////////////////////////////

	// arithmetic operations
	inline TinyVector<T, N> operator+ (const TinyVector<T, N> &rkV) const
	{
		TinyVector<T, N> tmp;

		for (int i = 0; i < N; i++)
		{
			tmp.data_[i] = data_[i] + rkV.data_[i];
		}

		return tmp;
	}
	//////////////////////////////////////////////////////////////////////////
	inline TinyVector<T, N> operator- (const TinyVector<T, N> &rkV) const
	{
		TinyVector<T, N> tmp;

		for (int i = 0; i < N; i++)
		{
			tmp.data_[i] = data_[i] - rkV.data_[i];
		}

		return tmp;
	}
	//////////////////////////////////////////////////////////////////////////
	inline TinyVector<T, N> operator* (T fScalar) const
	{
		TinyVector<T, N> tmp;
		for (int i = 0; i < N; i++)
		{
			tmp.data_[i] = fScalar * data_[i];
		}

		return tmp;
	}
	//////////////////////////////////////////////////////////////////////////
	inline TinyVector<T, N> operator/ (T fScalar) const
	{
		TinyVector<T, N> tmp;

		if (fScalar != (T) 0.0)
		{
			T fInvScalar = ((T) 1.0) / fScalar;

			for (int i = 0; i < N; i++)
			{
				tmp.data_[i] = fInvScalar * data_[i];
			}
		}
		else
		{
			for (int i = 0; i < N; i++)
			{
				tmp.data_[i] = FLT_MAX;
			}
		}

		return tmp;
	}
	//////////////////////////////////////////////////////////////////////////
	inline TinyVector<T, N> operator- () const
	{
		TinyVector<T, N> tmp;

		for (int i = 0; i < N; i++)
		{
			tmp.data_[i] = -data_[i];
		}

		return tmp;
	}
	//////////////////////////////////////////////////////////////////////////
	// arithmetic updates
	inline TinyVector<T, N> &operator+= (const TinyVector<T, N> &rkV)
	{
		for (int i = 0; i < N; i++)
		{
			data_[i] += rkV.data_[i];
		}

		return *this;
	}
	//////////////////////////////////////////////////////////////////////////
	inline TinyVector<T, N> &operator-= (const TinyVector<T, N> &rkV)
	{
		for (int i = 0; i < N; i++)
		{
			data_[i] -= rkV.data_[i];
		}

		return *this;
	}
	//////////////////////////////////////////////////////////////////////////
	inline TinyVector<T, N> &operator*= (T fScalar)
	{
		for (int i = 0; i < N; i++)
		{
			data_[i] *= fScalar;
		}

		return *this;
	}
	//////////////////////////////////////////////////////////////////////////
	inline TinyVector<T, N> &operator/= (T fScalar)
	{
		if (fScalar != (T) 0.0)
		{
			T fInvScalar = ((T) 1.0) / fScalar;

			for (int i = 0; i < N; i++)
			{
				data_[i] *= fInvScalar;
			}
		}
		else
		{
			for (int i = 0; i < N; i++)
			{
				data_[i] = FLT_MAX;
			}
		}

		return *this;
	}
	//////////////////////////////////////////////////////////////////////////
	// vector operations
	inline T Length() const
	{
		return sqrt(SquaredLength());
	}
	//////////////////////////////////////////////////////////////////////////
	inline T SquaredLength() const
	{
		T sum = (T) 0.0;

		for (int i = 0; i < N; i++)
		{
			sum += data_[i] * data_[i];
		}

		return sum;
	}
	//////////////////////////////////////////////////////////////////////////
	inline T Dot(const TinyVector<T, N> &TV) const
	{
		T sum = (T) 0.0;

		for (int i = 0; i < N; i++)
		{
			sum += data_[i] * TV.data_[i];
		}

		return sum;
	}
	//////////////////////////////////////////////////////////////////////////
	TinyVector<T, N> Cross(const TinyVector<T, N> &TV) const
	{
		assert(N > 1);

		TinyVector<T, N> tmp;

		if (N == 2)
		{
			tmp[0] = data_[0] * TV.data_[1] - data_[1] * TV.data_[0];
			tmp[1] = 0;
		}
		else if (N == 3)
		{
			tmp[0] = data_[1] * TV.data_[2] - data_[2] * TV.data_[1];
			tmp[1] = data_[2] * TV.data_[0] - data_[0] * TV.data_[2];
			tmp[2] = data_[0] * TV.data_[1] - data_[1] * TV.data_[0];
		}
		else if (N > 3)
		{
			for (int i = 0; i < N; i++)
			{
				int id1 = (i + 1) % N;
				int id2 = (i + 2) % N;
				tmp[i] = data_[id1] * TV.data_[id2] - data_[id2] * TV.data_[id1];
			}
		}

		return tmp;
	}
	///////////////////////////////////////////////////////////////////////////
	T Det() const
	{
		assert(N == 9);
		T tmp = 0;
		tmp = data_[0]*data_[4]*data_[8];
		tmp += data_[1]*data_[5]*data_[6];
		tmp += data_[2]*data_[3]*data_[7];
		tmp -= data_[2]*data_[4]*data_[6];
		tmp -= data_[1]*data_[3]*data_[8];
		tmp -= data_[0]*data_[5]*data_[7];
		return tmp;
	}
	///////////////////////////////////////////////////////////////////////////
	T Trace() const
	{
		assert(N == 9);
		T tmp = 0;
		tmp = data_[0] + data_[4] + data_[8];
		return tmp;
	}
	//////////////////////////////////////////////////////////////////////////
	TinyVector<T, N> UnitCross(const TinyVector<T, N> &TV) const
	{
		return Cross(TV).GetNormalized();
	}
	//////////////////////////////////////////////////////////////////////////
	inline T Normalize()
	{
		T fLength = Length();

		if (fLength != (T) 0.0)
		{
			T fInvLength = ((T) 1.0) / fLength;

			for (int i = 0; i < N; i++)
			{
				data_[i] *= fInvLength;
			}
		}
		else
		{
			fLength = (T) 0.0;
			set((T) 0.0);
		}

		return fLength;
	}
	//////////////////////////////////////////////////////////////////////////
	inline TinyVector<T, N> GetNormalized()
	{
		T fLength = Length();

		TinyVector<T, N> tmp;

		if (fLength != 0)
		{
			T fInvLength = ((T) 1.0) / fLength;

			for (int i = 0; i < N; i++)
			{
				tmp.data_[i] = fInvLength * data_[i];
			}
		}
		else
		{
			for (int i = 0; i < N; i++)
			{
				tmp.data_[i] = (T) 0.0;
			}
		}

		return tmp;
	}

	//////////////////////////////////////////////////////////////////////////
	static void ComputeExtremes(int iVQuantity, const TinyVector<T, N> *akPoint,
		TinyVector<T, N> &rkMin, TinyVector<T, N> &rkMax)
	{
		assert(iVQuantity > 0 && akPoint);

		rkMin = akPoint[0];
		rkMax = rkMin;

		for (int i = 1; i < iVQuantity; i++)
		{
			const TinyVector<T, N> &rkPoint = akPoint[i];

			for (int j = 0; j < N; j++)
			{
				if (rkPoint[j] < rkMin[j])
				{
					rkMin[j] = rkPoint[j];
				}
				else if (rkPoint[j] > rkMax[j])
				{
					rkMax[j] = rkPoint[j];
				}
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////

private:
	//////////////////////////////////////////////////////////////////////////
	void copy(const T *V)
	{
		memcpy(data_, V, sizeof (T)*N);
	}
	//////////////////////////////////////////////////////////////////////////
	void set(const T &scalar)
	{
		for (int i = 0; i < N; i++)
		{
			data_[i] = scalar;
		}
	}
	//////////////////////////////////////////////////////////////////////////
	int CompareArrays(const TinyVector<T, N> &rkV) const
	{
		return memcmp(data_, rkV.data_, N * sizeof (T));
	}
	//////////////////////////////////////////////////////////////////////////
private:
	T data_[N];
};

typedef TinyVector<double, 3> Vector3d;
typedef TinyVector<double, 2> Vector2d;
typedef TinyVector<double, 6> Vector6d;
typedef TinyVector<float, 3> Vector3f;
typedef TinyVector<float, 2> Vector2f;
typedef TinyVector<float, 6> Vector6f;
typedef TinyVector<float, 9> Matrix3f;
typedef TinyVector<double, 9> Matrix3d;
typedef TinyVector<float, 4> Matrix2f;
typedef TinyVector<double, 4> Matrix2d;

//////////////////////////////////////////////////////////////////////////
// arithmetic operations
template <typename T, int N>
TinyVector<T, N> operator* (T fScalar, const TinyVector<T, N> &rkV)
{
	TinyVector<T, N> tmp;

	for (int i = 0; i < N; i++)
	{
		tmp[i] = fScalar * rkV[i];
	}

	return tmp;
}
//////////////////////////////////////////////////////////////////////////
template <typename T, int N>
T operator* (const TinyVector<T, N> &rkU, const TinyVector<T, N> &rkV)
{
	assert(N != 9);
	return rkU.Dot(rkV);
}
//////////////////////////////////////////////////////////////////////////

template <typename T, int N>
std::ostream &operator<< (std::ostream &s, const TinyVector<T, N> &A)
{
	for (int i = 0; i < N - 1; i++)
	{
		s << A[i] << " ";
	}

	s << A[N - 1]; // <<"\n";
	return s;
}
//////////////////////////////////////////////////////////////////////////
template <typename T, int N>
std::istream &operator>> (std::istream &s, TinyVector<T, N> &A)
{
	for (int i = 0; i < N; i++)
	{
		s >> A[i];
	}

	return s;
}
//////////////////////////////////////////////////////////////////////////
template <typename T>
T dot(int N, const T *vec_x, const T *vec_y)
{
	T sum = 0;

	for (int i = 0; i < N; i++)
	{
		sum += vec_x[i] * vec_y[i];
	}

	return sum;
}
//////////////////////////////////////////////////////////////////////////
#endif
