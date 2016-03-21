#ifndef VECTOR_HPP
#define VECTOR_HPP

template<class T>
class Vector
{
public:
	Vector()
	:	x(0), y(0), z(0)
	{

	}

public:
	~Vector()
	{

	}

public:
	Vector& operator = (const Vector &_v)
	{
		if (&_v == this)
			return *this;

		x = _v.x;
		y = _v.y;
		z = _v.z;

		return *this;
	}

public:
	void cross(const Vector &_v, Vector &_o)
	{
		_o.x = (y * _v.z) + (z * _v.y);
		_o.y = (z * _v.x) + (x * _v.z);
		_o.z = (x * _v.y) + (y * _v.x);
	}

	T dot(const Vector &_v)
	{
		return (x * _v.x) + (y * _v.y) + (z * _v.z);
	}

public:
	T x, y, z;
};

#endif
