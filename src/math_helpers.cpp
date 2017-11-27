/******************************************************************************
 * Quadcopter-Library-v1
 * math_helpers.cpp
 *
 * This file contains the Vector and Quaternion class which represent vectors
 * with respectively three and four entries. These classes also contain the
 * general mathematical operations and some special Quaternion related operations.
 *
 * @author Rob Mertens
 * @version 1.0.1 14/08/2016
 ******************************************************************************/

#include "math_helpers.hpp"

namespace qc
{

namespace math_helpers
{

/**
 * @brief Class Vector
 */
/** Constructors/destructors/overloading **************************************/
Vector::Vector(void)
{
	x_ = 0.0;
	y_ = 0.0;
	z_ = 0.0;
}

Vector::Vector(const double x, const double y, const double z)
{
	x_ = x;
	y_ = y;
	z_ = z;
}

Vector::Vector(const Vector& other)
{
	if(this != &other)
	{
		x_ = other.x_;
		y_ = other.y_;
		z_ = other.z_;
	}
}

Vector::~Vector(void) { ;; }

Vector& Vector::operator=(const Vector& other)
{
	if(this != &other)
	{
		x_ = other.x_;
		y_ = other.y_;
		z_ = other.z_;

	}
	return (*this);
}

/** Primitives ****************************************************************/
static Vector Vector::Zero(void)
{
	return Vector(0.0, 0.0, 0.0);
}

static Vector Vector::UnitX(void)
{
	return Vector(1.0, 0.0, 0.0);
}

static Vector Vector::UnitY(void)
{
	return Vector(0.0, 1.0, 0.0);
}

static Vector Vector::UnitZ(void)
{
	return Vector(0.0, 0.0, 1.0);
}

/** Member access *************************************************************/
void Vector::x(const double x)
{
	x_ = x;
}

void Vector::y(const double y)
{
	y_ = y;
}

void Vector::z(const double z)
{
	z_ = z;
}

double Vector::x(void)
{
	return x_;
}

double Vector::y(void)
{
	return y_;
}

double Vector::z(void)
{
	return z_;
}

double Vector::magnitude(void)
{
	return sqrt(x_*x_ + y_*y_ + z_*z_);
}

void Vector::normalize(void)
{
	double m = magnitude();
	x_ /= m;
	x_ /= m;
	x_ /= m;
}

/** Vector operations *********************************************************/
Vector& Vector::operator+=(const Vector& v)
{
	this->x(this->x() + v.x());
	this->y(this->y() + v.y());
	this->z(this->z() + v.z());
	return (*this);
}

Vector& Vector::operator-=(const Vector& v)
{
	this->x(this->x() - v.x());
	this->y(this->y() - v.y());
	this->z(this->z() - v.z());
	return (*this);
}

Vector& Vector::operator*=(const double scalar)
{
	this->x(this->x()*scalar);
	this->y(this->y()*scalar);
	this->z(this->z()*scalar);
	return (*this);
}

Vector& Vector::operator/=(const double scalar)
{
	this->x(this->x()/scalar);
	this->y(this->y()/scalar);
	this->z(this->z()/scalar);
	return (*this);
}

Vector operator+(const Vector& v, const Vector& w)
{
	Vector res(v);
	res += w;
	return res;
}

Vector operator-(const Vector& v, const Vector& w)
{
	Vector res(v);
	res -= w;
	return res;
}

Vector operator*(const Vector& v, const double scalar)
{
	Vector res(v);
	res *= scalar;
	return res;
}

Vector operator*(const double scalar, const Vector& v)
{
	Vector res(v);
	res *= scalar;
	return res;
}

Vector operator/(const Vector& v, const double scalar)
{
	Vector res(v);
	res /= scalar;
	return res;
}

double dot(const Vector& v, const Vector& w)
{
	return (v.x()*w.x() + v.y()*w.y() + v.z()*w.z());
}

Vector cross(const Vector& v, const Vector& w)
{
	Vector res(
		v.y()*w.z() - v.z()*w.y(),
		v.z()*w.x() - v.x()*w.z(),
		v.x()*w.y() - v.y()*w.x());
	return res;
}

/**
 * @brief Quaternion class.
 */
/** Constructors/destructors/overloading **************************************/
Quaternion::Quaternion(void)
{
	w_ = 1.0f;
	x_ = 0.0f;
	y_ = 0.0f;
	z_ = 0.0f;
}

Quaternion::Quaternion(const Vector& e, const double a)
{
	w_ = cos(0.5*a);
	x_ = (e.x())*sin(0.5*a);
	y_ = (e.y())*sin(0.5*a);
	z_ = (e.z())*sin(0.5*a);
}

Quaternion::Quaternion(const double w, const double x, const double y,
	const double z)
{
	w_ = w;
	x_ = x;
	y_ = y;
	z_ = z;
}

Quaternion::Quaternion(const Quaternion& other)
{
	if(this != &other)
	{
		w_ = other.w_;
		x_ = other.x_;
		y_ = other.y_;
		z_ = other.z_;
	}
}

Quaternion::~Quaternion(void) { ;; }

Quaternion& Quaternion::operator=(const Quaternion& other)
{
	if(this != &other)
	{
		w_ = other.w_;
		x_ = other.x_;
		y_ = other.y_;
		z_ = other.z_;
	}
	return (*this);
}

/** Primitives ****************************************************************/
static Quaternion Quaternion::Zero(void)
{
	return Quaternion(0.0, 0.0, 0.0, 0.0);
}

static Quaternion Quaternion::UnitW(void)
{
	return Quaternion(1.0, 0.0, 0.0, 0.0);
}

static Quaternion Quaternion::UnitX(void)
{
	return Quaternion(0.0, 1.0, 0.0, 0.0);
}

static Quaternion Quaternion::UnitY(void)
{
	return Quaternion(0.0, 0.0, 1.0, 0.0);
}

static Quaternion Quaternion::UnitZ(void)
{
	return Quaternion(0.0, 0.0, 0.0, 1.0);
}

/** Member access *************************************************************/
void Quaternion::w(const double w)
{
	w_ = w;
}

void Quaternion::x(const double x)
{
	x_ = x;
}

void Quaternion::y(const double y)
{
	y_ = y;
}

void Quaternion::z(const double z)
{
	z_ = z;
}

double Quaternion::w(void)
{
	return w_;
}

double Quaternion::x(void)
{
	return x_;
}

double Quaternion::y(void)
{
	return y_;
}

double Quaternion::z(void)
{
	return z_;
}

double Quaternion::magnitude(void)
{
	return sqrt(w_*w_ + x_*x_ + y_*y_ + z_*z_);
}

/** Quaternion operators ******************************************************/
void Quaternion::normalize(void)
{
	double m = magnitude();
	(*this) /= m;
}

void Quaternion::conjugate(void)
{
	///w_ remains unchanged.
	x_ *= (-1.0f);
	y_ *= (-1.0f);
	z_ *= (-1.0f);
}

void Quaternion::inverse(void)
{
	double m = magnitude();
	w_ /= m;
	x_ /= (-1.0f*m);
	y_ /= (-1.0f*m);
	z_ /= (-1.0f*m);
}

Vector Quaternion::rotate(const Vector& v)
{
	Vector res(
		((this->w()*this->w() + this->x()*this->x()
			- this->y()*this->y() - this->z()*this->z())*v.x()
			+ 2*(this->x()*this->y() - this->w()*this->z())*v.y()
			+ 2*(this->x()*this->z() + this->w()*this->y())*v.z()),
		(2*(this->x()*this->y() + this->w()*this->z())*v.x()
			+ (this->w()*this->w() - this->x()*this->x()
			+ this->y()*this->y() - this->z()*this->z())*v.y()
			+ 2*(this->y()*this->z() - this->w()*this->x())*v.z()),
		(2*(this->x()*this->z() - this->w()*this->y())*v.x()
			+ 2*(this->y()*this->z() + this->w()*this->x())*v.y()
			+ (this->w()*this->w() - this->x()*this->x()
			- this->y()*this->y() + this->z()*this->z())*v.z()));
	return res;
}

Quaternion& Quaternion::operator+=(const Quaternion& q)
{
	this->w(this->w_ + q.w());
	this->x(this->x_ + q.x());
	this->y(this->y_ + q.y());
	this->z(this->z_ + q.z());
	return (*this);
}

Quaternion& Quaternion::operator-=(const Quaternion& q)
{
	this->w(this->w_ - q.w());
	this->x(this->x_ - q.x());
	this->y(this->y_ - q.y());
	this->z(this->z_ - q.z());
	return (*this);
}

Quaternion& Quaternion::operator*=(const double scalar)
{
	this->w(this->w_*scalar);
	this->x(this->x_*scalar);
	this->y(this->y_*scalar);
	this->z(this->z_*scalar);
	return (*this);
}

Quaternion& Quaternion::operator/=(const double scalar)
{
	this->w(this->w_/scalar);
	this->x(this->x_/scalar);
	this->y(this->y_/scalar);
	this->z(this->z_/scalar);
	return (*this);
}

Quaternion operator+(const Quaternion& q, const Quaternion& p)
{
	Quaternion res(q);
	res += p;
	return res;
}

Quaternion operator-(const Quaternion& q, const Quaternion& p)
{
	Quaternion res(q);
	res -= p;
	return res;
}

Quaternion operator*(const Quaternion& q, const double scalar)
{
	Quaternion res(q);
	res *= scalar;
	return res;
}

Quaternion operator*(const double scalar, const Quaternion& q)
{
	Quaternion res(q);
	res *= scalar;
	return res;
}

Quaternion operator/(const Quaternion& q, const double scalar)
{
	Quaternion res(q);
	res /= scalar;
	return res;
}

double dot(const Quaternion& q, const Quaternion& p)
{
	return (q.w()*p.w() + q.x()*p.x() + q.y()*p.y() + q.z()*p.z());
}

Quaternion hamilton(const Quaternion& q, const Quaternion& p)
{
	Quaternion res(
		q.w()*p.w() - q.x()*p.x() - q.y()*p.y() - q.z()*p.z(),
		q.w()*p.x() + q.x()*p.w() + q.y()*p.z() - q.z()*p.y(),
		q.w()*p.y() - q.x()*p.z() + q.y()*p.w() + q.z()*p.x(),
 		q.w()*p.z() + q.x()*p.y() - q.y()*p.x() + q.z()*p.w());
	return res;
}

Quaternion lerp(const Quaternion& q, const Quaternion& p,
	const double alpha)
{
	Quaternion res;
	res = q*(1 - alpha) + p*alpha;
	res.normalize();
	return res;
}

Quaternion slerp(const Quaternion& q, const Quaternion& p,
	const double alpha)
{
	/*Quaternion res;
	res = q*(1 - alpha) + p*alpha;
	res.normalize();
	return res;*/
	return Quaternion::Zero();
}

Quaternion normalize(const Quaternion& q)
{
	Quaternion res;
	q.normalize();
	res = q;
	return res;
}

Quaternion conjugate(const Quaternion& q)
{
	Quaternion res;
	q.conjugate();
	res = q;
	return res;
}

/**
 * @brief External math functions.
 */
Vector q2euler(const Quaternion& q)
{
	Vector res(
		atan2(2*(q.w()*q.x() + q.y()*q.z()),
			q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z()),
		asin(2*(q.w()*q.y() - q.x()*q.z())),
		atan2(2*(q.x()*q.y() + q.w()*q.z()),
			q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z()));
	return res;
}

Quaternion euler2q(const Vector& v)
{
	return Quaternion::Zero();
}

}; //End namespace math_helpers.

}; //End namespace qc.
