/******************************************************************************
 * Quadcopter-Library-v1
 * vmath.cpp
 *
 * This file contains the vector and quaternion class which represent vectors
 * with respectively three and four entries. These classes also contain the
 * general mathematical operations and some special quaternion related operations.
 *
 * @author Rob Mertens
 * @version 1.0.1 14/08/2016
 ******************************************************************************/

#include "vmath.h"

namespace vmath
{

/*******************************************************************************
 * Constructor for a null-vector.
 ******************************************************************************/
vector::vector(void)
{
	_x = 0.0f;
	_y = 0.0f;
	_z = 0.0f;
	_m = 0.0f;
}

/*******************************************************************************
 * Constructor for a vector.
 *
 * @param x The x value as a float.
 * @param y The y value as a float.
 * @param z The z value as a float.
 ******************************************************************************/
vector::vector(const float x, const float y, const float z)
{
	_x = x;
	_y = y;
	_z = z;
	_m = mag();
}

/*******************************************************************************
 *	Method for normalizing the vector as Euclidean norm.
 ******************************************************************************/
void vector::normalize(void)
{
	_x /= m;
	_x /= m;
	_x /= m;
	_m = 1.0f;
}

/*******************************************************************************
 *	Method for normalizing the vector as Euclidean norm.
 ******************************************************************************/
void vector::multiply(const float s)
{
	_x *= s;
	_y *= s;
	_z *= s;
	_m = mag();
}

/*******************************************************************************
 * Method for calculating the vector magnitude.
 *
 * @return m The magnitude float.
 ******************************************************************************/
void vector::x(const float _x)
{
	this->_x = _x;
}

/*******************************************************************************
 * Method for calculating the vector magnitude.
 *
 * @return m The magnitude float.
 ******************************************************************************/
void vector::y(const float _y)
{
	this->_y = _y;
}

/*******************************************************************************
 * Method for calculating the vector magnitude.
 *
 * @return m The magnitude float.
 ******************************************************************************/
void vector::z(const float _z)
{
	this->_z = _z;
}

/*******************************************************************************
 * Method for calculating the vector magnitude.
 *
 * @return m The magnitude float.
 ******************************************************************************/
float vector::x(void)
{
	return _x;
}

/*******************************************************************************
 * Method for calculating the vector magnitude.
 *
 * @return m The magnitude float.
 ******************************************************************************/
float vector::y(void)
{
	return _y;
}

/*******************************************************************************
 * Method for calculating the vector magnitude.
 *
 * @return m The magnitude float.
 ******************************************************************************/
float vector::z(void)
{
	return _z;
}

/*******************************************************************************
 * Method for calculating the vector magnitude.
 *
 * @return m The magnitude float.
 ******************************************************************************/
float vector::mag(void)
{
	return sqrt(_x*_x + _y*_y + _z*_z);
}

/*******************************************************************************
 * Method for summizing the instance with another vector instance.
 *
 * @param v The operand vector.
 * @return result The result vector.
 ******************************************************************************/
void sum(const vector::cptr& v, const vector::cptr& w, vector::cptr& res)
{
	res->_x = v->_x + w->_x;
	res->_y = v->_y + w->_y;
	res->_z = v->_z + w->_z;
	res->_m = res->mag();
}

/*******************************************************************************
 * Method for subtracting the instance with another vector instance.
 *
 * @param v The operand vector.
 * @return result The result vector.
 ******************************************************************************/
void subtract(const vector::cptr& v, const vector::cptr& w, vector::cptr& res)
{
 	res->_x = v->_x - w->_x;
 	res->_y = v->_x - w->_y;
 	res->_z = v->_x - w->_z;
 	res->_m = res->mag();
}

/*******************************************************************************
 * Method for multiplying the instance with a scalar.
 *
 * @param s The operand scalar.
 * @return result The result vector.
 ******************************************************************************/
void multiply(const vector::cptr& v, const float s, vector::cptr& res)
{
	res->_x = v->_x*s;
 	res->_y = v->_y*s;
 	res->_z = v->_z*s;
 	res->_m = res->mag();
}

/*******************************************************************************
 * Method for calculating the cross product of the instance with another vector instance.
 *
 * @param v The operand vector.
 * @return result The result vector.
 ******************************************************************************/
void cross(const vector::cptr& v, const vector::cptr& w, vector::cptr& res)
{
	res->_x = v->_y*w->_z - v->_z*w->_y;
	res->_y = v->_z*w->_x - v->_x*w->_z;
	res->_z = v->_x*w->_y - v->_y*w->_x;
	res->_m = res->mag();
}

/*******************************************************************************
 * Method for calculating the dot product of the instance with another vector instance.
 *
 * @param v The operand vector.
 * @return result The result float.
 ******************************************************************************/
float vector::dot(const vector::cptr& v, const vector::cptr& w)
{
	return (v->_x*w->_x + v->_y*w->_y + v->_z*w->_z);
}

/*******************************************************************************
 * Constructor for a real unit quaternion.
 ******************************************************************************/
quaternion::quaternion(void)
{
	_w = 1.0f;
	_x = 0.0f;
	_y = 0.0f;
	_z = 0.0f;
	_m = 1.0f;
}

/*******************************************************************************
 * Constructor for a quaternion based on rotation angle and vector.
 *
 * @param a The rotation angle float.
 * @param e The rotation vector.
 ******************************************************************************/
quaternion::quaternion(const vector::cptr& e, const float a)
{
	_w = cos(0.5*a);
	_x = (e->_x)*sin(0.5*a);
	_y = (e->_y)*sin(0.5*a);
	_z = (e->_z)*sin(0.5*a);
	_m = mag();
}

/*******************************************************************************
 * Constructor for a quaternion.
 *
 * @param w The w value float.
 * @param x The x value float.
 * @param y The y value float.
 * @param z The z value float.
 ******************************************************************************/
quaternion::quaternion(const float w, const float x, const float y, const float z)
{
	_w = w;
	_x = x;
	_y = y;
	_z = z;
	_m = mag();
}

/*******************************************************************************
 * Method for normalizing the quaternion as Euclidean norm.
 ******************************************************************************/
void quaternion::normalize(void)
{
	_w /= _m;
	_x /= _m;
	_y /= _m;
	_z /= _m;
	_m = 1.0f;
}

/*******************************************************************************
 * Method for normalizing the quaternion as Euclidean norm.
 ******************************************************************************/
void normalize(const quaternion::cptr& q, quaternion::cptr& res)
{
	res->_w = q->_w/q->_m;
	res->_x = q->_x/q->_m;
	res->_y = q->_y/q->_m;
	res->_z = q->_z/q->_m;
	res->_m = 1.0f;
}

/*******************************************************************************
 * Method for obtaining the conjugate quaternion.
 ******************************************************************************/
void quaternion::conjugate(void)
{
	///_w remains unchanged.
	_x *= (-1.0f);
	_y *= (-1.0f);
	_z *= (-1.0f);
	//_m remains unchanged.
}

/*******************************************************************************
 * Method for obtaining the conjugate quaternion.
 ******************************************************************************/
void conjugate(const quaternion::cptr& q, quaternion::cptr& res)
{
	res->_w = q->_w;
	res->_x = q->_x*(-1.0f);
	res->_y = q->_y*(-1.0f);
	res->_z = q->_z*(-1.0f);
	res->_m = q->_m;
}

/*******************************************************************************
 * Method for obtaining the inverse quaternion.
 ******************************************************************************/
void quaternion::inverse(void)
{
	_w /= _m;
	_x /= (-1.0f*_m);
	_y /= (-1.0f*_m);
	_z /= (-1.0f*_m);
	_m = 1.0f;
}

/*******************************************************************************
 * Method for multiplying the quaternion with a scalar.
 *
 * @param s The operand float.
 ******************************************************************************/
void quaternion::multiply(const float s)
{
	_w *= s;
	_x *= s;
	_y *= s;
	_z *= s;
	_m = mag();
}

/*******************************************************************************
 * Method for calculating the vector magnitude.
 *
 * @return m The magnitude float.
 ******************************************************************************/
void quaternion::w(const float w)
{
	_w = w;
}

/*******************************************************************************
 * Method for calculating the vector magnitude.
 *
 * @return m The magnitude float.
 ******************************************************************************/
void quaternion::x(const float x)
{
	_x = x;
}

/*******************************************************************************
 * Method for calculating the vector magnitude.
 *
 * @return m The magnitude float.
 ******************************************************************************/
void quaternion::y(const float y)
{
	_y = y;
}

/*******************************************************************************
 * Method for calculating the vector magnitude.
 *
 * @return m The magnitude float.
 ******************************************************************************/
void quaternion::z(const float z)
{
	_z = z;
}

/*******************************************************************************
 * Method for calculating the vector magnitude.
 *
 * @return m The magnitude float.
 ******************************************************************************/
float quaternion::w(void)
{
	return _w;
}

/*******************************************************************************
 * Method for calculating the vector magnitude.
 *
 * @return m The magnitude float.
 ******************************************************************************/
float quaternion::x(void)
{
	return _x;
}

/*******************************************************************************
 * Method for calculating the vector magnitude.
 *
 * @return m The magnitude float.
 ******************************************************************************/
float quaternion::y(void)
{
	return _y;
}

/*******************************************************************************
 * Method for calculating the vector magnitude.
 *
 * @return m The magnitude float.
 ******************************************************************************/
float quaternion::z(void)
{
	return _z;
}

/*******************************************************************************
 * Method for calculating the magnitude.
 *
 * @return m The quaternion magnitude.

 ******************************************************************************/
float quaternion::mag(void)
{
	return sqrt(_w*_w + _x*_x + _y*_y + _z*_z);
}

/*******************************************************************************
 * Method for summizing the instance with another quaternion instance.
 *
 * @param q The operand quaternion.
 * @return result The result quaternion.
 ******************************************************************************/
void sum(const quaternion::cptr& q, const quaternion::cptr& p, quaternion::cptr& res)
{
	res->_w = q->_w + p->_w;
	res->_x = q->_x + p->_x;
	res->_y = q->_y + p->_y;
	res->_z = q->_z + p->_z;
	res->_m = res->mag();
}

/*******************************************************************************
 * Method for summizing the instance with another quaternion instance.
 *
 * @param q The operand quaternion.
 * @return result The result quaternion.
 ******************************************************************************/
void subtract(const quaternion::cptr& q, const quaternion::cptr& p, quaternion::cptr& res)
{
	res->_w = q->_w + p->_w;
	res->_x = q->_x + p->_x;
	res->_y = q->_y + p->_y;
	res->_z = q->_z + p->_z;
	res->_m = res->mag();
}

/*******************************************************************************
 * Method for multiplying a quaternion with a scalar.
 *
 * @param s The operand float.
 * @return result The result quaternion.
 ******************************************************************************/
void multiply(const quaternion::cptr& q, const float s, quaternion::cptr& res)
{
	res->_w = q->_w*s;
	res->_x = q->_x*s;
	res->_y = q->_y*s;
	res->_z = q->_z*s;
	res->_m = res->mag();
}

/*******************************************************************************
 * Method for multiplying the instance with another quaternion instance.
 *
 * @param q The operand quaternion.
 * @return result The result quaternion.
 ******************************************************************************/
void cross(const quaternion::cptr& q, const quaternion::cptr& p, quaternion::cptr& res)
{
	res->_w = q->_w*p->_w - q->_x*p->_x - q->_y*p->_y - q->_z*p->_z,
	res->_x = q->_w*p->_x + q->_x*p->_w + q->_y*p->_z - q->_z*p->_y,
	res->_y = q->_w*p->_y - q->_x*p->_z + q->_y*p->_w + q->_z*p->_x,
	res->_z = q->_w*p->_z + q->_x*p->_y - q->_y*p->_x + q->_z*p->_w);
	res->_m = res->mag();
}

/*******************************************************************************
 * Method for calculating the dot product of the instance with another quaternion instance.
 *
 * @param q The operand quaternion.
 * @return result The result float.
 ******************************************************************************/
float quaternion::dot(const quaternion::cptr& q, const quaternion::cptr& p)
{
	return (q->_w*p->_w + q->_x*p->_x + q->_y*p->_y + q->_z*p->_z);
}

/*******************************************************************************
 * Method for rotating a vector over an instance quaternion.
 *
 * @param q The quaternion describing the rotation.
 * @param v The operand vector.
 * @param res The result vector.
 ******************************************************************************/
void rotate(const quaternion::cptr& q, const vector::cptr& v, vector::cptr& res)
{
	res->_x = (q->_w*q->_w + q->_x*q->_x - q->_y*q->_y - q->_z*q->_z)*v->_x + 2*(q->_x*q->_y - q->_w*q->_z)*v->_y + 2*(q->_x*q->_z + q->_w*q->_y)*v->_z;
	res->_y = 2*(q->_x*q->_y + q->_w*q->_z)*v->_x + (q->_w*q->_w - q->_x*q->_x + q->_y*q->_y - q->_z*q->_z)*v->_y + 2*(q->_y*q->_z - q->_w*q->_x)*v->_z;
	res->_z = 2*(q->_x*q->_z - q->_w*q->_y)*v->_x + 2*(q->_y*q->_z + q->_w*q->_x)*v->_y + (q->_w*q->_w - q->_x*q->_x - q->_y*q->_y + q->_z*q->_z)*v->_z;
	res->_m = res->mag();
}

/*******************************************************************************
 * Method for rotating a vector over an instance quaternion.
 *
 * @param q The quaternion describing the rotation.
 * @param v The operand vector.
 * @param res The result vector.
 ******************************************************************************/
void lerp(const quaternion::cptr& q,
					const quaternion::cptr& p,
					const float alpha,
					quaternion::cptr& res)
{
	quaternion::ptr operand1(new quaternion);
	quaternion::ptr operand2(new quaternion);
	multiply(q, 1-alpha, operand1);
	multiply(p, alpha, operand2);
	sum(operand1, operand2, res);
	res->normalize();
}

/*******************************************************************************
 * Method for rotating a vector over an instance quaternion.
 *
 * @param q The quaternion describing the rotation.
 * @param v The operand vector.
 * @param res The result vector.
 ******************************************************************************/
void slerp(const quaternion::cptr& q,
					const quaternion::cptr& p,
					const float alpha,
					quaternion::cptr& res)
{
	float omega;
	quaternion::ptr operand1(new quaternion);
	quaternion::ptr operand2(new quaternion);
	omega = acos(dot(q, p));
	if(fabs(omega) > 0.9995f)
	{
		lerp(q, p, alpha, res);
		return;
	}
	if(omega < 0.0f)
	{

	}
}

/*******************************************************************************
 * Method for obtaining the corresponding XYZ-Euler angles from a quaternion rotation.
 *
 * @return result The result vector.
 ******************************************************************************/
void q2euler(const quaternion::cptr& q, vector::cptr& res)
{
	res->_x = atan2(2*(q->_w*q->_x + q->_y*q->_z), q->_w*q->_w - q->_x*q->_x - q->_y*q->_y + q->_z*q->_z);
	res->_y = asin(2*(q->_w*q->_y - q->_x*q->_z));
	res->_z = atan2(2*(q->_x*q->_y + q->_w*q->_z), q->_w*q->_w + q->_x*q->_x - q->_y*q->_y - q->_z*q->_z));
	res->_m = res->mag();
}

/*******************************************************************************
 * TODO::
 ******************************************************************************/
void euler2q(const vector::cptr& v, quaternion::cptr& res)
{
	;;
}

}; //End namespace
