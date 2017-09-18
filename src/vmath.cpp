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
}

/*******************************************************************************
 * Constructor for a vector.
 *
 * @param x The x value as a float.
 * @param y The y value as a float.
 * @param z The z value as a float.
 ******************************************************************************/
vector::vector(const float x,
							 const float y,
							 const float z)
{
	_x = x;
	_y = y;
	_z = z;
}

/*******************************************************************************
 *	Method for normalizing the vector as Euclidean norm.
 ******************************************************************************/
void vector::normalize(void)
{
	_x /= _m;
	_x /= _m;
	_x /= _m;
}

/*******************************************************************************
 *	Method for normalizing the vector as Euclidean norm.
 ******************************************************************************/
void vector::multiply(const float s)
{
	_x *= s;
	_y *= s;
	_z *= s;
}

/*******************************************************************************
 * Method for calculating the vector magnitude.
 *
 * @return m The magnitude float.
 ******************************************************************************/
void vector::x(const float x)
{
	_x = x;
}

/*******************************************************************************
 * Method for calculating the vector magnitude.
 *
 * @return m The magnitude float.
 ******************************************************************************/
void vector::y(const float y)
{
	_y = y;
}

/*******************************************************************************
 * Method for calculating the vector magnitude.
 *
 * @return m The magnitude float.
 ******************************************************************************/
void vector::z(const float z)
{
	_z = z;
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
float vector::magnitude(void)
{
	return sqrt(_x*_x + _y*_y + _z*_z);
}

/*******************************************************************************
 * Method for summizing the instance with another vector instance.
 *
 * @param v The operand vector.
 * @return result The result vector.
 ******************************************************************************/
void sum(const vector::cptr& v,
				 const vector::cptr& w,
				 vector::cptr& res)
{
	res->x(v->x() + w->x());
	res->y(v->y() + w->y());
	res->z(v->z() + w->z());
}

/*******************************************************************************
 * Method for subtracting the instance with another vector instance.
 *
 * @param v The operand vector.
 * @return result The result vector.
 ******************************************************************************/
void subtract(const vector::cptr& v,
							const vector::cptr& w,
							vector::cptr& res)
{
 	res->x(v->x() - w->x());
 	res->y(v->y() - w->y());
 	res->z(v->z() - w->z());
}

/*******************************************************************************
 * Method for multiplying the instance with a scalar.
 *
 * @param s The operand scalar.
 * @return result The result vector.
 ******************************************************************************/
void multiply(const vector::cptr& v,
							const float s, vector::cptr& res)
{
	res->x(v->x()*s);
 	res->y(v->y()*s);
 	res->z(v->z()*s);
}

/*******************************************************************************
 * Method for calculating the cross product of the instance with another vector instance.
 *
 * @param v The operand vector.
 * @return result The result vector.
 ******************************************************************************/
void cross(const vector::cptr& v,
					 const vector::cptr& w,
					 vector::cptr& res)
{
	res->x(v->y()*w->z() - v->z()*w->y());
	res->y(v->z()*w->x() - v->x()*w->z());
	res->z(v->x()*w->y() - v->y()*w->x());
}

/*******************************************************************************
 * Method for calculating the dot product of the instance with another vector instance.
 *
 * @param v The operand vector.
 * @return result The result float.
 ******************************************************************************/
float dot(const vector::cptr& v,
					const vector::cptr& w)
{
	return (v->x()*w->x() + v->y()*w->y() + v->z()*w->z());
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
}

/*******************************************************************************
 * Constructor for a quaternion based on rotation angle and vector.
 *
 * @param a The rotation angle float.
 * @param e The rotation vector.
 ******************************************************************************/
quaternion::quaternion(const vector::cptr& e,
											 const float a)
{
	_w = cos(0.5*a);
	_x = (e->x())*sin(0.5*a);
	_y = (e->y())*sin(0.5*a);
	_z = (e->z())*sin(0.5*a);
}

/*******************************************************************************
 * Constructor for a quaternion.
 *
 * @param w The w value float.
 * @param x The x value float.
 * @param y The y value float.
 * @param z The z value float.
 ******************************************************************************/
quaternion::quaternion(const float w,
											 const float x,
											 const float y,
											 const float z)
{
	_w = w;
	_x = x;
	_y = y;
	_z = z;
}

/*******************************************************************************
 * Method for normalizing the quaternion as Euclidean norm.
 ******************************************************************************/
void quaternion::normalize(void)
{
	float m = magnitude();
	_w /= m;
	_x /= m;
	_y /= m;
	_z /= m;
}

/*******************************************************************************
 * Method for normalizing the quaternion as Euclidean norm.
 ******************************************************************************/
void normalize(const quaternion::cptr& q,
							 quaternion::cptr& res)
{
	float m = q->magnitude();
	res->w(q->w()/m);
	res->x(q->x()/m);
	res->y(q->y()/m);
	res->z(q->z()/m);
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
}

/*******************************************************************************
 * Method for obtaining the conjugate quaternion.
 ******************************************************************************/
void conjugate(const quaternion::cptr& q,
							 quaternion::cptr& res)
{
	res->w(q->w());
	res->x(q->x()*(-1.0f));
	res->y(q->y()*(-1.0f));
	res->z(q->z()*(-1.0f));
}

/*******************************************************************************
 * Method for obtaining the inverse quaternion.
 ******************************************************************************/
void quaternion::inverse(void)
{
	float m = magnitude();
	_w /= m;
	_x /= (-1.0f*m);
	_y /= (-1.0f*m);
	_z /= (-1.0f*m);
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
float quaternion::magnitude(void)
{
	return sqrt(_w*_w + _x*_x + _y*_y + _z*_z);
}

/*******************************************************************************
 * Method for summizing the instance with another quaternion instance.
 *
 * @param q The operand quaternion.
 * @return result The result quaternion.
 ******************************************************************************/
void sum(const quaternion::cptr& q,
				 const quaternion::cptr& p,
				 quaternion::cptr& res)
{
	res->w(q->w() + p->w());
	res->x(q->x() + p->x());
	res->y(q->y() + p->y());
	res->z(q->z() + p->z());
}

/*******************************************************************************
 * Method for summizing the instance with another quaternion instance.
 *
 * @param q The operand quaternion.
 * @return result The result quaternion.
 ******************************************************************************/
void subtract(const quaternion::cptr& q,
							const quaternion::cptr& p,
							quaternion::cptr& res)
{
	res->w(q->w() + p->w());
	res->x(q->x() + p->x());
	res->y(q->y() + p->y());
	res->z(q->z() + p->z());
}

/*******************************************************************************
 * Method for multiplying a quaternion with a scalar.
 *
 * @param s The operand float.
 * @return result The result quaternion.
 ******************************************************************************/
void multiply(const quaternion::cptr& q,
							const float s,
							quaternion::cptr& res)
{
	res->w(q->w()*s);
	res->x(q->x()*s);
	res->y(q->y()*s);
	res->z(q->z()*s);
}

/*******************************************************************************
 * Method for multiplying the instance with another quaternion instance.
 *
 * @param q The operand quaternion.
 * @return result The result quaternion.
 ******************************************************************************/
void cross(const quaternion::cptr& q,
					 const quaternion::cptr& p,
					 quaternion::cptr& res)
{
	res->w(q->w()*p->w() - q->x()*p->x() - q->y()*p->y() - q->z()*p->z());
	res->x(q->w()*p->x() + q->x()*p->w() + q->y()*p->z() - q->z()*p->y());
	res->y(q->w()*p->y() - q->x()*p->z() + q->y()*p->w() + q->z()*p->x());
	res->z(q->w()*p->z() + q->x()*p->y() - q->y()*p->x() + q->z()*p->w());
}

/*******************************************************************************
 * Method for calculating the dot product of the instance with another quaternion instance.
 *
 * @param q The operand quaternion.
 * @return result The result float.
 ******************************************************************************/
float dot(const quaternion::cptr& q,
					const quaternion::cptr& p)
{
	return (q->w()*p->w() + q->x()*p->x() + q->y()*p->y() + q->z()*p->z());
}

/*******************************************************************************
 * Method for rotating a vector over an instance quaternion.
 *
 * @param q The quaternion describing the rotation.
 * @param v The operand vector.
 * @param res The result vector.
 ******************************************************************************/
void rotate(const quaternion::cptr& q,
						const vector::cptr& v,
						vector::cptr& res)
{
	res->x((q->w()*q->w() + q->x()*q->x() - q->y()*q->y() - q->z()*q->z())*v->x() + 2*(q->x()*q->y() - q->w()*q->z())*v->y() + 2*(q->x()*q->z() + q->w()*q->y())*v->z());
	res->y(2*(q->x()*q->y() + q->w()*q->z())*v->x() + (q->w()*q->w() - q->x()*q->x() + q->y()*q->y() - q->z()*q->z())*v->y() + 2*(q->y()*q->z() - q->w()*q->x())*v->z());
	res->z(2*(q->x()*q->z() - q->w()*q->y())*v->x() + 2*(q->y()*q->z() + q->w()*q->x())*v->y() + (q->w()*q->w() - q->x()*q->x() - q->y()*q->y() + q->z()*q->z())*v->z());
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
void q2euler(const quaternion::cptr& q,
						 vector::cptr& res)
{
	res->x(atan2(2*(q->w()*q->x() + q->y()*q->z()), q->w()*q->w() - q->x()*q->x() - q->y()*q->y() + q->z()*q->z()));
	res->y(asin(2*(q->w()*q->y() - q->x()*q->z())));
	res->z(atan2(2*(q->x()*q->y() + q->w()*q->z()), q->w()*q->w() + q->x()*q->x() - q->y()*q->y() - q->z()*q->z()));
}

/*******************************************************************************
 * TODO::
 ******************************************************************************/
void euler2q(const vector::cptr& v,
						 quaternion::cptr& res)
{
	;;
}

}; //End namespace
