/******************************************************************************
 *	Quadcopter-Library-v1
 *  math.cpp
 *  
 *	This file contains the vector and quaternion class which represent vectors
 *	with respectively three and four entries. These classes also contain the
 *	general mathematical operations and some special quaternion related operations.
 *
 *	TODO::Generalizing vector class and extend as quaternion class.
 *
 *  @author Rob Mertens
 *  @version 1.0.1 14/08/2016
 ******************************************************************************/

#include <math.h>

/*******************************************************************************
 *	Constructor for a null-vector.
 ******************************************************************************/
vector::vector()
{
	_x = 0.0f;
	_y = 0.0f;
	_z = 0.0f;
	_m = 0.0f;
}

/*******************************************************************************
 *	Constructor for a vector.
 *
 * @param x The x value as a float.
 * @param y The y value as a float.
 * @param z The z value as a float.
 ******************************************************************************/
vector::vector(float x, float y, float z)
{
	_x = x;
	_y = y;
	_z = z;
	_m = getMagnitude();
}

/*******************************************************************************
 *	Method for normalizing the vector as Euclidean norm.
 ******************************************************************************/
void vector::norm()
{
	_x /= _m;
	_y /= _m;
	_z /= _m;
	_m = 1.0f;
}

/*******************************************************************************
 *	Method for summizing the instance with another vector instance.
 *
 *  @param v The operand vector.
 *  @return result The result vector.
 ******************************************************************************/
vector vector::sum(vector v)
{
	return vector(_x + v._x,
				  _y + v._y,
				  _z + v._z);
}

/*******************************************************************************
 *	Method for substracting the instance with another vector instance.
 *
 *  @param v The operand vector.
 *  @return result The result vector.
 ******************************************************************************/
vector vector::substract(vector v)
{
	return vector(_x - v._x,
				  _y - v._y,
				  _z - v._z);
}

/*******************************************************************************
 *	Method for multiplying the instance with a scalar.
 *
 *  @param s The operand scalar.
 *  @return result The result vector.
 ******************************************************************************/
vector vector::multiply(float s)
{
	return vector(s*_x,
				  s*_y,
				  s*_z);
}

/*******************************************************************************
 *	Method for calculating the dot product of the instance with another vector instance.
 *
 *  @param v The operand vector.
 *  @return result The result float.
 ******************************************************************************/
float vector::dot(vector v)
{
	return (_x*v._x + _y*v._y + _z*v._z);
}

/*******************************************************************************
 *	Method for calculating the cross product of the instance with another vector instance.
 *
 *  @param v The operand vector.
 *  @return result The result vector.
 ******************************************************************************/
vector vector::cross(vector v)
{
	return vector(_y*v._z - _z*v._y,
				 -_x*v._z + _z*v._x,
				  _x*v._y - _y*v._x);
}

/*******************************************************************************
 *	Method for calculating the vector magnitude.
 *
 *  @return m The magnitude float.
 ******************************************************************************/
float vector::getMagnitude()
{
	return sqrt(_x*_x + _y*_y + _z*_z);
}

/*******************************************************************************
 *	Constructor for a real unit quaternion.
 ******************************************************************************/
quaternion::quaternion()
{
	_w = 1.0f;
	_x = 0.0f;
	_y = 0.0f;
	_z = 0.0f;
	_m = 1.0f; 
}

/*******************************************************************************
 *	Constructor for a quaternion based on rotation angle and vector.
 *
 *  @param a The rotation angle float.
 *  @param e The rotation vector.
 ******************************************************************************/
quaternion::quaternion(float a, vector e)
{
	_w = cos(0.5*a);
	_x = (e.x)*sin(0.5*a);
	_y = (e.y)*sin(0.5*a);
	_z = (e.z)*sin(0.5*a);
	_m = getMagnitude();
}

/*******************************************************************************
 *	Constructor for a quaternion.
 *
 *  @param w The w value float.
 *  @param x The x value float.
 *  @param y The y value float.
 *  @param z The z value float.
 ******************************************************************************/
quaternion::quaternion(float w, float x, float y, float z)
{
	_w = w;
	_x = x;
	_y = y;
	_z = z;
	_m = getMagnitude();
}

/*******************************************************************************
 *	Method for normalizing the quaternion as Euclidean norm.
 ******************************************************************************/
void quaternion::norm()
{
	_w /= _m;
	_x /= _m;
	_y /= _m;
	_z /= _m;
	_m = 1.0f;
}

/*******************************************************************************
 *	Method for obtaining the conjugate quaternion.
 ******************************************************************************/
void quaternion::conj()
{
	_x *= -1.0f;
	_y *= -1.0f;
	_z *= -1.0f; 
}

/*******************************************************************************
 *	Method for obtaining the inverse quaternion.
 ******************************************************************************/
void quaternion::inv()
{
	_w /= _m;
	_x /= -(1.0f*_m);
	_y /= -(1.0f*_m);
	_z /= -(1.0f*_m);
	_m = getMagnitude();
}

/*******************************************************************************
 *	Method for summizing the instance with another quaternion instance.
 *
 * @param q The operand quaternion.
 * @return result The result quaternion.
 ******************************************************************************/
quaternion quaternion::sum(quaternion q)
{
	return quaternion(_w + q._w,
					  _x + q._x,
					  _y + q._y,
					  _z + q._z);
}

/*******************************************************************************
 *	Method for calculating the dot product of the instance with another quaternion instance.
 *
 * @param q The operand quaternion.
 * @return result The result float.
 ******************************************************************************/
float quaternion::dot(quaternion q)
{
	return (_w*q._w + _x*q._x + _y*q._y + _z*q._z);
}

/*******************************************************************************
 *	Method for multiplying the instance with a scalar.
 *
 * @param s The operand float.
 * @return result The result quaternion.
 ******************************************************************************/
quaternion quaternion::multiply(float s)
{        
	return quaternion(s*_w,
					  s*_x,
					  s*_y,
					  s*_z);
}

/*******************************************************************************
 *	Method for multiplying the instance with another quaternion instance.
 *
 * @param q The operand quaternion.
 * @return result The result quaternion.
 ******************************************************************************/
quaternion quaternion::cross(quaternion q)
{
return quaternion(_w*q._w - _x*q._x - _y*q._y - _z*q._z,
				  _w*q._x + _x*q._w + _y*q._z - _z*q._y,
				  _w*q._y - _x*q._z + _y*q._w + _z*q._x,
				  _w*q._z + _x*q._y - _y*q._x + _z*q._w);
}

/*******************************************************************************
 *	Method for rotating a vector over an instance quaternion.
 *
 * @param v The operand vector.
 * @return result The result vector.
 ******************************************************************************/
vector quaternion::getRotation(vector v)
{
return vector((_w*_w + _x*_x - _y*_y - _z*_z)*v._x 	+ 	 2*(_x*_y - _w*_z)*v._y 			+ 	 	2*(_x*_z + _w*_y)*v._z		  ,
				    2*(_x*_y + _w*_z)*v._x 	  		+ (_w*_w - _x*_x + _y*_y - _z*_z)*v._y  + 	 	2*(_y*_z - _w*_x)*v._z		  ,
				    2*(_x*_z - _w*_y)*v._x	  		+ 	 2*(_y*_z + _w*_x)*v._y 			+ (_w*_w - _x*_x - _y*_y + _z*_z)*v._z);
}

/*******************************************************************************
 *	Method for obtaining the corresponding XYZ-Euler angles from a quaternion rotation.
 *
 * @return result The result vector.
 ******************************************************************************/
vector quaternion::getEulerAngles()
{
return vector(atan2(2*(_w*_x + _y*_z), _w*_w - _x*_x - _y*_y + _z*_z),
			  asin(2*(_w*_y - _x*_z)),
			  atan2(2*(_x*_y + _w*_z), _w*_w + _x*_x - _y*_y - _z*_z));
}

/*******************************************************************************
 *	Method for calculating the magnitude.
 *
 * @return m The quaternion magnitude.
 ******************************************************************************/
float quaternion::getMagnitude()
{
	return sqrt(_w*_w + _x*_x + _y*_y + _z*_z);
}
