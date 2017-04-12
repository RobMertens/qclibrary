/******************************************************************************
 * Quadcopter-Library-v1
 * math.cpp
 *  
 * This file contains the vector and quaternion class which represent vectors
 * with respectively three and four entries. These classes also contain the
 * general mathematical operations and some special quaternion related operations.
 *
 * TODO::Generalizing vector class and extend as quaternion class.
 *
 * @author Rob Mertens
 * @version 1.0.1 14/08/2016
 ******************************************************************************/

#include <math.h>

/*******************************************************************************
 * Constructor for a null-vector.
 ******************************************************************************/
vector::vector()
{
	x = 0.0f;
	y = 0.0f;
	z = 0.0f;
	m = 0.0f;
}

/*******************************************************************************
 * Constructor for a vector.
 *
 * @param x The x value as a float.
 * @param y The y value as a float.
 * @param z The z value as a float.
 ******************************************************************************/
vector::vector(float x, float y, float z)
{
	x = x;
	y = y;
	z = z;
	m = mag();
}

/*******************************************************************************
 *	Method for normalizing the vector as Euclidean norm.
 ******************************************************************************/
void vector::norm()
{
	x /= m;
	y /= m;
	z /= m;
	m = 1.0f;
}

/*******************************************************************************
 * Method for summizing the instance with another vector instance.
 * 
 * @param v The operand vector.
 * @return result The result vector.
 ******************************************************************************/
vector vector::sum(vector v)
{
	return vector(x + v.x,
		      y + v.y,
		      z + v.z);
}

/*******************************************************************************
 * Method for substracting the instance with another vector instance.
 * 
 * @param v The operand vector.
 * @return result The result vector.
 ******************************************************************************/
vector vector::substract(vector v)
{
	return vector(x - v.x,
		      y - v.y,
		      z - v.z);
}

/*******************************************************************************
 * Method for multiplying the instance with a scalar.
 * 
 * @param s The operand scalar.
 * @return result The result vector.
 ******************************************************************************/
vector vector::multiply(float s)
{
	return vector(s*x,
		      s*y,
		      s*z);
}

/*******************************************************************************
 * Method for calculating the dot product of the instance with another vector instance.
 * 
 * @param v The operand vector.
 * @return result The result float.
 ******************************************************************************/
float vector::dot(vector v)
{
	return (x*v.x + y*v.y + z*v.z);
}

/*******************************************************************************
 * Method for calculating the cross product of the instance with another vector instance.
 *
 * @param v The operand vector.
 * @return result The result vector.
 ******************************************************************************/
vector vector::cross(vector v)
{
	return vector(y*v.z - z*v.y,
		      z*v.x - x*v.z,
		      x*v.y - y*v.x);
}

/*******************************************************************************
 * Method for calculating the vector magnitude.
 * 
 * @return m The magnitude float.
 ******************************************************************************/
float vector::mag()
{
	return sqrt(x*x + y*y + z*z);
}

/*******************************************************************************
 * Constructor for a real unit quaternion.
 ******************************************************************************/
quaternion::quaternion()
{
	w = 1.0f;
	x = 0.0f;
	y = 0.0f;
	z = 0.0f;
	m = 1.0f; 
}

/*******************************************************************************
 * Constructor for a quaternion based on rotation angle and vector.
 *
 * @param a The rotation angle float.
 * @param e The rotation vector.
 ******************************************************************************/
quaternion::quaternion(float a, vector e)
{
	w = cos(0.5*a);
	x = (e.x)*sin(0.5*a);
	y = (e.y)*sin(0.5*a);
	z = (e.z)*sin(0.5*a);
	m = getMagnitude();
}

/*******************************************************************************
 * Constructor for a quaternion.
 * 
 * @param w The w value float.
 * @param x The x value float.
 * @param y The y value float.
 * @param z The z value float.
 ******************************************************************************/
quaternion::quaternion(float w, float x, float y, float z)
{
	w = w;
	x = x;
	y = y;
	z = z;
	m = mag();
}

/*******************************************************************************
 * Method for normalizing the quaternion as Euclidean norm.
 ******************************************************************************/
void quaternion::norm()
{
	w /= m;
	x /= m;
	y /= m;
	z /= m;
	m = 1.0f;
}

/*******************************************************************************
 * Method for obtaining the conjugate quaternion.
 ******************************************************************************/
void quaternion::conj()
{
	x *= - 1.0f;
	y *= - 1.0f;
	z *= - 1.0f; 
}

/*******************************************************************************
 * Method for obtaining the inverse quaternion.
 ******************************************************************************/
void quaternion::inv()
{
	w /= m;
	x /= -(1.0f*m);
	y /= -(1.0f*m);
	z /= -(1.0f*m);
	m = mag();
}

/*******************************************************************************
 * Method for summizing the instance with another quaternion instance.
 * 
 * @param q The operand quaternion.
 * @return result The result quaternion.
 ******************************************************************************/
quaternion quaternion::sum(quaternion q)
{
	return quaternion(w + q.w,
			  x + q.x,
			  y + q.y,
			  z + q.z);
}

/*******************************************************************************
 * Method for calculating the dot product of the instance with another quaternion instance.
 * 
 * @param q The operand quaternion.
 * @return result The result float.
 ******************************************************************************/
float quaternion::dot(quaternion q)
{
	return (w*q.w + x*q.x + y*q.y + z*q.z);
}

/*******************************************************************************
 * Method for multiplying the instance with a scalar.
 * 
 * @param s The operand float.
 * @return result The result quaternion.
 ******************************************************************************/
quaternion quaternion::multiply(float s)
{        
	return quaternion(s*w,
			  s*x,
			  s*y,
			  s*z);
}

/*******************************************************************************
 * Method for multiplying the instance with another quaternion instance.
 * 
 * @param q The operand quaternion.
 * @return result The result quaternion.
 ******************************************************************************/
quaternion quaternion::cross(quaternion q)
{
	return quaternion(w*q.w - x*q.x - y*q.y - z*q.z,
			  w*q.x + x*q.w + y*q.z - z*q.y,
			  w*q.y - x*q.z + y*q.w + z*q.x,
			  w*q.z + x*q.y - y*q.x + z*q.w);
}

/*******************************************************************************
 * Method for rotating a vector over an instance quaternion.
 * 
 * @param v The operand vector.
 * @return result The result vector.
 ******************************************************************************/
vector quaternion::rotate(vector v)
{
	return vector((w*w + x*x - y*y - z*z)*v.x + 	      2*(x*y - w*z)*v.y + 	    2*(x*z + w*y)*v.z,
				2*(x*y + w*z)*v.x + (w*w - x*x + y*y - z*z)*v.y + 	    2*(y*z - w*x)*v.z,
				2*(x*z - w*y)*v.x + 	      2*(y*z + w*x)*v.y + (w*w - x*x - y*y + z*z)*v.z);
}

/*******************************************************************************
 * Method for obtaining the corresponding XYZ-Euler angles from a quaternion rotation.
 * 
 * @return result The result vector.
 ******************************************************************************/
vector quaternion::q2euler()
{
	return vector(atan2(2*(w*x + y*z), w*w - x*x - y*y + z*z),
		      asin(2*(w*y - x*z)),
		      atan2(2*(x*y + w*z), w*w + x*x - y*y - z*z));
}

/*******************************************************************************
 * Method for calculating the magnitude.
 * 
 * @return m The quaternion magnitude.
 ******************************************************************************/
float quaternion::mag()
{
	return sqrt(w*w + x*x + y*y + z*z);
}
