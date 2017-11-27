/******************************************************************************
 * Quadcopter-Library-v1
 * qc_math.ino
 *
 * This is a test file for the vmath class.
 *
 * @author:	Rob Mertens
 * @date:	27/09/2017
 * @version:	1.1.1
 ******************************************************************************/

#include <math.h>
#include <math_helpers.hpp>

using namespace qc::math_helpers;

/**
 * @brief Create the Vector instances.
 */
//Create instances.
Vector u = Vector::Zero();
Vector v = Vector( 200.0,  -300.0, 20.0 );
Vector w(Vector::UnitY());

//Pointer typedefs to instances also supported.
Vector::Ptr u_ptr(&u);
Vector::CPtr v_cptr(&v);

/**
 * @brief Create the Quaternion instances.
 */
//Create instances.
Quaternion q = Quaternion::Zero();
Quaternion p = Quaternion( 52.0, -12.0, 20.0, 0.0 );
//Create a rotation along the x-axis with an angle of 45 degrees.
Quaternion r = Quaternion( Vector::UnitX(),  M_PI/4 );
//A so called "pure" quaternion, e.g. no rotation quaternion.
Quaternion s(Quaternion::UnitW());

//Pointer typedefs to instances also supported.
Quaternion::Ptr r_ptr(&r);
Quaternion::CPtr s_cptr(&s);

/**
 * @brief Setup function.
 */
void setup()
{
	//Serial.
	Serial.begin(9600);
}

/**
 * @brief Create a loop function.
 */
void loop()
{
	/**
	 * @brief Functions for Vectors.
	 */
	//Basic operators.
	Vector sum = v + u;
	Vector sub = v - w;
	Vector mult = 3.0*v;
	Vector frac = v/2.0;

	//Basic operators on itself.
	sum += w;
	sub -= (*u_ptr);
	mult *= 3.0;
	frac /= 5.9;

	//Vector attributes.
	double v_x = v_cptr->x();
	double mag = mult.magnitude();
	v.normalize();
	double d = dot(v, w);
	Vector c = cross(v, w);

	/**
	 * @brief Functions for Quaternion.
	 */
	//Basic operators.
 	Quaternion plus = q + p;
 	Quaternion minus = p - q;
 	Quaternion three_times_r = 3.0*r;
 	Quaternion one_time_r = three_times_r/3.0;

	//
	plus += p;
	minus -= *r_ptr;

	//Consecutive quaternion rotations.
	//The order of rotation DOES matter.
	//NOTE::(first_s_second_r != first_r_second_s)
	Quaternion first_s_second_r = hamilton(*r_ptr, *s_cptr);
	Quaternion first_r_second_s = hamilton(*s_cptr, *r_ptr);

}
