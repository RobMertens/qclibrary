#ifndef _VMATH_H_
#define _VMATH_H_

#include <math.h>

namespace vmath
{

	//Vector class ***************************************************************
	class vector
	{
		public:
			//Typedefs ***************************************************************
			typedef vector * ptr;//typedef std::shared_ptr<vector> ptr;
			typedef vector * const cptr;//typedef std::shared_ptr<const vector> cptr;

			//Variables **************************************************************
			float _x;
			float _y;
			float _z;
			float _m;

			//Constructors ***********************************************************
			vector(void);
			vector(const float, const float, const float);

			//Setters ****************************************************************
			void normalize(void);
			void multiply(const float);

			//Getters ****************************************************************
			float mag(void);

	}; //End vector class.

	//Functions for vector math **************************************************
	void sum(const vector::cptr&, const vector::cptr&, vector::cptr&);
	void subtract(const vector::cptr&, const vector::cptr&, vector::cptr&);
	void cross(const vector::cptr&, const vector::cptr&, vector::cptr&);
	float dot(const vector::cptr&, const vector::cptr&);

	//Quaternion class ***********************************************************
	class quaternion
	{
		public:
			//Typedefs ***************************************************************
			typedef quaternion * ptr;//typedef std::shared_ptr<quaternion> ptr;
			typedef quaternion * const cptr;//typedef std::shared_ptr<const quaternion> cptr;

			//Variables **************************************************************
			float _w;
			float _x;
			float _y;
			float _z;
			float _m;

			//Constructors ***********************************************************
			quaternion(void);
			quaternion(const vector::cptr&, const float);
			quaternion(const float, const float, const float, const float);

			//Setters ****************************************************************
			void normalize(void);
			void conjugate(void);
			void inverse(void);
			void multiply(const float);

			//Getters ****************************************************************
			float mag(void);

	}; //End quaternion class.

	//Functions for quaternion math **********************************************
	void normalize(const quaternion::cptr&, quaternion::cptr&);
	void conjugate(const quaternion::cptr&, quaternion::cptr&);
	void inverse(const quaternion::cptr&, quaternion::cptr&);
	void sum(const quaternion::cptr&, const quaternion::cptr&, quaternion::cptr&);
	void subtract(const quaternion::cptr&, const quaternion::cptr&, quaternion::cptr&);
	void multiply(const quaternion::cptr&, const float, quaternion::cptr&);
	void cross(const quaternion::cptr&, const quaternion::cptr&, quaternion::cptr&); //Quaternion cross product.
	float dot(const quaternion::cptr&, const quaternion::cptr&);

	//Functions for orientation math *********************************************
	void rotate(const quaternion::cptr&, const vector::cptr&, vector::cptr&);
	void q2euler(const quaternion::cptr&, vector::cptr&);
	void euler2q(const vector::cptr&, quaternion::cptr&);

}; //End vmath namespace.

#endif
