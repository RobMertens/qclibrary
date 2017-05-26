#ifndef _QCMATH_H_
#define _QCMATH_H_

class vector
{
	public:
		//Variables ******************************************************************
		float x;
		float y;
		float z;
		float m;

		//Constructors ***************************************************************
		vector();
		vector(float, float, float);

		//Setters ********************************************************************
		vector norm(void);

		//Getters ********************************************************************
		float mag(void);
		float dot(vector);

		vector sum(vector);
		vector substract(vector);
		vector multiply(float);
		vector cross(vector);
	
	private:

};

class quaternion
{
	public:
		//Variables ******************************************************************
		float w;
		float x;
		float y;
		float z;
		float m; 

		//Constructors ***************************************************************
		quaternion();
		quaternion(float, vector);
		quaternion(float, float, float, float);

		//Setters ********************************************************************
		quaternion norm(void);
		quaternion conj(void);
		quaternion inv(void);

		//Getters ********************************************************************     
		float mag(void);
		float dot(quaternion);    
		
		quaternion sum(quaternion);
		quaternion multiply(float);
		quaternion cross(quaternion); //Quaternion cross product.

		vector rotate(vector);
		vector q2euler(void);

	private:

};

#endif

