#ifndef _MATH_H_
#define _MATH_H_

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
		void norm();

		//Getters ********************************************************************
		float dot(vector);

		vector sum(vector);
		vector substract(vector);
		vector multiply(float);
		vector cross(vector);
	
	private:
		float mag();

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
		void norm();
		void conj();
		void inv();

		//Getters ********************************************************************     
		float dot(quaternion);    

		quaternion sum(quaternion);
		quaternion multiply(float);
		quaternion cross(quaternion); //Quaternion cross product.

		vector rotate(vector);
		vector q2euler();

	private:
		float mag();
};

#endif

