#ifndef _MATH_H_
#define _MATH_H_

class vector
{
    
    public:
  //Variables ******************************************************************
      float _x;
      float _y;
      float _z;
      float _m;
      
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
      float getMagnitude();
    

};

class quaternion
{   
    public:
  //Variables ******************************************************************
      float _w;
      float _x;
      float _y;
      float _z;
      float _m; 

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
      
      vector getRotation(vector);
      vector getEulerAngles();
      
    private:
    	
      float getMagnitude();
    
};

#endif

