/**
 * file vmath.hpp
 *
 * @brief This file belongs to the qc_library project. It provides functions
 *        for Vector and rotation math.
 *
 * @author 	Rob Mertens
 * @date		27/10/2017
 */

#ifndef qc_MATH_HELPERS_HPP
#define qc_MATH_HELPERS_HPP

#include <math.h>

namespace qc
{

namespace math_helpers
{

	/**
	 * @brief
	 */
	class Vector
	{

		public:

			/** Typedefs ************************************************************/
			/**
			 * @brief
			 */
			typedef Vector * Ptr; //std::shared_ptr<Vector> ptr;

			/**
			 * @brief
			 */
			typedef Vector * const CPtr; //std::shared_ptr<const Vector> cptr;

			/** Constructors/destructors/overloading ********************************/
			/**
			 * @brief
			 */
			Vector(void);

			/**
			 * @brief
			 */
			Vector(const double, const double, const double);

			/**
			 * @brief Copy-constructor.
			 */
			Vector(const Vector&);

			/**
			 * @brief Destructor for a Vector instance.
			 */
			virtual ~Vector(void);

			/**
			 * @brief Operator overloading.
			 */
			Vector& operator=(const Vector&);

			/** Primitives **********************************************************/
			/**
			 * @brief Zero vector.
			 */
			static Vector Zero(void);

			/**
			 * @brief Unit vector x-axis.
			 */
			static Vector UnitX(void);

			/**
			 * @brief Unit vector y-axis.
			 */
			static Vector UnitY(void);

			/**
			 * @brief Unit vector z-axis.
			 */
			static Vector UnitZ(void);

			/** Member access *******************************************************/
			/**
			 * @brief
			 */
			void x(const double);

			/**
			 * @brief
			 */
			void y(const double);

			/**
			 * @brief
			 */
			void z(const double);

			/**
			 * @brief
			 */
			double x(void);

			/**
			 * @brief
			 */
			double y(void);

			/**
			 * @brief
			 */
			double z(void);

			/**
			 * @brief
			 */
			double magnitude(void);

			/** Vector operations ***************************************************/
			/**
			 * @brief Method for normalizing the Vector as Euclidean norm.
			 */
			void normalize(void);

			/**
			 * @brief
			 * @param
			 * @param[out]
			 */
			Vector& operator+=(const Vector&);

			/**
			 * @brief
			 * @param
			 * @param[out]
			 */
			Vector& operator-=(const Vector&);

			/**
			 * @brief
			 * @param
			 * @param[out]
			 */
			Vector& operator*=(const double);

			/**
			 * @brief
			 * @param
			 * @param[out]
			 */
			Vector& operator/=(const double);

			/**
			 * @brief Function for operator "sum" overloading.
			 * @param an arbitrary Vector.
			 * @param[out] a Vector copy.
			 */
			friend Vector operator+(const Vector&, const Vector&);

			/**
			 * @brief Function for operator "sum" overloading.
			 * @param an arbitrary Vector.
			 * @param[out] a Vector copy.
			 */
			friend Vector operator-(const Vector&, const Vector&);

			/**
			 * @brief Function for operator "multiplication" overloading.
			 * @param an arbitrary Vector.
			 * @param[out] a Vector copy.
			 */
			friend Vector operator*(const Vector&, const double);

			/**
			 * @brief Function for operator "multiplication" overloading.
			 * @param an arbitrary Vector.
			 * @param[out] a Vector copy.
			 */
			friend Vector operator*(const double, const Vector&);

			/**
			 * @brief Function for operator "sum" overloading.
			 * @param an arbitrary Vector.
			 * @param[out] a Vector copy.
			 */
			friend Vector operator/(const Vector&, const double);

			/**
			 * @brief
			 * @param
			 * @param
			 * @param[out]
			 */
			friend double dot(const Vector&, const Vector&);

			/**
			 * @brief
			 * @param
			 * @param
			 * @param[out]
			 */
			friend Vector cross(const Vector&, const Vector&);

		private:
			/**
			 * @brief Vector x-value.
			 */
			double x_;

			/**
			 * @brief Vector y-value.
			 */
			double y_;

			/**
			 * @brief Vector z-value.
			 */
			double z_;

			/**
			 * @brief The vector magnitude.
			 *
			 * NOTE::we want to calculate this at runtime only.
			 */
			double m_ __attribute__ ((deprecated));

	}; //End Vector class.

	/**
	 * @brief
	 */
	class Quaternion
	{

		public:

			/** Typedefs ************************************************************/
			/**
			 * @brief
			 */
			typedef Quaternion * Ptr; //std::shared_ptr<Quaternion> ptr;

			/**
			 * @brief
			 */
			typedef Quaternion * const CPtr; //std::shared_ptr<const Quaternion> cptr;

			/** Constructors/destructors/overloading ********************************/
			/**
			 * @brief
			 */
			Quaternion(void);

			/**
			 * @brief Constructor for a Quaternion based on rotation angle and Vector.
			 *
			 * @param The rotation angle double.
			 * @param The rotation Vector.
			 */
			Quaternion(const Vector&, const double);

			/**
			 * @brief Constructor for a Quaternion.
			 *
			 * @param The w value double.
			 * @param The x value double.
			 * @param The y value double.
			 * @param The z value double.
			 */
			Quaternion(const double, const double, const double,
				const double);

			/**
			 * @brief Copy-constructor.
			 */
			Quaternion(const Quaternion&);

			/**
			 * @brief Destructor for a Quaternion instance.
			 */
			virtual ~Quaternion(void);

			/**
			 * @brief Operator overloading.
			 */
			Quaternion& operator=(const Quaternion&);

			/** Primitives **********************************************************/
			/**
			 * @brief Zero vector.
			 */
			static Quaternion Zero(void);

			/**
			 * @brief Unit quaternion w-axis is real quaternion.
			 * @return The quaternion.
			 */
			static Quaternion UnitW(void);

			/**
			 * @brief Unit quaternion x-axis.
			 * @return The quaternion.
			 */
			static Quaternion UnitX(void);

			/**
			 * @brief Unit quaternion y-axis.
			 * @return The quaternion.
			 */
			static Quaternion UnitY(void);

			/**
			 * @brief Unit quaternion z-axis.
			 * @return The quaternion.
			 */
			static Quaternion UnitZ(void);

			/** Member access *******************************************************/
			/**
			 * @brief
			 */
			void w(const double);

			/**
			 * @brief
			 */
			void x(const double);

			/**
			 * @brief
			 */
			void y(const double);

			/**
			 * @brief
			 */
			void z(const double);

			/**
			 * @brief
			 */
			double w(void);

			/**
			 * @brief
			 */
			double x(void);

			/**
			 * @brief
			 */
			double y(void);

			/**
			 * @brief
			 */
			double z(void);

			/**
			 * @brief Method for calculating the magnitude.
			 *
			 * @return The Quaternion magnitude.
			 */
			double magnitude(void);

			/** Quaternion operators ************************************************/
			/**
			 * Method for normalizing the Quaternion as Euclidean norm.
			 */
			void normalize(void);

			/**
			 * Method for obtaining the conjugate Quaternion.
			 */
			void conjugate(void);

			/**
			 * Method for obtaining the inverse Quaternion.
			 */
			virtual void inverse(void);

			/**
			 * @brief
			 */
			virtual Vector rotate(const Vector&);

			/**
			 * @brief
			 * @param
			 * @param[out]
			 */
			Quaternion& operator+=(const Quaternion&);

			/**
			 * @brief
			 * @param
			 * @param[out]
			 */
			Quaternion& operator-=(const Quaternion&);

			/**
			 * @brief
			 * @param
			 * @param[out]
			 */
			Quaternion& operator*=(const double);

			/**
			 * @brief
			 * @param
			 * @param[out]
			 */
			Quaternion& operator/=(const double);

			/**
			 * @brief
			 */
			friend Quaternion normalize(const Quaternion&);

			/**
			 * @brief
			 */
			friend Quaternion conjugate(const Quaternion&);

			/**
			 * @brief Function for operator "sum" overloading.
			 * @param an arbitrary Quaternion.
			 * @param[out] a Quaternion copy.
			 */
			friend Quaternion operator+(const Quaternion&, const Quaternion&);

			/**
			 * @brief
			 * @param
			 * @param
			 * @param[out]
			 */
			friend Quaternion operator-(const Quaternion&, const Quaternion&);

			/**
			 * @brief
			 * @param
			 * @param
			 * @param[out]
			 */
			friend Quaternion operator*(const Quaternion&, const double);

			/**
			 * @brief
			 * @param
			 * @param
			 * @param[out]
			 */
			friend Quaternion operator*(const double, const Quaternion&);

			/**
			 * @brief
			 * @param
			 * @param
			 * @param[out]
			 */
			friend Quaternion operator/(const Quaternion&, const double);

			/**
			 * @brief
			 * @param
			 * @param
			 * @param[out]
			 */
			friend double dot(const Quaternion&, const Quaternion&);

			/**
			 * @brief
			 * @param
			 * @param
			 * @param[out]
			 */
			friend Quaternion hamilton(const Quaternion&, const Quaternion&);

			/**
			 * @brief
			 * @param
			 * @param
			 * @param[out]
			 */
			friend Quaternion lerp(const Quaternion&, const Quaternion&,
				const double);

			/**
			 * @brief TODO::
			 * @param
			 * @param
			 * @param[out]
			 */
			friend Quaternion slerp(const Quaternion&, const Quaternion&,
				const double);

		private:

			/** Variables ***********************************************************/
			/**
			 * @brief
			 */
			double w_;

			/**
			 * @brief
			 */
			double x_;

			/**
			 * @brief
			 */
			double y_;

			/**
			 * @brief
			 */
			double z_;

			/**
			 * @brief TODO::work with real and Vector part.
			 */
			//Vector v_;

			/**
			 * @brief _deprecated_
			 */
			//double m_;

			/**
			 * @brief _deprecated_
			 */
			//friend class Vector;

			/** Statics *************************************************************/
			/**
			 * @brief The zero quaternion.
			 */
			//static const Quaternion zero_ = new Quaternion(1.0, 0.0, 0.0, 0.0);

			/**
			 * @brief The zero quaternion.
			 */
			//const static Quaternion unitw_ = new Quaternion(1.0, 0.0, 0.0, 0.0);

			/**
			 * @brief The zero quaternion.
			 */
			//const static Quaternion unitx_ = new Quaternion(0.0, 1.0, 0.0, 0.0);

			/**
			 * @brief The zero quaternion.
			 */
			//const static Quaternion unity_ = new Quaternion(0.0, 0.0, 1.0, 0.0);

			/**
			 * @brief The zero quaternion.
			 */
			//const static Quaternion unitz_ = new Quaternion(0.0, 0.0, 0.0, 1.0);

	}; //End Quaternion class.

	/** External operations *****************************************************/
	/**
	 * @brief Method for obtaining the corresponding XYZ-Euler angles from a
	 *				Quaternion rotation.
	 *
	 * NOTE::this should be part of a base Rotation class.
	 *
	 * @param
	 * @return result The result Vector.
	 */
	Vector q2euler(const Quaternion&);

	/**
	 * @brief Method for obtaining the corresponding XYZ-Euler angles from a
	 *				Quaternion rotation.
	 *
	 * TODO::create this function.
	 * NOTE::this should be part of a base Rotation class.
	 *
	 * @param
	 * @return result The result Vector.
	 */
	Quaternion euler2q(const Vector&);

}; //End namespace math_helpers.

}; //End namespace qc.

#endif //End wrapper qc_MATH_HELPERS_HPP.
