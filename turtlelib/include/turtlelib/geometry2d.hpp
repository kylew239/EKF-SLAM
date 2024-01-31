#ifndef TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
#define TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
/// \file
/// \brief Two-dimensional geometric primitives.

#include <cmath>  // contains math operations
#include <iostream> // for inputs and outputs


namespace turtlelib
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12){
        return std::abs(d1 - d2) < epsilon;
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    constexpr double deg2rad(double deg){
        return deg*PI/180.0;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad){
        return rad*180.0/PI;
    }

    /// \brief wrap an angle to (-PI, PI]
    /// \param rad (angle in radians)
    /// \return an angle equivalent to rad but in the range (-PI, PI]
    double normalize_angle(double rad);

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

    /// \brief a 2-Dimensional Point
    struct Point2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;
    };

    /// \brief output a 2 dimensional point as [xcomponent ycomponent]
    /// \param os - stream to output to
    /// \param p - the point to print
    std::ostream & operator<<(std::ostream & os, const Point2D & p);

    /// \brief input a 2 dimensional point
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param p [out] - output vector
    /// HINT: See operator>> for Vector2D
    std::istream & operator>>(std::istream & is, Point2D & p);

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// NOTE: Adding the operator overloads to the struct for easier access to Vector2D vars
        /// NOTE: This decision was influenced by the structure of the Transform2D class in se2d.hpp

        /// \brief Operator overload for adding 2 vectors tegether
        /// \param v The vector to be added
        /// \return A reference to the original, modified vector
        Vector2D & operator+=(const Vector2D & v);

        /// \brief Operator overload from subtracting vectors
        /// \param v The vector the subtract
        /// \return A reference to the original, modified vector
        Vector2D & operator-=(const Vector2D & v);

        /// \brief Operator overload for multiplying a vector with a scalar
        /// \param s scalar to be multiplied
        /// \return A reference to the original, modified vector
        Vector2D & operator*=(const double s);
    };

    /// \brief Subtracting one point from another yields a vector
    /// \param head point corresponding to the head of the vector
    /// \param tail point corresponding to the tail of the vector
    /// \return a vector that points from p1 to p2
    /// NOTE: this is not implemented in terms of -= because subtracting two Point2D yields a Vector2D
    Vector2D operator-(const Point2D & head, const Point2D & tail);

    /// \brief Adding a vector to a point yields a new point displaced by the vector
    /// \param tail The origin of the vector's tail
    /// \param disp The displacement vector
    /// \return the point reached by displacing by disp from tail
    /// NOTE: this is not implemented in terms of += because of the different types
    Point2D operator+(const Point2D & tail, const Vector2D & disp);

    /// \brief normalize a vecotr
    /// \param v vector to normalize
    /// \return a unit vector
    Vector2D normalize(const Vector2D & v);

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// \param os - stream to output to
    /// \param v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    ///
    /// The way input works is (more or less): what the user types is stored in a buffer until the user enters
    /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin).
    ///
    /// We have lower level control however. For example:
    /// peek looks at the next unprocessed character in the buffer without removing it
    /// get removes the next unprocessed character from the buffer.
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief Addes 2 vectors together, using +=
    /// \param v1 The first vector to be added
    /// \param v2 Thes second vector to be added
    /// \return A new vector that is the sum of v1 and v2
    Vector2D operator+(Vector2D v1, const Vector2D & v2);

    /// \brief Subtracts 2 vectors, using -=
    /// \param v1 The vector to subtract from
    /// \param v2 The vector to subtract 
    /// \return A new vector that is the difference bewteen v1 and v2
    Vector2D operator-(Vector2D v1, const Vector2D & v2);

    /// \brief Multiply a vector and a scalar
    /// \param v The vector to multiply
    /// \param s The scalar to multiply
    /// \return A new vector that is the product of v and s
    Vector2D operator*(Vector2D v, const double s);

    /// \brief Multiply a vector and a scalar, but with the scalar first
    /// \param s The scalar to multiply
    /// \param v The vector to multiply
    /// \return A new vector that is the product of v and s
    Vector2D operator*(const double s, Vector2D v);

    /// \brief Compute the dot product of two vectors
    /// \param v1 The first vector to use
    /// \param v2 The second vector to use
    /// \return The dot product value
    double dot(const Vector2D & v1, const Vector2D & v2);

    /// \brief Compute the magnitude of a vector
    /// \param v The vector to use
    /// \return The magnitude of the vector
    double magnitude(const Vector2D & v);

    /// \brief Compute the angle between two vectors
    /// \param v1 The first vector
    /// \param v2 The second Vector
    /// \return The angle bewteen the vectors, in radians
    double angle(const Vector2D & v1, const Vector2D & v2);
}

#endif
