#include "turtlelib/geometry2d.hpp"
#include <iosfwd> // contains forward definitions for iostream objects
#include <string> // for string operations

namespace turtlelib{
    double normalize_angle(double rad){
        double twoPI = 2*PI;
        while(rad > PI){
            rad -= twoPI;
        }
        while (rad <= -PI){
            rad += twoPI;
        }
        return rad;
    }

    std::ostream & operator<<(std::ostream & os, const Point2D & p){
        os << "[" << p.x << " " << p.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Point2D & p){
        // if point inputted as [x y] format
        if(is.peek() == '['){
            is.get();   // Get rid of the [ at the start
        }
        is >> p.x >> p.y; // Read double values, stopping at spaces, and store into variable
        return is;
    }

    Vector2D operator-(const Point2D & head, const Point2D & tail){
        return Vector2D{head.x-tail.x, head.y-tail.y};
    }

    Point2D operator+(const Point2D & tail, const Vector2D & disp){
        return Point2D{tail.x+disp.x, tail.y+disp.y};
    }

    Vector2D normalize(const Vector2D & v){
        double mag = std::sqrt(std::pow(v.x, 2.0) + std::pow(v.y, 2.0));
        return Vector2D{v.x/mag, v.y/mag};
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        os << "[" << v.x << " " << v.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v){
        // if point inputted as [x y] format
        if(is.peek() == '['){
            is.get();   // Get rid of the [ at the start
        }
        is >> v.x >> v.y; // Read double values, stopping at spaces, and store into variable
        return is;
    }

    Vector2D & Vector2D::operator+=(const Vector2D & v){
        // can do += because these are all doubles
        x += v.x;
        y += v.y;
        return *this;
    }

    Vector2D & Vector2D::operator-=(const Vector2D & v){
        // can do += because these are all doubles
        x -= v.x;
        y -= v.y;
        return *this;
    }

    Vector2D & Vector2D::operator*=(const double s){
        // can do += because these are all doubles
        x *= s;
        y *= s;
        return *this;
    }

    Vector2D operator+(Vector2D v1, const Vector2D & v2){
        return v1+=v2;
    }

    Vector2D operator-(Vector2D v1, const Vector2D & v2){
        return v1-=v2;
    }

    Vector2D operator*(Vector2D v, const double s){
        return v*=s;
    }

    Vector2D operator*(const double s, Vector2D v){
        return v*=s;
    }

    double dot(const Vector2D & v1, const Vector2D & v2){
        return v1.x*v2.x + v1.y*v2.y;
    }

    double magnitude(const Vector2D & v){
        return std::sqrt(v.x*v.x + v.y*v.y);
    }

    double angle(const Vector2D & v1, const Vector2D & v2){
        return std::acos(dot(v1, v2) / (magnitude(v1)*magnitude(v2)) );
    }
}
