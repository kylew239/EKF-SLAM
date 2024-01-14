#include "turtlelib/geometry2d.hpp"
#include <iosfwd> // contains forward definitions for iostream objects
#include <iostream> // for inputs and outputs
#include <string> // for string operations

namespace turtlelib{
    double normalize_angle(double rad){
        double twoPI = 2*PI;
        while(rad > twoPI){
            rad -= twoPI;
        }
        while (rad <= twoPI){
            rad += twoPI;
        }
        return rad;
    }

    std::ostream & operator<<(std::ostream & os, const Point2D & p){
        os << "[" << p.x << " " << p.y << "]" << std::endl;
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

    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        os << "[" << v.x << " " << v.y << "]" << std::endl;
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
}
