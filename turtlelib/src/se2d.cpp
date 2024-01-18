#include"turtlelib/se2d.hpp"
#include <iosfwd> // contains forward definitions for iostream objects
#include <string> // for string operations
#include <cmath>  // contains math operations


namespace turtlelib
{
    std::ostream & operator<<(std::ostream & os, const Twist2D & tw){
        os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & tw){
        // if twist inputted as [w x y] format
        if(is.peek() == '['){
            is.get();   // Get rid of the [ at the start
        }
        is >> tw.omega >> tw.x >> tw.y; // Read double values, stopping at spaces, and store into variable
        return is;
    }

    Transform2D::Transform2D(){
        theta = 0.0;
        x = 0.0;
        y = 0.0;
    }

    Transform2D::Transform2D(Vector2D trans){
        theta = 0.0;
        x = trans.x;
        y = trans.y;
    }

    Transform2D::Transform2D(double radians){
        theta = radians;
        x = 0.0;
        y = 0.0;
    }

    Transform2D::Transform2D(Vector2D trans, double radians){
        theta = radians;
        x = trans.x;
        y = trans.y;
    }

    Point2D Transform2D::operator()(Point2D p) const{
        return Point2D{x*std::cos(theta) - y*std::sin(theta) + p.x,
                       x*std::sin(theta) + y*std::cos(theta) + p.y};
    }

    Vector2D Transform2D::operator()(Vector2D v) const{
        return Vector2D{x*std::cos(theta) - y*std::sin(theta) + v.x,
                        x*std::sin(theta) + y*std::cos(theta) + v.y};
    }

    Twist2D Transform2D::operator()(Twist2D twist) const{
        return Twist2D{
            twist.omega,
            twist.x*std::cos(theta) - twist.y*std::sin(theta) + y*twist.omega,
            twist.x*std::cos(theta) + twist.y*std::sin(theta) - y*twist.omega
            };   
    }

    Transform2D Transform2D::inv() const{
        return Transform2D{Vector2D{-x*std::cos(theta)-y*std::sin(theta),
                                    -y*std::cos(theta)+x*std::sin(theta)},
                           -theta};
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        theta = theta + rhs.theta;
        x = x + rhs.x*std::cos(theta) - rhs.y*sin(theta);
        y = y + rhs.x*std::sin(theta) + rhs.y*cos(theta);
        return *this;
    }

    Vector2D Transform2D::translation() const{
        return Vector2D{x, y};
    }

    double Transform2D::rotation() const{
        return theta;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        os << "deg: " << rad2deg(tf.theta) << " x: " << tf.x << " y: " << tf.y;
        return os;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf){
        double deg, x, y;
        is >> deg >> x >> y;
        tf = Transform2D{Vector2D{x, y},
                         deg2rad(deg)};
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        return lhs*=rhs;
    }
}