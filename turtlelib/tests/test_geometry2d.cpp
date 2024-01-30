#include "turtlelib/geometry2d.hpp"

#include <catch2/catch_test_macros.hpp> // Test Cases
#include <catch2/matchers/catch_matchers_floating_point.hpp> // Floating point matchers
#include <sstream> // for testing stream insertion/extraction

using namespace turtlelib;
// Testing 0, pi, -pi, -pi/4, 3pi/2, -5pi/2
TEST_CASE("Normalize angle", "[geometry2d]"){
    double pi = turtlelib::PI;
    double zero = 0.0; // Need to declare as double or matchers will conflict between double and float
    REQUIRE_THAT(normalize_angle(0.0), Catch::Matchers::WithinRel(zero));
    REQUIRE_THAT(normalize_angle(pi), Catch::Matchers::WithinRel(pi));
    REQUIRE_THAT(normalize_angle(-pi), Catch::Matchers::WithinRel(pi));
    REQUIRE_THAT(normalize_angle(-pi/4.0), Catch::Matchers::WithinRel(-pi/4.0));
    REQUIRE_THAT(normalize_angle(3.0*pi/2.0), Catch::Matchers::WithinRel(-pi/2.0));
    REQUIRE_THAT(normalize_angle(-5.0*pi/2.0), Catch::Matchers::WithinRel(-pi/2.0));
}

TEST_CASE("Stream insertion operator <<", "[point]"){
    turtlelib::Point2D point = {-1.3, 2.7};
    std::stringstream strStream;
    strStream << point;
    std::string str = "[-1.3 2.7]";
    REQUIRE(strStream.str() == str);
}

TEST_CASE("Stream extraction operator >>", "[point]"){
    turtlelib::Point2D emptyP;
    std::stringstream strStream1;
    std::stringstream strStream2;

    // Test bracket extraction
    strStream1 << "[-1.3 2.7]";
    strStream1 >> emptyP;
    REQUIRE_THAT(emptyP.x, Catch::Matchers::WithinRel(-1.3));
    REQUIRE_THAT(emptyP.y, Catch::Matchers::WithinRel(2.7));

    // Test no bracket extraction
    emptyP = {0.0, 0.0};
    strStream2 << "-1.3 2.7";
    strStream2 >> emptyP;
    REQUIRE_THAT(emptyP.x, Catch::Matchers::WithinRel(-1.3));
    REQUIRE_THAT(emptyP.y, Catch::Matchers::WithinRel(2.7));
}

TEST_CASE("Vector between 2 points: Operator-", "[geometry2d]"){
    turtlelib::Point2D p1 = {0.0, 0.0};
    turtlelib::Point2D p2 = {0.0, -4.0};
    turtlelib::Point2D p3 = {5.0, 5.0};

    turtlelib::Vector2D v12 = p2-p1; // {0.0, -4.0}
    turtlelib::Vector2D v13 = p3-p1; // {5.0, 5.0}
    turtlelib::Vector2D v32 = p2-p3; // {-5.0, -9.0}

    REQUIRE_THAT(v12.x, Catch::Matchers::WithinRel(0.0));
    REQUIRE_THAT(v12.y, Catch::Matchers::WithinRel(-4.0));
    REQUIRE_THAT(v13.x, Catch::Matchers::WithinRel(5.0));
    REQUIRE_THAT(v13.y, Catch::Matchers::WithinRel(5.0));
    REQUIRE_THAT(v32.x, Catch::Matchers::WithinRel(-5.0));
    REQUIRE_THAT(v32.y, Catch::Matchers::WithinRel(-9.0));
}

TEST_CASE("Adding a vector to a point", "[geometry2d]"){
    turtlelib::Point2D p1 = {0.0, 0.0};
    turtlelib::Point2D p2 = {1.2, -4.8};
    turtlelib::Vector2D v1 = {1.3, -0.5};
    turtlelib::Vector2D v2 = {-5.7, 3.1};

    turtlelib::Point2D p1v1 = p1+v1; // {1.3, -0.5}
    turtlelib::Point2D p1v2 = p1+v2; // {-5.7, 3.1}
    turtlelib::Point2D p2v1 = p2+v1; // {2.5, -5.3}
    turtlelib::Point2D p2v2 = p2+v2; // {-4.5, -1.7}

    REQUIRE_THAT(p1v1.x, Catch::Matchers::WithinRel(1.3));
    REQUIRE_THAT(p1v1.y, Catch::Matchers::WithinRel(-0.5));
    REQUIRE_THAT(p1v2.x, Catch::Matchers::WithinRel(-5.7));
    REQUIRE_THAT(p1v2.y, Catch::Matchers::WithinRel(3.1));
    REQUIRE_THAT(p2v1.x, Catch::Matchers::WithinRel(2.5));
    REQUIRE_THAT(p2v1.y, Catch::Matchers::WithinRel(-5.3));
    REQUIRE_THAT(p2v2.x, Catch::Matchers::WithinRel(-4.5));
    REQUIRE_THAT(p2v2.y, Catch::Matchers::WithinRel(-1.7));
}

TEST_CASE("Normalize vector", "[vector]"){
    turtlelib::Vector2D v = {5.0, 5.0};
    turtlelib::Vector2D v_hat = normalize(v);
    REQUIRE_THAT(v_hat.x, Catch::Matchers::WithinRel(0.707, 0.001));
    REQUIRE_THAT(v_hat.y, Catch::Matchers::WithinRel(0.707, 0.001));
}

TEST_CASE("Stream insertion operator <<", "[vector]"){
    turtlelib::Vector2D vector = {-1.3, 2.7};
    std::stringstream strStream;
    strStream << vector;
    std::string str = "[-1.3 2.7]";
    REQUIRE(strStream.str() == str);
}

TEST_CASE("Stream extraction operator >>", "[vector]"){
    turtlelib::Point2D emptyV;
    std::stringstream strStream1;
    std::stringstream strStream2;

    // Test bracket extraction
    strStream1 << "[-1.3 2.7]";
    strStream1 >> emptyV;
    REQUIRE_THAT(emptyV.x, Catch::Matchers::WithinRel(-1.3));
    REQUIRE_THAT(emptyV.y, Catch::Matchers::WithinRel(2.7));

    // Test no bracket extraction
    emptyV = {0.0, 0.0};
    strStream2 << "-1.3 2.7";
    strStream2 >> emptyV;
    REQUIRE_THAT(emptyV.x, Catch::Matchers::WithinRel(-1.3));
    REQUIRE_THAT(emptyV.y, Catch::Matchers::WithinRel(2.7));
}

TEST_CASE("Add operator +", "[vector]"){
    Vector2D v1 = {-1.3, 2.7};
    Vector2D v2 = {0.8, 5.2};
    Vector2D v3 = {-3.1, -5.7};

    Vector2D v12 = v1 + v2;
    REQUIRE_THAT(v12.x, Catch::Matchers::WithinRel(-0.5));
    REQUIRE_THAT(v12.y, Catch::Matchers::WithinRel(7.9));

    Vector2D v13 = v1 + v3;
    REQUIRE_THAT(v13.x, Catch::Matchers::WithinRel(-4.4));
    REQUIRE_THAT(v13.y, Catch::Matchers::WithinRel(-3.0));

    Vector2D v23 = v2 + v3;
    REQUIRE_THAT(v23.x, Catch::Matchers::WithinRel(-2.3));
    REQUIRE_THAT(v23.y, Catch::Matchers::WithinRel(-0.5));
    
    Vector2D v32 = v3 + v2;
    REQUIRE_THAT(v23.x, Catch::Matchers::WithinRel(v32.x));
    REQUIRE_THAT(v23.y, Catch::Matchers::WithinRel(v32.y));
}

TEST_CASE("Add operator +=", "[vector]"){
    Vector2D v1 = {-1.3, 2.7};
    Vector2D v2 = {0.8, 5.2};
    Vector2D v3 = {-3.1, -5.7};

    v1 += v2;
    REQUIRE_THAT(v1.x, Catch::Matchers::WithinRel(-0.5));
    REQUIRE_THAT(v1.y, Catch::Matchers::WithinRel(7.9));

    v1 += v3;
    REQUIRE_THAT(v1.x, Catch::Matchers::WithinRel(-3.6));
    REQUIRE_THAT(v1.y, Catch::Matchers::WithinRel(2.2));
}

TEST_CASE("Subtract Operator -", "[vector]"){
    Vector2D v1 = {-1.3, 2.7};
    Vector2D v2 = {0.8, 5.2};
    Vector2D v3 = {-3.1, -5.7};

    Vector2D v12 = v1 - v2;
    REQUIRE_THAT(v12.x, Catch::Matchers::WithinRel(-2.1));
    REQUIRE_THAT(v12.y, Catch::Matchers::WithinRel(-2.5));

    Vector2D v13 = v1 - v3;
    REQUIRE_THAT(v13.x, Catch::Matchers::WithinRel(1.8));
    REQUIRE_THAT(v13.y, Catch::Matchers::WithinRel(8.4));

    Vector2D v23 = v2 - v3;
    REQUIRE_THAT(v23.x, Catch::Matchers::WithinRel(3.9));
    REQUIRE_THAT(v23.y, Catch::Matchers::WithinRel(10.9));
    
    Vector2D v32 = v3 - v2;
    REQUIRE_THAT(v32.x, Catch::Matchers::WithinRel(-3.9));
    REQUIRE_THAT(v32.y, Catch::Matchers::WithinRel(-10.9));
}

TEST_CASE("Subtract Operator -=", "[vector]"){
    Vector2D v1 = {-1.3, 2.7};
    Vector2D v2 = {0.8, 5.2};
    Vector2D v3 = {-3.1, -5.7};

    v1 -= v2;
    REQUIRE_THAT(v1.x, Catch::Matchers::WithinRel(-2.1));
    REQUIRE_THAT(v1.y, Catch::Matchers::WithinRel(-2.5));

    v1 -= v3;
    REQUIRE_THAT(v1.x, Catch::Matchers::WithinRel(1.0));
    REQUIRE_THAT(v1.y, Catch::Matchers::WithinRel(3.2));
}

TEST_CASE("Multiply operator *", "[vector]"){
    Vector2D v = {-1.3, 2.7};

    Vector2D res = v*1.0;
    REQUIRE_THAT(res.x, Catch::Matchers::WithinRel(-1.3));
    REQUIRE_THAT(res.y, Catch::Matchers::WithinRel(2.7));

    res = v*2.0;
    REQUIRE_THAT(res.x, Catch::Matchers::WithinRel(-2.6));
    REQUIRE_THAT(res.y, Catch::Matchers::WithinRel(5.4));

    res = -2.0*v;
    REQUIRE_THAT(res.x, Catch::Matchers::WithinRel(2.6));
    REQUIRE_THAT(res.y, Catch::Matchers::WithinRel(-5.4));

    res = v*0.1;
    REQUIRE_THAT(res.x, Catch::Matchers::WithinRel(-.13));
    REQUIRE_THAT(res.y, Catch::Matchers::WithinRel(.27));

    res = 1.5*v;
    REQUIRE_THAT(res.x, Catch::Matchers::WithinRel(-1.95));
    REQUIRE_THAT(res.y, Catch::Matchers::WithinRel(4.05));
}

TEST_CASE("Multiple Operator *=", "[vector]"){
    Vector2D v = {-1.3, 2.7};
    
    v *= 1.0;
    REQUIRE_THAT(v.x, Catch::Matchers::WithinRel(-1.3));
    REQUIRE_THAT(v.y, Catch::Matchers::WithinRel(2.7));

    v *= -2.0;
    REQUIRE_THAT(v.x, Catch::Matchers::WithinRel(2.6));
    REQUIRE_THAT(v.y, Catch::Matchers::WithinRel(-5.4));

    v *= 0.1;
    REQUIRE_THAT(v.x, Catch::Matchers::WithinRel(.26));
    REQUIRE_THAT(v.y, Catch::Matchers::WithinRel(-.54));

    v *= -0.5;
    REQUIRE_THAT(v.x, Catch::Matchers::WithinRel(-.13));
    REQUIRE_THAT(v.y, Catch::Matchers::WithinRel(.27));
}

TEST_CASE("Dot Product", "[vector]"){
    Vector2D v1 = {-1.3, 2.7};
    Vector2D v2 = {0.8, 5.2};
    Vector2D v3 = {-3.1, -5.7};

    double res12 = dot(v1, v2);
    REQUIRE_THAT(res12, Catch::Matchers::WithinRel(13.0));

    double res23 = dot(v2, v3);
    REQUIRE_THAT(res23, Catch::Matchers::WithinRel(-32.12));

    double res13 = dot(v1, v3);
    REQUIRE_THAT(res13, Catch::Matchers::WithinRel(-11.36));

    double res31 = dot(v3, v1);
    REQUIRE_THAT(res31, Catch::Matchers::WithinRel(res13));
}

TEST_CASE("Magnitude", "[vector]"){
    Vector2D v1 = {-1.3, 2.7};
    REQUIRE_THAT(magnitude(v1), Catch::Matchers::WithinRel(2.996, 0.001));

    Vector2D v2 = {0.8, 5.2};
    REQUIRE_THAT(magnitude(v2), Catch::Matchers::WithinRel(5.261, 0.001));

    Vector2D v3 = {-3.1, -5.7};
    REQUIRE_THAT(magnitude(v3), Catch::Matchers::WithinRel(6.488, 0.001));

    Vector2D v4 = {3.0, 4.0};
    REQUIRE_THAT(magnitude(v4), Catch::Matchers::WithinRel(5.0));
}

TEST_CASE("Angle", "[vector]"){
    Vector2D v1 = {-1.3, 2.7};
    Vector2D v2 = {0.8, 5.2};
    Vector2D v3 = {-3.1, -5.7};

    double ang12 = angle(v1, v2);
    REQUIRE_THAT(ang12, Catch::Matchers::WithinRel(0.601, 0.001));

    double ang13 = angle(v1, v3);
    REQUIRE_THAT(ang13, Catch::Matchers::WithinRel(2.195, 0.001));

    double ang23 = angle(v2, v3);
    REQUIRE_THAT(ang23, Catch::Matchers::WithinRel(2.796, 0.001));

    double ang32 = angle(v3, v2);
    REQUIRE_THAT(ang32, Catch::Matchers::WithinRel(ang23, 0.001));
}