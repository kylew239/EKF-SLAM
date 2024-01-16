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
