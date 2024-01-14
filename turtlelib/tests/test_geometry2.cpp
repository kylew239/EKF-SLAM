#include "turtlelib/geometry2d.hpp"

#include <catch2/catch_test_macros.hpp> // Test Cases
#include <catch2/matchers/catch_matchers_floating_point.hpp> // Floating point matchers
#include <sstream> // for testing stream insertion/extraction

using namespace turtlelib;

// Testing 0, pi, -pi, -pi/4, 3pi/2, -5pi/2
TEST_CASE("Normalize angle", "[geometry2d]"){
    double pi = turtlelib::PI;
    double zero = 0.0; // Need to declare as double or matchers will conflict between double and float
    REQUIRE_THAT(normalize_angle(0), Catch::Matchers::WithinRel(zero));
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
    std::stringstream strStream;

    // Test bracket extraction
    std::string str1 = "[-1.3 2.7]";
    strStream << str1;
    strStream >> emptyP;
    REQUIRE_THAT(emptyP.x, Catch::Matchers::WithinRel(-1.3));
    REQUIRE_THAT(emptyP.y, Catch::Matchers::WithinRel(2.7));

    // Test no bracket extraction
    std::string str2 = "-1.3 2.7";
    strStream << str2;
    strStream >> emptyP;
    REQUIRE_THAT(emptyP.x, Catch::Matchers::WithinRel(-1.3));
    REQUIRE_THAT(emptyP.y, Catch::Matchers::WithinRel(2.7));
}

TEST_CASE("Vector between 2 points: Operator-", "[geometry2d]"){

}

TEST_CASE("Adding a vector to a point", "[geometry2d]"){

}

TEST_CASE("Stream insertion operator <<", "[vector]"){
    turtlelib::Vector2D vector = {-1.3, 2.7};
    std::stringstream strStream;
    strStream << vector;
    std::string str = "[-1.3 2.7]";
    REQUIRE(strStream.str() == str);
}

TEST_CASE("Stream extraction operator >>", "[point]"){
    turtlelib::Point2D emptyV;
    std::stringstream strStream;

    // Test bracket extraction
    std::string str1 = "[-1.3 2.7]";
    strStream << str1;
    strStream >> emptyV;
    REQUIRE_THAT(emptyV.x, Catch::Matchers::WithinRel(-1.3));
    REQUIRE_THAT(emptyV.y, Catch::Matchers::WithinRel(2.7));
    
    // Test no bracket extraction
    std::string str2 = "-1.3 2.7";
    strStream << str2;
    strStream >> emptyV;
    REQUIRE_THAT(emptyV.x, Catch::Matchers::WithinRel(-1.3));
    REQUIRE_THAT(emptyV.y, Catch::Matchers::WithinRel(2.7));
}