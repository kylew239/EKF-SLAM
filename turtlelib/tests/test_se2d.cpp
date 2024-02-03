#include "turtlelib/se2d.hpp"

#include <catch2/catch_test_macros.hpp> // Test Cases
#include <catch2/matchers/catch_matchers_floating_point.hpp> // Floating point matchers
#include <sstream> // for testing stream insertion/extraction

using namespace turtlelib;

    TEST_CASE("Twist2D<<", "[se2d]") { // Max Palay
        Twist2D twist = {0, -0.203, 0.145};
        std::stringstream ss{""}; 
        ss << twist;
        REQUIRE(ss.str() == "[0 -0.203 0.145]");
    }

    TEST_CASE("Twist2D>>", "[se2d]") { // Max Palay
        std::stringstream ss("0.1 1.43 -0.2\n"); 
        Twist2D twist;
        ss >> twist;
        REQUIRE_THAT(twist.omega, Catch::Matchers::WithinAbs(0.1, 1.0E-8));
        REQUIRE_THAT(twist.x, Catch::Matchers::WithinAbs(1.43, 1.0E-8));
        REQUIRE_THAT(twist.y, Catch::Matchers::WithinAbs(-0.2, 1.0E-8));
        ss = std::stringstream("[0.1 1.43 -0.2]\n"); 
        twist = Twist2D{};
        ss >> twist;
        REQUIRE_THAT(twist.omega, Catch::Matchers::WithinAbs(0.1, 1.0E-8));
        REQUIRE_THAT(twist.x, Catch::Matchers::WithinAbs(1.43, 1.0E-8));
        REQUIRE_THAT(twist.y, Catch::Matchers::WithinAbs(-0.2, 1.0E-8));
    }

    TEST_CASE("Initializing transform", "[Transform2D]") {
        Transform2D tf1{0.32}; // can't use tf1 = {...} because explicit, can't use copy-list-initialization
        Transform2D tf2{Vector2D{3.1, -1.3}};
        Transform2D tf3 = {Vector2D{3.1, -1.3}, 0.32};
        Transform2D tf4;

        REQUIRE_THAT(tf1.rotation(), Catch::Matchers::WithinRel(0.32));
        REQUIRE_THAT(tf1.translation().x, Catch::Matchers::WithinRel(0.0));
        REQUIRE_THAT(tf1.translation().y, Catch::Matchers::WithinRel(0.0));

        REQUIRE_THAT(tf2.rotation(), Catch::Matchers::WithinRel(0.0));
        REQUIRE_THAT(tf2.translation().x, Catch::Matchers::WithinRel(3.1));
        REQUIRE_THAT(tf2.translation().y, Catch::Matchers::WithinRel(-1.3));

        REQUIRE_THAT(tf3.rotation(), Catch::Matchers::WithinRel(0.32));
        REQUIRE_THAT(tf3.translation().x, Catch::Matchers::WithinRel(3.1));
        REQUIRE_THAT(tf3.translation().y, Catch::Matchers::WithinRel(-1.3));

        REQUIRE_THAT(tf4.rotation(), Catch::Matchers::WithinRel(0.0));
        REQUIRE_THAT(tf4.translation().x, Catch::Matchers::WithinRel(0.0));
        REQUIRE_THAT(tf4.translation().y, Catch::Matchers::WithinRel(0.0));
    }

    TEST_CASE("Transform2D(point)", "[se2d]") { // Max Palay
        // identity
        Vector2D trans{0.0, 0.0};
        Transform2D tf{trans, 0.0};
        Point2D a{0.3, -0.1};
        Point2D res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.3, 1.0E-4));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.1, 1.0E-4));

        // pure translation
        trans = {0.2, -0.2};
        tf = Transform2D(trans, 0.0);
        a = {0.3, -0.1};
        res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.5, 1.0E-4));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.3, 1.0E-4));

        // pure rotation
        trans = {0.0, 0.0};
        tf = Transform2D(trans, 0.2);
        a = {0.1, -0.1};
        res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.1179, 1.0E-3));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.0781, 1.0E-4));

        // rotation & translation
        trans = {0.1, -0.1};
        tf = Transform2D(trans, 3.8);
        a = {0.1, -0.1};
        res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(-0.0403, 1.0E-3));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.0821, 1.0E-4));
    }

    TEST_CASE("Transform2D(vector)", "[se2d]") { // Max Palay
        // identity
        Vector2D trans{0.0, 0.0};
        Transform2D tf{trans, 0.0};
        Vector2D a{0.3, -0.1};
        Vector2D res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.3, 1.0E-4));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.1, 1.0E-4));

        // pure translation
        trans = {0.2, -0.2};
        tf = Transform2D(trans, 0.0);
        a = {0.3, -0.1};
        res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.3, 1.0E-4));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.1, 1.0E-4));

        // pure rotation
        trans = {0.0, 0.0};
        tf = Transform2D(trans, 0.2);
        a = {0.1, -0.1};
        res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.1179, 1.0E-3));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.0781, 1.0E-4));

        // rotation & translation
        trans = {0.1, -0.1};
        tf = Transform2D(trans, 3.8);
        a = {0.1, -0.1};
        res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(-0.1403, 1.0E-3));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(0.0179, 1.0E-4));
    }

    TEST_CASE("Transforming a twist", "[se2d]") {
        Transform2D tf = {Vector2D{0.2, 1.1}, 0.34};
        Twist2D tw = {1.0, 0.707, 0.707};
        Twist2D ans = tf(tw);
        REQUIRE_THAT(ans.omega, Catch::Matchers::WithinRel(1.0));
        REQUIRE_THAT(ans.x, Catch::Matchers::WithinRel(1.531, 0.001));
        REQUIRE_THAT(ans.y, Catch::Matchers::WithinRel(0.702, 0.001));
    }

    TEST_CASE("Inverting a transform", "[transform2D]") {
        Transform2D tf = {Vector2D{-1.2, 0.35}, 0.29};
        Transform2D tfInv = tf.inv();
        REQUIRE_THAT(tfInv.rotation(), Catch::Matchers::WithinRel(-0.29));
        REQUIRE_THAT(tfInv.translation().x, Catch::Matchers::WithinRel(1.050, 0.001));
        REQUIRE_THAT(tfInv.translation().y, Catch::Matchers::WithinRel(-0.679, 0.001));
    }

    TEST_CASE("Transform2D*=", "[se2d]") { // Max Palay
        // try composition with the inverse
        Vector2D trans{1.2, -2.2};
        Transform2D tf{trans, 0.6};
        Transform2D inv;
        inv = tf.inv();
        tf *= inv;
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1.0E-3));
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.0, 1.0E-3));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0E-3));

        // try composition with another regular tf
        Vector2D trans1{1.2, -2.2};
        Transform2D tf1{trans1, 0.6};
        Vector2D trans2{0.3, 4.1};
        Transform2D tf2{trans2, -0.1};
        tf1 *= tf2;
        REQUIRE_THAT(tf1.rotation(), Catch::Matchers::WithinAbs(0.5, 1.0E-3));
        REQUIRE_THAT(tf1.translation().x, Catch::Matchers::WithinAbs(-0.867, 1.0E-3));
        REQUIRE_THAT(tf1.translation().y, Catch::Matchers::WithinAbs(1.353, 1.0E-3));
    }

    TEST_CASE("Obtain transform components", "[Transform2D]"){
        Transform2D tf = {Vector2D{2.1, -4.3}, 2.3};
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinRel(2.3));
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinRel(2.1));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinRel(-4.3));
    }

    TEST_CASE("Stream insertion operator <<", "[transform2D]"){
        Transform2D tf = {Vector2D{1.4, -2.7}, deg2rad(-52)};
        std::stringstream strStream; 
        strStream << tf;
        REQUIRE(strStream.str() == "deg: -52 x: 1.4 y: -2.7");
    }

    TEST_CASE("Stream extraction operator >>", "[transform2D]"){
        std::stringstream strStream1; 
        std::stringstream strStream2;
        std::stringstream strStream3;
        Transform2D tf;

        // Testing seperated by space
        strStream1 << "49 -0.2 -4.1";
        strStream1 >> tf;
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinRel(-0.2));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinRel(-4.1));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinRel(deg2rad(49)));

        // Testing seperated by newline
        tf = Transform2D();
        strStream2 << "49\n-0.2\n-4.1\n";
        strStream2 >> tf;
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinRel(-0.2));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinRel(-4.1));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinRel(deg2rad(49)));

        strStream3 << "deg: 49 x: -0.2 y: -4.1";
        strStream3 >> tf;
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinRel(-0.2));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinRel(-4.1));
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinRel(deg2rad(49)));
    }

    TEST_CASE("Multiply operator *", "[transform2D]"){
        Transform2D tf1 = {Vector2D{1.2, -2.2}, 0.6};
        Transform2D tf2 = {Vector2D{0.3, 4.1}, -0.1};
        Transform2D tf3 = tf1 * tf2;
        REQUIRE_THAT(tf3.rotation(), Catch::Matchers::WithinRel(0.5));
        REQUIRE_THAT(tf3.translation().x, Catch::Matchers::WithinRel(-0.867, 0.001));
        REQUIRE_THAT(tf3.translation().y, Catch::Matchers::WithinRel(1.353, 0.001));
    }

    TEST_CASE("Integrating twist", "[twist]"){
        SECTION("Pure translation"){
            Twist2D tw = {0.0, -0.32, 1.35};
            Transform2D tf = integrate_twist(tw);

            REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinRel(tw.x, 0.001));
            REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinRel(tw.y, 0.001));
            REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinRel(0.0, 0.001));
        }

        SECTION("Pure rotation"){
            Twist2D tw = {0.23, 0.0, 0.0};
            Transform2D tf = integrate_twist(tw);

            REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinRel(0.0, 0.001));
            REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinRel(0.0, 0.001));
            REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinRel(0.23, 0.001));
        }

        SECTION("Rotation and translation"){
            Twist2D tw = {0.23, -0.32, 1.35};
            Transform2D tf = integrate_twist(tw);

            REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinRel(-.472, 0.001));
            REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinRel(1.301, 0.001));
            REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinRel(0.23, 0.001));
        }

        SECTION("Tests from Stella"){ // Stela Yu
            // Only Translation
            Twist2D tw = Twist2D{0, 1, 2};
            Transform2D Tbbprime = integrate_twist(tw);
            REQUIRE_THAT(Tbbprime.translation().x, Catch::Matchers::WithinAbs(1.0, 1e-5));
            REQUIRE_THAT(Tbbprime.translation().y, Catch::Matchers::WithinAbs(2.0, 1e-5));
            REQUIRE_THAT(Tbbprime.rotation(), Catch::Matchers::WithinAbs(0.0, 1e-5));

            // Only Rotation
            tw = Twist2D{PI, 0, 0};
            Tbbprime = integrate_twist(tw);
            REQUIRE_THAT(Tbbprime.translation().x, Catch::Matchers::WithinAbs(0.0, 1e-5));
            REQUIRE_THAT(Tbbprime.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
            REQUIRE_THAT(Tbbprime.rotation(), Catch::Matchers::WithinAbs(PI, 1e-5));
            
            // Translation + Rotation
            tw = Twist2D{-1.24, -2.15,-2.92};
            Tbbprime = integrate_twist(tw);
            REQUIRE_THAT(Tbbprime.translation().x, Catch::Matchers::WithinAbs(-3.229863264722, 1e-5));
            REQUIRE_THAT(Tbbprime.translation().y, Catch::Matchers::WithinAbs(-1.05645265317421, 1e-5));
            REQUIRE_THAT(Tbbprime.rotation(), Catch::Matchers::WithinAbs(-1.24, 1e-5));
        }
    }

