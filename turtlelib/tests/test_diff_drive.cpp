#include "turtlelib/diff_drive.hpp"

#include <catch2/catch_test_macros.hpp> // Test Cases
#include <catch2/matchers/catch_matchers_floating_point.hpp> // Floating point matchers

using namespace turtlelib;


// All test cases use the turtlebot's track width and wheel radius
TEST_CASE("Drive forward", "[diff drive]"){
    SECTION("Forward Kinematics"){
        DiffDrive robot = {
            0.16,
            0.033,
            {0.0, 0.0, 0.0}
        };

        state config1 = robot.fk(PI, PI);
        REQUIRE_THAT(config1.x, Catch::Matchers::WithinAbs(0.104, 0.001));
        REQUIRE_THAT(config1.y, Catch::Matchers::WithinAbs(0, 0.001));
        REQUIRE_THAT(config1.th, Catch::Matchers::WithinAbs(0, 0.001));

        state config2 = robot.fk(-PI, -PI);
        REQUIRE_THAT(config2.x, Catch::Matchers::WithinAbs(-0.104, 0.001));
        REQUIRE_THAT(config2.y, Catch::Matchers::WithinAbs(0, 0.001));
        REQUIRE_THAT(config2.th, Catch::Matchers::WithinAbs(0, 0.001));
    }

    SECTION("Inverse Kinematics"){
        DiffDrive robot = {
            0.16,
            0.033,
            {0.0, 0.0, 0.0}
        };

        // Test movement in just the +x direction
        Twist2D tw1 = {
            0.0,
            PI,
            0.0
        };

        wheels vel1 = robot.ik(tw1);
        REQUIRE_THAT(vel1.r, Catch::Matchers::WithinRel(95.200, 0.001));
        REQUIRE_THAT(vel1.l, Catch::Matchers::WithinRel(95.200, 0.001));

        // Test movement in just the -x direction
        Twist2D tw2 = {
            0.0,
            -PI,
            0.0
        };

        wheels vel2 = robot.ik(tw2);
        REQUIRE_THAT(vel2.r, Catch::Matchers::WithinRel(-95.200, 0.001));
        REQUIRE_THAT(vel2.l, Catch::Matchers::WithinRel(-95.200, 0.001));
    }
}

TEST_CASE("Pure rotation", "[diff drive]"){
    SECTION("Forward Kinematics"){
        DiffDrive robot = {
            0.16,
            0.033,
            {0.0, 0.0, 0.0}
        };

        state config1 = robot.fk(PI, -PI);
        REQUIRE_THAT(config1.x, Catch::Matchers::WithinAbs(0, 0.001));
        REQUIRE_THAT(config1.y, Catch::Matchers::WithinAbs(0, 0.001));
        REQUIRE_THAT(config1.th, Catch::Matchers::WithinAbs(-0.648, 0.001));

        state config2 = robot.fk(-PI, PI);
        REQUIRE_THAT(config2.x, Catch::Matchers::WithinAbs(0, 0.001));
        REQUIRE_THAT(config2.y, Catch::Matchers::WithinAbs(0, 0.001));
        REQUIRE_THAT(config2.th, Catch::Matchers::WithinAbs(0.648, 0.001));
    }

    SECTION("Inverse Kinematics"){
        DiffDrive robot = {
            0.16,
            0.033,
            {0.0, 0.0, 0.0}
        };
        Twist2D tw1 = {
            PI,
            0.0,
            0.0
        };
        Twist2D tw2 = {
            -PI/2.0,
            0.0,
            0.0
        };

        wheels vel1 = robot.ik(tw1);
        wheels vel2 = robot.ik(tw2);

        // For pure rotation, the wheels must rotate the same amount in the opposite direction
        REQUIRE_THAT(vel1.r, Catch::Matchers::WithinRel(-vel1.l, 0.001));
        REQUIRE_THAT(vel2.r, Catch::Matchers::WithinRel(-vel2.l, 0.001));
    }
}

TEST_CASE("Circle Arc", "[diff drive]"){
    SECTION("Forward Kinematics"){
        DiffDrive robot = {
            0.16,
            0.033,
            {0.0, 0.0, 0.0}
        };

        state config1 = robot.fk(PI, 2*PI);
        REQUIRE_THAT(config1.x, Catch::Matchers::WithinAbs(0.152, 0.001));
        REQUIRE_THAT(config1.y, Catch::Matchers::WithinAbs(0.025, 0.001));
        REQUIRE_THAT(config1.th, Catch::Matchers::WithinAbs(0.324, 0.001));

        state config2 = robot.fk(2*PI, PI);
        REQUIRE_THAT(config2.x, Catch::Matchers::WithinAbs(0.153, 0.001));
        REQUIRE_THAT(config2.y, Catch::Matchers::WithinAbs(0.025, 0.001));
        REQUIRE_THAT(config2.th, Catch::Matchers::WithinAbs(-0.324, 0.001));
    }

    SECTION("Inverse Kinematics"){
        DiffDrive robot = {
            0.16,
            0.033,
            {0.0, 0.0, 0.0}
        };

        Twist2D tw = {
            PI,
            -PI/2,
            0.0
        };

        wheels vel = robot.ik(tw);
        REQUIRE_THAT(vel.r, Catch::Matchers::WithinRel(-32.368, 0.001));
        REQUIRE_THAT(vel.l, Catch::Matchers::WithinRel(-62.832, 0.001));
    }
}

TEST_CASE("Impossible to follow", "[diff drive]"){
    DiffDrive robot = {
        0.16,
        0.033,
        {0.0, 0.0, 0.0}
    };

    // Test movement in just the y direction
    Twist2D tw = {
        0.0,
        0.0,
        1.0
    };

    REQUIRE_THROWS_AS(robot.ik(tw), std::logic_error);
}