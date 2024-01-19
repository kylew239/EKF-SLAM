#include "turtlelib/svg.hpp"

#include <catch2/catch_test_macros.hpp> // Test Cases
#include <sstream> // for testing stream insertion/extraction
#include <fstream>
#include <iostream>
#include <string>

using namespace turtlelib;


TEST_CASE("Create SVG", "[svg]"){
    // Making svg
    std::string svgString = "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\
<defs><marker style=\"overflow:visible\" id=\"Arrow1Sstart\" refX=\"0.0\" refY=\"0.0\" orient=\"auto\">\
        <path transform=\"scale(0.2) translate(6,0)\"\
              style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"\
              d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"/>\
      </marker>\
</defs>\
<circle cx=\"292.8\" cy=\"480\" r=\"3\" stroke=\"purple\" fill=\"purple\" stroke-width=\"1\" />\
<circle cx=\"408\" cy=\"528\" r=\"3\" stroke=\"black\" fill=\"black\" stroke-width=\"1\" />\
<line x1=\"523.2\" x2=\"532.8\" y1=\"480\" y2=\"403.2\" stroke=\"purple\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\
<g>\
<line x1=\"628.8\" x2=\"532.8\" y1=\"403.2\" y2=\"403.2\" stroke=\"red\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\
<line x1=\"532.8\" x2=\"532.8\" y1=\"307.2\" y2=\"403.2\" stroke=\"green\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\
<text x=\"504\" y=\"412.8\">{a}</text>\
</g>\
</svg>";
    Drawer svg("temp.svg");
    svg.circle(Point2D{-1.2, 0.5}, "purple", "purple");
    svg.circle(Point2D{0.0, 0.0}, "black", "black");
    svg.vector(Point2D{1.2, 0.5}, Point2D{1.3,1.3}, "purple");
    svg.frame(Point2D{1.3,1.3}, Point2D{2.3,1.3}, Point2D{1.3,2.3}, "a", Point2D{1.0, 1.2});
    svg.save();

    // Testing svg
    std::ifstream file;
    std::string temp;
    std::string fstring;
    file.open("temp.svg");
    while(file.good()){
        std::getline(file, temp);
        fstring += temp;
    }
    REQUIRE(fstring == svgString);
}
