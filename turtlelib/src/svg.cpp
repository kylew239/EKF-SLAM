#include <iostream>
#include <fstream>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"

namespace turtlelib{
    Drawer::Drawer(std::string name){
        // Various spaces and new lines added to make the output svg look nicer
        svgData << "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n";
        svgData << "<defs><marker style=\"overflow:visible\" id=\"Arrow1Sstart\" refX=\"0.0\" refY=\"0.0\" orient=\"auto\">\n";
        svgData << "        <path transform=\"scale(0.2) translate(6,0)\"\n\
              style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"\n\
              d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"/>\n\
      </marker>\n</defs>\n";
        filename = name;
    }

    std::string Drawer::save(){
        svgData << "</svg>";
        std::ofstream file;
        file.open(filename);
        file << svgData.rdbuf();
        file.close();
        return filename;
    }

    int Drawer::circle(Point2D center, std::string stroke, std::string fill){
        Point2D pixel = convertToPixel(center);
        svgData << "<circle cx=\"" << pixel.x;
        svgData << "\" cy=\"" << pixel.y;
        svgData << "\" r=\"3\" ";
        svgData << "stroke=\"" << stroke << "\" ";
        svgData << "fill=\"" << fill << "\" ";
        svgData << "stroke-width=\"1\" />\n";
        return 0;
    }

    int Drawer::vector(Point2D head, Point2D tail, std::string stroke){
        Point2D pixelH = convertToPixel(head);
        Point2D pixelT = convertToPixel(tail);
        svgData << "<line x1=\"" << pixelH.x << "\" x2=\"" << pixelT.x << "\" ";
        svgData << "y1=\"" << pixelH.y << "\" y2=\"" << pixelT.y << "\" ";
        svgData << "stroke=\"" << stroke << "\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\n";
        return 0;
    }

    int Drawer::frame(Point2D center, Point2D x, Point2D y, std::string text, Point2D textLocation){
        Point2D textP = convertToPixel(textLocation);
        svgData << "<g>\n";
        vector(x, center, "red");
        vector(y, center, "green");
        svgData << "<text x=\"" << textP.x << "\" y=\"" << textP.y << "\">{";
        svgData << text << "}</text>\n</g>\n";
        return 0;
    }

    Point2D Drawer::convertToPixel(Point2D point){
        return Point2D{point.x*ppi + centerx,
                       -point.y*ppi + centery};
    }
}