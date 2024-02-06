#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.
#include <sstream>
#include <string>
#include "turtlelib/geometry2d.hpp"

// do not use #define, instead use constexpr
#define ppi 96
#define centerx 408.0
#define centery 528.0

namespace turtlelib{
    class Drawer{
    private:
        /// \brief String stream to store svg data in
        std::stringstream svgData; 

        /// \brief filename to save the svg to
        std::string filename;

        /// \brief convert a point into pixels
        /// \param point point to convert
        /// \return pixel corresponding to the point
        Point2D convertToPixel(Point2D point);

    public:
        /// \brief create a drawing
        /// \param name filename of the svg file
        Drawer(std::string name);

        /// \brief Save the file
        /// \return the filename the svg was saved in
        std::string save(); // does not leave this object in a valid state

        /// \brief Draw a circle
        /// \param center center point of the circle
        /// \param stroke color of the outline of the circle
        /// \param fill fill of the circle
        /// \return 0 for success
        // don't use return codes throw an exception on error
        // const std::string & fill
        int circle(Point2D center, std::string stroke, std::string fill);

        /// \brief Draw a vector
        /// \param head head of the vector
        /// \param tail tail of the vector
        /// \param stroke color of the vector
        /// \return 0 for success
        // don't use return codes throw an exception on error
        // const std::string & stroke
        int vector(Point2D head, Point2D tail, std::string stroke);

        /// \brief Draw a 2d coordinate frame
        /// \param center center of the coordinate frame
        /// \param x Direction of the x axis
        /// \param y Direction of the y axis
        /// \param text name of the frame
        /// \param textLocation coordinate for the location of the frame name
        /// \return 0 for success
        int frame(Point2D center, Point2D x, Point2D y, std::string text, Point2D textLocation);

    };
    
}
#endif
