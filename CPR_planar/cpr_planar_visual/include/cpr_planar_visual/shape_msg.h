/*!
MIT License

Copyright (c) 2022 Gotelli Andrea

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


#ifndef SHAPE_MSG_H
#define SHAPE_MSG_H


#include <map>
#include <array>
#include <vector>
#include <string>
#include <memory>
#include <cassert>
#include <iostream>


namespace cpr_planar_visual
{


/*!
 * \brief socket_name defines the name of the socket where writing/reading the visualization message
 */
const std::string socket_name = "ipc://@rod_centerlines";
//const std::string socket_name = "ipc:///tmp/rod_centerlines/data";

/*!
 * \brief points_per_centerline is the number of points defining a rod centerline
 */
const unsigned int points_per_centerline = 100;


/*!
 * \brief number_of_rods defines how many rod are represented in the visualization message.
 */
const unsigned int number_of_rods = 6;

/*!
 * \brief The PROPERTIES enum contains properties for the visualization
 */
enum VISUAL_PROPERTIES {

  POINTS_PER_CENTERLINE = 101,  //  How many points define a rod centerline
  MAX_NUMBER_OF_RODS = 6            //  How many rod centerlines can be stored in the visualization message

};

/*!
 * \brief The Point struct used to represent the rod centerline in Gazebo
 *
 * This structure consists in three coordinates only. They define the position of
 * a point in the space.
 */
struct Point {
  //  Default initialization (cannot forgot to initialize)
  Point()=default;

  //  Coordinates initialization
  Point(const double &x_, const double &y_, const double &z_=0) : x(x_), y(y_), z(z_) {}

  //  Set of points
  double x { 0.0 };
  double y { 0.0 };
  double z { 0.0 };
};

/*!
 * \brief The RodCenterline struct defines the shape of a rod centerline
 *
 * This structure represents the rod cenerline throught a series of Point.
 * The number of point is fixed to 100.
 */
struct RodCenterline {
  std::array<Point, VISUAL_PROPERTIES::POINTS_PER_CENTERLINE> points;
};


/*!
 * \brief The ShapeMsg struct contains the shapes of many rods
 *
 */
struct ShapeMsg
{
  ShapeMsg()=default;

  //  Set how many rods there are in the simulation
  unsigned int shapes_to_define { 0 };

  //  Array of several rod centerlines
  std::array<RodCenterline, 6> rod_centerlines;
//  std::array<RodCenterline, PROPERTIES::NUMBER_OF_RODS> rod_centerlines;
};




/*
 * This is a protoype of class that I was thinking to use.
 * The main thing is that it accounts for the maximum shapes that can be defined
 * at its initialization. Also, returns smart pointers to avoid memory leaks.
 * It might be useful as the application will keep running and quite a lot of
 * long arrays will be defined and transferred.
 *
 */
typedef std::array<RodCenterline, VISUAL_PROPERTIES::MAX_NUMBER_OF_RODS> RodsCenterlines;

class ShapesMessage
{
  ShapesMessage()=default;

  ShapesMessage(const unsigned int shapes_to_define_) : shapes_to_define(shapes_to_define_)
  {
    //  Check number of shapes to define it smaller the the maximum. Otherwise terminate.
    assert(("You defined too many centerlines", shapes_to_define <= VISUAL_PROPERTIES::MAX_NUMBER_OF_RODS));
  }

  inline std::unique_ptr<RodsCenterlines> GetRodsCenterlines() const {return std::make_unique< RodsCenterlines >(rod_centerlines);}

  inline void SetRodsCenterlines(const RodsCenterlines &rod_centerlines_) const {rod_centerlines = rod_centerlines_;}

private:
  //  Set how many rods there are in the simulation
  const unsigned int shapes_to_define { 0 };

  //  Array of several rod centerlines
  mutable RodsCenterlines rod_centerlines;

};

} //  END namespace cpr_planar_visual

#endif // SHAPE_MSG_H
