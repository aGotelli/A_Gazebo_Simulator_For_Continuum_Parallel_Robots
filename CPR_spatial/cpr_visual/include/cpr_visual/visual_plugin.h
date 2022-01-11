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


#ifndef VISUAL_PLUGIN_H
#define VISUAL_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <cpr_visual/shape_listener.h>


#include <ros/ros.h>

using ignition::math::Vector3d;

namespace gazebo
{

/*!
 * \brief The RodVisualPlugin class defines the visual plugin
 *
 * The visual plugin make it possible to displays the rods centerlines in the Gazebo simulation.
 * This is achieved by reading the shape message computed in RobotPhysics using the ShapeListener.
 *
 * The lines in Gazebo are "plotted" with respect to the local frame of the link hosting the plugin.
 * Hence, is recommended to use the plugin with a link fixed and coincident with the reference frame.
 */
class RodVisualPlugin : public VisualPlugin
{
public:
    RodVisualPlugin() {}

    /*!
     * \brief Load loads and initializes the plugin
     * \param parent the pointer to the hosting link
     * \param _sdf the pointer to the sdf containing the model
     *
     * This function initializes the plugin and sets the function Update() to be
     * called at each iteration.
     */
    void Load(rendering::VisualPtr parent, sdf::ElementPtr _sdf) override;

    /*!
     * \brief Update main visualization function, publishes the lines in gazebo
     *
     * First we read the visualization message wrote in the dedicated socket.
     * Then we use the informations to define the lines to plot.
     * Finally it plots the defined lines.
     */
    void Update();

protected:
    //  Account for the dimensions of the link hosting the plugin
    Vector3d inv_scale;

    //  Pointer to the visual of the hosting link
    rendering::VisualPtr visual;

    //  Instance of the listener for obtaining the rod centerlines
    cpr_visual::ShapeListener shape_listener;

    event::ConnectionPtr update_event;

    //  Array of lines to show in Gazebo
    std::array<rendering::DynamicLines*, cpr_visual::VISUAL_PROPERTIES::MAX_NUMBER_OF_RODS> lines;
    unsigned int points = 201;

};

GZ_REGISTER_VISUAL_PLUGIN(RodVisualPlugin)

} //  END namespace gazebo


#endif // VISUAL_PLUGIN_H
