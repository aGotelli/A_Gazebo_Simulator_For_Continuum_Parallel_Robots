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

#include <cpr_visual/visual_plugin.h>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/DynamicLines.hh>


template <typename T>
inline T read(sdf::ElementPtr sdf, std::string key, T fallback)
{
  if(sdf->HasElement(key))
    return sdf->Get<T>(key);
  return fallback;
}




namespace gazebo
{


void RodVisualPlugin::Load(rendering::VisualPtr parent, sdf::ElementPtr sdf)
{
  inv_scale = Vector3d(1./parent->Scale().X(),
                       1./parent->Scale().Y(),
                       1./parent->Scale().Z());

  visual = parent;
  //  Initialize each of the lines
  for(auto& line : lines){
    line = visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
    line->setMaterial("Gazebo/Red");
    line->setVisibilityFlags(GZ_VISIBILITY_GUI);
    // Add empty points to the line (initialization)
    for(unsigned int i = 0; i < cpr_visual::VISUAL_PROPERTIES::POINTS_PER_CENTERLINE; ++i)
      line->AddPoint(inv_scale*Vector3d(0, 0, 0));
    line->Update();
  }

//  sub_to_shape = nh.subscribe<robot_messages::rod_shape>("/rod_shape", 1, &RodVisualPlugin::callback, this);

  // connect update function
  update_event = event::Events::ConnectPreRender(std::bind(&RodVisualPlugin::Update, this));
}


void RodVisualPlugin::Update()
{
  auto msg = shape_listener.lastVisual();

  //  If no shape to define, then nothing to do here
  if(msg.shapes_to_define == 0)
    return;

  //  Loop in the useful shapes
  for(unsigned int i=0; i<msg.shapes_to_define; i++){
    for(unsigned int j=0; j<cpr_visual::VISUAL_PROPERTIES::POINTS_PER_CENTERLINE; j++){
      lines[i]->SetPoint(j, inv_scale*Vector3d(msg.rod_centerlines[i].points[j].x,
                                               msg.rod_centerlines[i].points[j].y,
                                               msg.rod_centerlines[i].points[j].z));
    }
    //  Update the newly defined line
    lines[i]->Update();
  }

}



} //  END namespace gazebo
