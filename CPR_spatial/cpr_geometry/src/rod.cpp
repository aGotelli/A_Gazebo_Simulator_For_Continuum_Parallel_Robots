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

#include "cpr_geometry/rod.h"

using namespace Eigen;

namespace cpr_geometry {


void Rod::operator()(const state_type& y , state_type &dyds , const double) const
{
/* The state vector is composed as follows
 * [  px, py, pz, r11, ..., r33, nx, ny, nz, mx, my, mz  ]
 * [   0,  1,  2,   3, ...,  11, 12, 13, 14, 15, 16, 17  ]
 *
 * And from this we unpack the state vector
*/
  //Unpack state vector
  VectorXd rr = y.segment<9>(3);
  Matrix3d R = Map<Matrix3d>(rr.data());

  Vector3d n = y.segment<3>(12);
  Vector3d m = y.segment<3>(15);

  //Hard-coded material constitutive equation w/ no precurvature
  Vector3d v = Kse.inverse()*R.transpose()*n + Vector3d::UnitZ();
  Vector3d u = Kbt.inverse()*R.transpose()*m;

  //ODEs
//  Vector3d p_s = R*v;
//  Matrix3d R_s = R*hat(u);
//  Vector3d n_s = -rho*A*g;
//  Vector3d m_s = -p_s.cross(n);
  Vector3d p_s = L*(R*v);
  Matrix3d R_s = L*(R*hat(u));
  Vector3d n_s = L*(-rho*A*g);
  Vector3d m_s = L*(-p_s.cross(n));

  //Pack state vector derivative
  dyds << p_s, Map<VectorXd>(R_s.data(), 9), n_s, m_s;
}



cpr_visual::RodCenterline Rod::IntegrateWithObserver(state_type &y)
{

  std::vector<double> sequence;
  sequence.reserve(cpr_visual::VISUAL_PROPERTIES::POINTS_PER_CENTERLINE);

//  const double step = L/(robot_visual::VISUAL_PROPERTIES::POINTS_PER_CENTERLINE-1);
  const double step = (integration_end-integration_begin)/(cpr_visual::VISUAL_PROPERTIES::POINTS_PER_CENTERLINE-1);
  for(unsigned int i=0; i<cpr_visual::VISUAL_PROPERTIES::POINTS_PER_CENTERLINE; i++)
    sequence.push_back( i*step );

  index = 0;

  integrate_times(stepper(), boost::ref( *this ), y, sequence.begin(), sequence.end(), ds,
                  [this](const state_type& y, const double& s){this->IntegratorObserver(y, s);} );


  return centerline;
}


void Rod::IntegratorObserver(const state_type& y, const double &) const
{
  centerline.points[index++] = cpr_visual::Point(y[0], y[1], y[2]);
}






} //  END namespace cpr_geometry


