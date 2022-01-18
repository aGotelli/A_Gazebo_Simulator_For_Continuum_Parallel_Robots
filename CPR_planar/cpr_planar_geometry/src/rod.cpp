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



#include "cpr_planar_geometry/rod.h"

using namespace Eigen;

namespace cpr_planar_geometry {


void Rod::cosseratRodOde(const state_type& y , state_type &dyds , const double /* t */ ) const
{
  //Unpack state vector
  const Rotation2D<double> R(y(2));
  const Vector2d n = y.segment<2>(3);
  const double m = y(5);

  //Hard-coded material constitutive equation w/ no precurvature
  const Vector2d v = Kse.inverse()*R.toRotationMatrix().transpose()*n + Vector2d::UnitY();

  //ODEs
  const Vector2d p_s = R*v;
  const double theta_s = m/Kbt;
  const Vector2d n_s = Vector2d::Zero();
  const double m_s = - p_s(0)*n(1) + p_s(1)*n(0);

  //Pack state vector derivative
  dyds << p_s, theta_s, n_s, m_s;
}

void Rod::operator()(const state_type& y , state_type &dyds , const double) const
{
  //Unpack state vector
  const Rotation2D<double> R(y(2));
  const Vector2d n = y.segment<2>(3);
  const double m = y(5);

  //Hard-coded material constitutive equation w/ no precurvature
  const Vector2d v = Kse.inverse()*R.toRotationMatrix().transpose()*n + Vector2d::UnitY();

  //ODEs
  const Vector2d p_s = L*(R*v);
  const double theta_s = L*(m/Kbt);
  const Vector2d n_s = Vector2d::Zero();
  const double m_s = - L*(p_s(0)*n(1) - p_s(1)*n(0));

  //Pack state vector derivative
  dyds << p_s, theta_s, n_s, m_s;
}


cpr_planar_visual::RodCenterline Rod::IntegrateWithObserver(state_type &y)
{

  std::vector<double> sequence;
  sequence.reserve(cpr_planar_visual::VISUAL_PROPERTIES::POINTS_PER_CENTERLINE);

  const double step = (integration_end - integration_begin)/(cpr_planar_visual::VISUAL_PROPERTIES::POINTS_PER_CENTERLINE-1);
  for(unsigned int i=0; i<cpr_planar_visual::VISUAL_PROPERTIES::POINTS_PER_CENTERLINE; i++)
    sequence.push_back( i*step );

  index = 0;

  integrate_times(stepper(), boost::ref( *this ),
                  y, sequence.begin(), sequence.end(), ds,
                  [this](const state_type& y, const double& s){this->IntegratorObserver(y, s);} );

  return centerline;
}


void Rod::IntegratorObserver(const state_type& y, const double&) const
{
  centerline.points[index++] = cpr_planar_visual::Point(y[0], y[1], 0.05);
}






} //  END namespace cpr_planar_geometry


