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



#ifndef ROD_H
#define ROD_H
#include <memory>
#include <Eigen/Dense>

#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/config.hpp>
#include <boost/numeric/odeint/stepper/bulirsch_stoer.hpp>
#include <boost/numeric/odeint/stepper/bulirsch_stoer_dense_out.hpp>


#include "cpr_planar_visual/shape_msg.h"
#include <valarray>

using namespace boost::numeric::odeint;

namespace cpr_planar_geometry {

//  Declare the state type which will be handled by the odeint integrator
typedef Eigen::Matrix<double, 6, 1 > state_type;

/*!
 * \brief The Rod class defines a rod in 2D coordinate system.
 *
 * This class provides the geometrical and material proprerties of a rod as well
 * as the ODE system described by the Cosserat rod theory.
 */
class Rod
{
public:
  Rod()=default;

  Rod(const double &E_,const double &G_,
      const double &radius_,const double &L_,
      const double &ds_) : E(E_), G(G_), rad(radius_), L(L_), ds(ds_) {}

  //Rod(rod_visual::UniqueShape rod_shape_) : rod_shape( std::make_unique<rod_visual::UniqueShape>(rod_shape_) ) {}


  /*!
   * \brief cosseratRodOde member function describing the rod ODE system
   * \param y current state vector for a cross section
   * \param dxdt derivative of the state vector wrt the arc-length
   *
   * This function is used in the odeint integrator. It computes the derivative
   * of the state vector based on its current values and the rod properties.
   */
  void cosseratRodOde(const state_type& y , state_type &dyds , const double) const;

  /*!
   * \brief operator () member function describing the rod ODE system
   * \param y current state vector for a cross section
   * \param dxdt  derivative of the state vector wrt the arc-length
   *
   * This function is used in the odeint integrator. It computes the derivative
   * of the state vector based on its current values and the rod properties.
   */
  void operator()(const state_type& y , state_type &dyds , const double) const;


  inline void Integrate(state_type &y){ integrate_const(stepper(), boost::ref( *this ), y, integration_begin, integration_end, ds); }


  cpr_planar_visual::RodCenterline IntegrateWithObserver(state_type &y);


  void SetLenght(const double &L_) const {L = L_;}

private:
  //Independent Parameters
  const double E    { 200e9 };
  const double G    { 80e9 };
  const double rad  { 0.001 };
  mutable double L    { 1.0 };

  //Dependent parameter calculations
  const double A { M_PI*pow(rad, 2) };
  const double I = M_PI*pow(rad,4)/4;
  const Eigen::DiagonalMatrix<double, 2> Kse = Eigen::DiagonalMatrix<double, 2>(G*A,E*A);
  const double Kbt = E*I;

  //  The parameters of the problem
  const double ds { 0.005 };

  constexpr static double integration_begin { 0.0 };
  constexpr static double integration_end { 1.0 };

  //  The used integrator
  typedef runge_kutta_dopri5< state_type, double, state_type, double, vector_space_algebra> stepper;

  mutable cpr_planar_visual::RodCenterline centerline;

  mutable unsigned int index;
  void IntegratorObserver(const state_type &y, const double&) const;

};


} //  END namespace cpr_planar_geometry



#endif // ROD_H

