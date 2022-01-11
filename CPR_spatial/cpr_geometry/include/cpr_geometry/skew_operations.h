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

#ifndef SKEW_OPERATIONS_H
#define SKEW_OPERATIONS_H

#include <Eigen/Dense>

namespace cpr_geometry {

/*! Maps a vector in R3 to a 3x3 skew-symettric matrix in so3. */
inline const Eigen::Matrix3d hat(Eigen::Vector3d y)
{
    Eigen::Matrix3d y_hat;
    y_hat <<    0, -y(2),  y(1),
             y(2),     0, -y(0),
            -y(1),  y(0),     0;

    return y_hat;
}

/*! Maps a matrix in so3 to a vector in R3. The input should be skew-symmetric but is not validated. */
inline const Eigen::Vector3d inv_hat(Eigen::Matrix3d y_hat)
{
    Eigen::Vector3d y;
    y << y_hat(2,1), y_hat(0,2), y_hat(1,0);

    return y;
}

}


#endif // SKEW_OPERATIONS_H
