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

#ifndef SHAPE_LISTENER_H
#define SHAPE_LISTENER_H


#include "cpr_visual/shape_msg.h"

#include <chrono>
#include <thread>


namespace cpr_visual
{

class ShapeListener
{
public:
  ShapeListener(std::chrono::milliseconds timeout = std::chrono::milliseconds{10});
  ~ShapeListener();

  /*!
   * \brief lastVisual returns the last message read from the socket
   * \return a ShapeMsg with the rod shapes
   */
  inline ShapeMsg lastVisual() const { return msg; }

private:
  //  Instance of the message to read
  ShapeMsg msg;

  //  Thead to allocate for reading data from the socket
  std::thread listener;

  //  Timeout to exit
  std::chrono::milliseconds timeout;
  //  Flag to stop the reading loop
  bool running = true;

  /*!
   * \brief listening_loop is a looping function reading messages from the socket
   */
  void listening_loop();

};

}  //  END namespace cpr_visual

#endif // SHAPE_LISTENER_H
